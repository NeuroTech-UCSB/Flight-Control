from pymavlink import mavutil
import math
import time

# Heartbeat: Establish connection
print("Connecting to BCI-Drone Pipeline...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Heartbeat Check: Script-drone communication pipeline check
master.wait_heartbeat()
print("Heartbeat Received")
last_heartbeat = time.time()

# GPS: Set starting position as home
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
home_lat = msg.lat / 1e7
home_lon = msg.lon / 1e7
home_alt = msg.alt / 1000

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of Earth in meters
    p1, p2 = math.radians(lat1), math.radians(lat2)
    l1, l2 = math.radians(lon1), math.radians(lon2)
    dp, dl = p2 - p1, l2 - l1
    a = math.sin(dp/2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def set_home_location(master, lat, lon, alt):
    """Explicitly sets the EKF home position."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 0, 0, 0, 0, 
        lat, lon, alt # Target coordinates
    )
    print(f"Home location locked: {lat}, {lon}")

def prep_flight(master):
    global last_heartbeat
    
    # Switch Mode to GUIDED if not already
    if master.flightmode != 'GUIDED':
        print("Switching to GUIDED mode...")
        master.set_mode('GUIDED')
        
        # Wait for COMMAND_ACK for the mode change (Command ID 11 is SET_MODE)
        start_time = time.time()
        while master.flightmode != 'GUIDED':
            # Prompting COMMAND_ACK works better than only checking .flightmode() for updates
            msg = master.recv_match(type=['COMMAND_ACK', 'HEARTBEAT'], blocking=True, timeout=1.0)
            if msg:
                last_heartbeat = time.time() # Connection still alive
                if msg.get_type() == 'COMMAND_ACK' and msg.command == 11:
                    if msg.result == 0: # 0 is MAV_RESULT_ACCEPTED
                        print("Mode change ACK received.")
                        break
            
            if time.time() - start_time > 10:
                raise Exception("Failed to switch to GUIDED mode: Timeout waiting for ACK")

    # ARM Motors
    if not master.motors_armed():
        print("Arming motors...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )
        
        # Custom motors_armed_wait loop that also checks heartbeat
        start_time = time.time()
        while not master.motors_armed():
            msg = master.recv_match(type=['COMMAND_ACK', 'HEARTBEAT'], blocking=True, timeout=1.0)
            if msg:
                last_heartbeat = time.time()
                if msg.get_type() == 'COMMAND_ACK' and msg.command == 400: # 400 is ARM_DISARM
                    print("Arming command ACK received.")
            
            if time.time() - start_time > 10:
                raise Exception("Arming failed: check Pre-Arm messages in SITL terminal")
        
        print("Armed and Dangerous!")

def take_off(master, altitude):
    """Automated takeoff sequence."""

    print(f"Initiating Takeoff to {altitude}m...")
    # Call semi-automated flight prep
    prep_flight(master)

    # Send TAKEOFF command
    print(f"Climbing to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )

    while True:
        # Wait for a position message
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        # relative_alt is in millimeters, convert to meters
        curr_alt = msg.relative_alt / 1000.0
        
        print(f"Current Altitude: {curr_alt:.1f}m", end='\r')

        # Check if we are within 1 meter of target
        if curr_alt >= (altitude - 1.0):
            print(f"\nTarget Altitude Reached: {curr_alt:.1f}m")
            break
            
        time.sleep(0.2)

def nav_waypoint(master, target_lat, target_lon, target_alt):
    global last_heartbeat
    
    # Call semi-automated flight prep
    prep_flight(master)

    # Send Waypoint
    master.mav.mission_item_int_send(
        master.target_system, master.target_component,
        0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 2, 2, 0, 0, 
        int(target_lat * 1e7), int(target_lon * 1e7), target_alt
    )
    
    # Wait for ACK (Command 16 is NAV_WAYPOINT)
    start_time = time.time()
    while True:
        msg = master.recv_match(type=['MISSION_ACK', 'HEARTBEAT'], blocking=True, timeout=1.0)
        if msg:
            last_heartbeat = time.time()
            if msg.get_type() == 'MISSION_ACK': # and msg.command == 16:
                print(f"Waypoint Accepted by Flight Controller")
                break
        
        if time.time() - start_time > 5:
            print("Waypoint ACK not received, but continuing mission...")
            break

def check_geofence(master, distance, range_limit, distance_tolerance, rel_alt, alt_tolerence):
    """Automates Geofence breach detection"""
    global breach_handled

    if distance > range_limit and not breach_handled:
        print("Geofence Breach: RTL Triggered.")
        master.set_mode('RTL')
        breach_handled = True
        raise Exception("Geofence Breach Detected")

    if breach_handled:
        if master.flightmode == 'RTL':
            if distance < distance_tolerance and rel_alt < alt_tolerence:
                print("Drone Landed Home.")
                return True
        else:
            print("Waiting for RTL confirmation...")
    
    return False

# Set spawn point as home
set_home_location(master, home_lat, home_lon, home_alt)

print("Waiting for EKF and GPS")
while True:
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    # Check if the EKF (positioning system) is connected
    # 0x400000 is the bitmask for EKF/Positioning
    if msg.onboard_control_sensors_present & 0x400000:
        print("System Ready for Flight.")
        break
    time.sleep(1)

take_off(master, 15)
nav_waypoint(master, home_lat + 0.0001, home_lon + 0.0001, 15)

print("Takeoff complete. Handing over to Mission Monitor.")
last_heartbeat = time.time() 
dist = 0.0
last_alt = 0.0
current_mode = "UNKOWN"

try:
    while True:
        msg = master.recv_match(blocking=False)

        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'HEARTBEAT':
                last_heartbeat = time.time()
                current_mode = master.flightmode 
                
            elif msg_type == 'GLOBAL_POSITION_INT':
                curr_lat, curr_lon = msg.lat / 1e7, msg.lon / 1e7
                dist = haversine_distance(home_lat, home_lon, curr_lat, curr_lon)
                last_alt = msg.relative_alt 

        if time.time() - last_heartbeat > 10:
            raise Exception("Heartbeat Timeout")
        
        # Telemetry Print
        print(f"Dist: {dist:.2f}m | Alt: {last_alt/1000:.1f}m | Mode: {master.flightmode}", end='\r')

        # check_geofence(master, dist, 30, 3, last_alt, 500)

        time.sleep(0.1)

except Exception as e:
    # Heartbeat/Signal Loss Failsafe
    print(f"RC Pipeline Error: {e}")
    if current_mode not in ['LAND', 'RTL']:
        print("Landing Drone...")
        master.set_mode('LAND')