from pymavlink import mavutil
import math
import time

# ==========================================
# 1. Global States, Trackers, Variables
# ==========================================
last_heartbeat = time.time()
last_alt = 0.0
dist = 0.0
home_lat, home_lon, home_alt = 0.0, 0.0, 0.0
current_mode = "UNKNOWN"
breach_handled = False

# ==========================================
# 2. Utility and Navigation
# ==========================================
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of Earth in meters
    p1, p2 = math.radians(lat1), math.radians(lat2)
    l1, l2 = math.radians(lon1), math.radians(lon2)
    dp, dl = p2 - p1, l2 - l1
    a = math.sin(dp/2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

# ==========================================
# 3. Custom Heartbeat, GPS, and State Variable Sync
# ==========================================
def sync(master, msg_types=None, timeout=0.1):
    """
    Universal MAVLink listener. Automatically updates heartbeat 
    and state variables whenever it is called.
    """
    global last_heartbeat, last_alt, current_mode
    
    # Ensure we always listen for heartbeats to keep the watchdog fresh
    if msg_types is None:
        request = None # Listen for everything
    elif isinstance(msg_types, str):
        request = [msg_types, 'HEARTBEAT']
    else:
        request = list(set(msg_types + ['HEARTBEAT']))

    msg = master.recv_match(type=request, blocking=True, timeout=timeout)
    
    if msg:
        last_heartbeat = time.time()
        m_type = msg.get_type()
        
        if m_type == 'HEARTBEAT':
            current_mode = master.flightmode
        elif m_type == 'GLOBAL_POSITION_INT':
            last_alt = msg.relative_alt # mm
            
    return msg

# ==========================================
# 4. Flight Primitives (Wrappers)
# ==========================================
def prep_flight(master):
    """Ensures GUIDED mode and ARMED status before movement."""
    global current_mode
    
    # 1. Mode Change
    if master.flightmode != 'GUIDED':
        print("Switching to GUIDED mode...")
        master.set_mode('GUIDED')
        while current_mode != 'GUIDED':
            sync(master)

    # 2. Arming Logic
    if not master.motors_armed():
        print("Arming motors...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )
        
        start = time.time()
        while not master.motors_armed():
            # Listen for COMMAND_ACK and/or Heartbeat
            msg = sync(master, msg_types='COMMAND_ACK')
            
            # Handle COMMAND_ACK
            if msg and msg.get_type() == 'COMMAND_ACK':
                if msg.command == 400: # 400 is ARM_DISARM
                    if msg.result == 0:
                        print("Arming command ACK received.")
                    else:
                        print(f"Arming rejected: Result {msg.result}")
            
            if time.time() - start > 10:
                raise Exception("Arming Timeout")
            
def take_off(master, altitude):
    """Automated takeoff with heartbeat-safe blocking."""
    prep_flight(master)
    print(f"Climbing to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
    )

    while True:
        msg = sync(master, msg_types='GLOBAL_POSITION_INT')
        alt_m = last_alt / 1000.0
        print(f"Takeoff Alt: {alt_m:.1f}m", end='\r')
        if alt_m >= (altitude - 1.0):
            print(f"\nTakeoff Complete.")
            break

def nav_waypoint(master, target_lat, target_lon, target_alt):
    """Sends waypoint and waits for MISSION_ACK."""
    print(f"Navigating to: {target_lat}, {target_lon}")
    master.mav.mission_item_int_send(
        master.target_system, master.target_component,
        0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 2, 2, 0, 0, 
        int(target_lat * 1e7), int(target_lon * 1e7), target_alt
    )

    start = time.time()
    while time.time() - start < 5:
        msg = sync(master, msg_types='MISSION_ACK')
        if msg:
            print("Waypoint Accepted (Mission ACK)")
            break

def check_geofence(master, distance, range_limit):
    """Monitors distance from home and triggers RTL on breach."""
    global breach_handled
    if distance > range_limit and not breach_handled:
        print("\nGeofence Breach: Triggering RTL...")
        master.set_mode('RTL')
        breach_handled = True
        return True
    return False

# ==========================================
# 5. Mission Execution
# ==========================================
print("Connecting to BCI-Drone Pipeline...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

# Set Home reference
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
home_lat, home_lon = msg.lat / 1e7, msg.lon / 1e7
master.mav.command_long_send(master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 
                            home_lat, home_lon, msg.alt/1000.0)

# EKF Wait
print("Waiting for EKF/GPS Readiness...")
while not (sync(master, msg_types='SYS_STATUS').onboard_control_sensors_present & 0x400000):
    time.sleep(1)

# Sequence Start
take_off(master, 15)

lat_off = 0.0001 
lon_off = 0.0001 * 0.866

triangle_mission = [
    (home_lat + lat_off, home_lon),                     # Point A: North
    (home_lat - (lat_off/2), home_lon + lon_off),       # Point B: South-East
    (home_lat - (lat_off/2), home_lon - lon_off),       # Point C: South-West
    (home_lat + lat_off, home_lon),                     # Point A: North
    (home_lat, home_lon)                               # Return Home
]

print("\n--- Starting Triangle Mission ---")
last_heartbeat = time.time() 
for i, (wp_lat, wp_lon) in enumerate(triangle_mission):
    print(f"\nTargeting Vertex {i+1}/{len(triangle_mission)}...")
    nav_waypoint(master, wp_lat, wp_lon, 15)

    # Blocking wait until we arrive at the vertex
    while True:
        msg = sync(master, msg_types='GLOBAL_POSITION_INT')
        
        # Monitor Geofence and Heartbeat while in transit
        if time.time() - last_heartbeat > 5.0:
            raise Exception("Heartbeat Lost during Transit")
        
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            curr_lat, curr_lon = msg.lat / 1e7, msg.lon / 1e7
            dist_to_wp = haversine_distance(wp_lat, wp_lon, curr_lat, curr_lon)
            dist_from_home = haversine_distance(home_lat, home_lon, curr_lat, curr_lon)
            
            # Continuous Geofence Check
            check_geofence(master, dist_from_home, 30.0)
            
            print(f"Dist to WP: {dist_to_wp:.1f}m | Home: {dist_from_home:.1f}m", end='\r')
            
            # Arrival condition: Within 1.5 meters of target
            if dist_to_wp < 1.5:
                print(f"\nArrived at Vertex {i+1}")
                time.sleep(3) #  Hover for stability
                break

last_heartbeat = time.time() # Final reset for the main loop

# ==========================================
# 6. Post-Mission Telemetry and Connection
# ==========================================
try:
    while True:
        # Use sync() as the only reader in the loop
        msg = sync(master, msg_types='GLOBAL_POSITION_INT', timeout=0.1)

        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            curr_lat, curr_lon = msg.lat / 1e7, msg.lon / 1e7
            dist = haversine_distance(home_lat, home_lon, curr_lat, curr_lon)

        # Safety Watchdog (Check the global tracker)
        if time.time() - last_heartbeat > 5.0:
            raise Exception("Heartbeat Timeout: Link Lost")
        
        # Geofence Monitor
        check_geofence(master, dist, 30.0)

        # Telemetry
        print(f"Dist: {dist:.2f}m | Alt: {last_alt/1000:.1f}m | Mode: {current_mode}", end='\r')
        time.sleep(0.1)

except Exception as e:
    print(f"\nRC Pipeline Error: {e}")
    if current_mode not in ['LAND', 'RTL']:
        print("Safety Protocol: Forcing Land...")
        master.set_mode('LAND')