from pymavlink import mavutil
import math
import time

# HEARTBEAT: Establish connection
print("Connecting to BCI-Drone Pipeline...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# HEARTBEAT CHECK: Script-drone communication pipeline check
master.wait_heartbeat()
print("Heartbeat Received! System is healthy.")
last_heartbeat = time.time()

# GPS: Store the starting position as "Home" for the Geofence
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
home_lat = msg.lat / 1e7
home_lon = msg.lon / 1e7
home_alt = msg.alt / 1000
print(f"Home set to: {home_lat}, {home_lon}")

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of Earth in meters
    p1, p2 = math.radians(lat1), math.radians(lat2)
    l1, l2 = math.radians(lon1), math.radians(lon2)
    dp, dl = p2 - p1, l2 - l1
    a = math.sin(dp/2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def set_home_location(master, lat, lon, alt):
    # MAV_CMD_DO_SET_HOME (179)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,    # Confirmation
        0,    # Param 1: 1 = Use current, 0 = Use specified
        0, 0, 0, # Unused params
        lat, lon, alt # Target coordinates
    )
    print(f"Home location locked at: {lat}, {lon}")

def take_off(master, altitude):
    """
    Automates the sequence: GUIDED Mode -> ARM -> TAKEOFF
    """
    print(f"Initiating Takeoff to {altitude}m...")

    # 1. Ensure we are in GUIDED mode
    # master.flightmode stores the current mode string
    if master.flightmode != 'GUIDED':
        print("Switching to GUIDED mode...")
        master.set_mode('GUIDED')
        # Wait for mode to change
        while master.flightmode != 'GUIDED':
            master.wait_heartbeat()
            time.sleep(0.5)

    # 2. ARM the motors
    # MAV_CMD_COMPONENT_ARM_DISARM: param1=1 to arm, param1=0 to disarm
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    # Wait for the drone to confirm it is armed
    master.motors_armed_wait()
    print("Armed!")

    # 3. Send the TAKEOFF command
    # MAV_CMD_NAV_TAKEOFF: Param 7 is altitude
    print(f"Climbing to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )


# Set spawn point as home
set_home_location(master, home_lat, home_lon, home_alt)

'''
def send_waypoint(master, lat, lon, alt):
    master.mav.mission_item_int_send(
        master.target_system, 
        master.target_component,
        0,                      # Sequence (not used for direct commands)
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Use altitude relative to home
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,           # THE MESSAGE ID (16)
        2,                      # Current (2 = Guided mode target)
        0,                      # Autocontinue
        0, 2, 0, 0,             # Params 1-4 (Hold time, radius, etc.)
        int(lat * 1e7),         # Param 5: Latitude
        int(lon * 1e7),         # Param 6: Longitude
        alt                     # Param 7: Altitude
    )
    print(f"Sent Waypoint Packet: {lat}, {lon} at {alt}m")
'''
take_off(master, 15)

# State machine flag
breach_handled = False 

try:
    while True:
        msg = master.recv_match(blocking=False)

        # PROCESS MESSAGES (If they exist)
        if msg:
            msg_type = msg.get_type()
            if msg_type == 'HEARTBEAT':
                last_heartbeat = time.time()
            elif msg_type == 'GLOBAL_POSITION_INT':
                curr_lat, curr_lon = msg.lat / 1e7, msg.lon / 1e7
                dist = haversine_distance(home_lat, home_lon, curr_lat, curr_lon)
                # Keep these variables accessible outside this 'if msg' block
                last_dist = dist
                last_alt = msg.relative_alt

        if time.time() - last_heartbeat > 3.0:
            print("Heartbeat not detected: Commanding Emergency Land")
            raise Exception("Heartbeat Timeout")
        
        # Get current mode from the heartbeat
        current_mode = master.flightmode

        print(f"Dist: {dist:.2f}m | Alt: {msg.relative_alt/1000:.1f}m | Mode: {current_mode}")

        if time.time() - last_heartbeat > 3.0:
            print("Heartbeat not detected: Commanding Emergency Land")
            raise

        # Geofence Logic
        if dist > 30.0 and not breach_handled:
            print("Geofence breach: Commanding RTL...")
            master.set_mode('RTL')
            breach_handled = True # Prevents re-sending the command
            
        # Monitor return
        if breach_handled:
            if current_mode == 'RTL':
                if dist < 3.0 and msg.relative_alt < 500: # Within 2m and 0.5m alt
                    print("Droned landed successfully.")
                    break
            else:
                # If 5 seconds pass and we still aren't in RTL, then LAND
                print("Waiting for RTL confirmation...")
                time.sleep(1) 

        time.sleep(0.5)

except Exception as e:
    # HEARTBEAT/SIGNAL LOSS FAILSAFE
    print(f"BCI Pipeline Error: {e}")
    print("EMERGENCY: Loss of Control - Landing Drone...")
    master.set_mode('LAND')