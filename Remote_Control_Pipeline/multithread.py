from pymavlink import mavutil
import math
import time
import threading
import os

from bci import process_bci, init_bci
# ==========================================
# 1. Class of Shared States & Flags
# ==========================================
class FlightState:
    """
    This is the shared memory bank. All threads read and write to this.
    The 'Lock' prevents two threads from editing or accessing variables at the exact same time.
    """
    def __init__(self):
        # Mutex (Mutual Exclusion Object)
        self.lock = threading.Lock() # Use keyword 'with' to acquire the lock and access/update mem variables
        
        # Telemetry Data
        self.last_heartbeat = time.time()

        self.last_gcs_heartbeat_sent = 0.0

        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.flightmode = "UNKNOWN"
        self.is_armed = False

        self.ekf_ready = False
        self.rc_found = False
        self.ready_to_fly = False 

        self.onboard_control_sensors_health = 0
        self.voltage_battery = 0

        # Navigation / Home Data (Initialized to None until EKF is ready)
        self.home_lat = None
        self.home_lon = None
        self.home_alt_amsl = None
        
        # Command Acknowledgments
        self.mission_ack_received = False
        self.command_ack = None # Will store a tuple: (command_id, result)
        
        # Sensor Data
        self.user_command = "STANDBY" # Variable stub
        self.bci_command = None
        self.use_bci = False
        
        # Safety Flags
        self.geofence_breached = False
        self.geofence_range = 15.0

# ==========================================
# 2. Utility and Computation Functions
# ==========================================
def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculates the Haversine distance from location A to B
    Provides more accuracy than Euclidian distance (2D) by accounting for Earth's curvature (3D)
    """
    R = 6371000  # Radius of Earth in meters
    p1, p2 = math.radians(lat1), math.radians(lat2)
    l1, l2 = math.radians(lon1), math.radians(lon2)
    dp, dl = p2 - p1, l2 - l1
    a = math.sin(dp/2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

# ==========================================
# 3. Thread 1: Telemetry
# ==========================================
def telemetry_thread(master, state):
    """Runs continuously in the background at ~100Hz. Non-blocking process."""
    last_telem_time = time.time()
    while True:
        # 1. Receive packet (Non-Blocking)
        msg = master.recv_match(blocking=False)
        
        if msg:
            m_type = msg.get_type()
            
            # 2. Acquire Lock: Locks the memory bank for other threads while current thread executes 'with' block (avoids Race Condition)
            with state.lock:
                state.last_heartbeat = time.time()
                
                # Update gps location
                if m_type == 'GLOBAL_POSITION_INT':
                    state.lat = msg.lat / 1e7
                    state.lon = msg.lon / 1e7
                    state.alt = msg.relative_alt / 1000.0
                
                # Update flightmode
                elif m_type == 'HEARTBEAT':
                    state.flightmode = master.flightmode
                    state.is_armed = master.motors_armed()

                # Update ready to fly status
                elif m_type == 'SYS_STATUS':
                    # 0x400000 is the bitmask for EKF/Positioning active
                    state.onboard_control_sensors_health = msg.onboard_control_sensors_health
                    state.voltage_battery = msg.voltage_battery

                    #print(state.onboard_control_sensors_health)
                    if (state.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK):
                        if (not state.ready_to_fly):
                            print("System sensors healthy, ready to fly")
                        state.ready_to_fly = True


                elif m_type == 'RC_CHANNELS':
                    #print("Rc_channels")
                    if msg.chancount == 0: 
                        print("Not receiving any rc channels, please check receiver and transmitter are enabled")
                    elif msg.chancount < 6:
                        print("Not receiving enough channels for safe flight operation")
                    else:
                        if (not state.rc_found):
                            print("RC receiving channels")    
                        state.rc_found = True

                #Extended Kalman Filter Status
                elif m_type == 'EKF_STATUS_REPORT':
                    #print("received EKF_STATUS_REPORT")
                    # Check if EKF has valid attitude, velocity, and absolute position
                    ekf_healthy = (
                        (msg.flags & mavutil.mavlink.EKF_ATTITUDE) and
                        (msg.flags & mavutil.mavlink.EKF_VELOCITY_HORIZ) and
                        (msg.flags & mavutil.mavlink.EKF_VELOCITY_VERT) and
                        (msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS) and
                        (msg.flags & mavutil.mavlink.EKF_POS_VERT_ABS) and
                        (msg.flags & mavutil.mavlink.EKF_PRED_POS_HORIZ_ABS)
                    )
                    #print("EKF status",ekf_healthy)

                    if (msg.flags & mavutil.mavlink.EKF_UNINITIALIZED):
                        print("EKF uninitialized")
                    elif (msg.flags & mavutil.mavlink.EKF_GPS_GLITCHING):
                        print("EKF thinks GPS is unhealthy")
                        state.ekf_ready = False

                    elif ekf_healthy:
                        if (not state.ekf_ready):
                            print(f"Set EKF Ready {msg.flags}")
                        state.ekf_ready = True
                    else:
                        state.ekf_ready = False

                    #print(msg.flags)

                # Update mission and command acks
                elif m_type == 'MISSION_ACK':
                    state.mission_ack_received = True
                    
                elif m_type == 'COMMAND_ACK':
                    state.command_ack = (msg.command, msg.result) # (command_id, result)
        
                # Catch critical system messages (e.g., pre-arm failures, errors, warnings)
                elif m_type == 'STATUSTEXT':
                    text = msg.text
                    severity = msg.severity
                    # Severity levels: 0=EMERGENCY, 1=ALERT, 2=CRITICAL, 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG
                    # We print INFO and above (<= 6) to catch typical QGroundControl status texts
                    if severity <= 6:
                        # Clear current line if we are doing inline printing elsewhere
                        print(f"\n[SYS_MSG] {text}")
                else:
                    #print(f"m_type: {m_type}")
                    pass 

        # 3. Active Checks
        with state.lock:
            hb_gap = time.time() - state.last_heartbeat
            ekf_ready = state.ekf_ready
            curr_lat, curr_lon = state.lat, state.lon
            h_lat, h_lon = state.home_lat, state.home_lon
            breached = state.geofence_breached
            range = state.geofence_range

        # Send GCS Heartbeat (~1 Hz) to prevent GCS failsafe
        current_time = time.time()
        if (current_time - state.last_gcs_heartbeat_sent) >= 0.5:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            with state.lock:
                state.last_gcs_heartbeat_sent = current_time
            
            
        # Active Watch 1: Heartbeat
        if hb_gap > 5.0:
            print("\n[DAEMON] Radio Link Lost")
            os._exit(1) 

        # Active Watch 2: Geofence
        if h_lat is not None and ekf_ready and not breached:
            dist = haversine_distance(h_lat, h_lon, curr_lat, curr_lon)
            if dist > range:
                print(f"\n[DAEMON] Geofence Breach ({dist:.1f}m): Triggering RTL.")
                master.set_mode('RTL')
                with state.lock:
                    state.geofence_breached = True

        if (time.time() - last_telem_time > 5):
            last_telem_time = time.time()
            print_telemetry(state)

        time.sleep(0.01) # Yield
        
# ==========================================
# 4. Thread 2: BCI Sensor (Stub)
# ==========================================
def user_input_thread(state):
    #use keyboard input for stability
    print("Started User Shell:")
    while True:
        user_cmd = input(">>")
        user_cmd = user_cmd.lower()

        prev_cmd = state.user_command
        if user_cmd == "h" or user_cmd == "help":
            print("Enter Up/U to command the drone to hover or Down/D to land")
            continue

        elif user_cmd == "u" or user_cmd == "up":
            if state.use_bci:
                print("Using BCI mode, can only command LAND (enter d or down)")
            with state.lock:
                state.user_command = "HOVER"

        elif user_cmd == "d" or user_cmd == "down":
            with state.lock:
                state.user_command = "LAND"

        elif user_cmd == "b" or user_cmd == "bci":
            with state.lock:
                state.use_bci = True
            print("set input to use bci")

        elif user_cmd == "nb" or user_cmd == "nobci":
            with state.lock:
                state.use_bci = False
            print("set input to no bci")
        else:
            state.user_command = prev_cmd #keep last state
            print(f"Unknown command, keeping current user input: {state.user_command}")

        #state.previous_user_command = cmd 

def bci_thread(state):
    """Simulates reading incoming brainwave/eye-tracking data asynchronously."""

    prev_event = None
    while True:
        # Simulate processing time for an EEG/Eye-Tracker frame (e.g., 200ms)
        time.sleep(0.2) 
        
        # Placeholder for actual BCI SDK logic
        event = process_bci()
        if (event is not None and event != prev_event):
            print(f"New BCI Event: {event}")
            prev_event = event

            if (event == "RELAXED"):
                with state.lock:
                    state.bci_command = "LAND"
                print("set bci command to land")
            elif (event == "FOCUSED"):
                with state.lock:
                    state.bci_command = "HOVER"
                print("set bci command to hover")
            else:
                print(f"====Unhandled BCI event {event}====")

# ==========================================
# 5. Flight Primitives
# ==========================================
def set_home(master, state, lat, lon, alt=0):
    """Explicitly sets the EKF home position."""
    print(f"Setting Home to: {lat:.6f}, {lon:.6f}")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 
        lat, lon, alt
    )
    with state.lock:
        state.home_lat = lat
        state.home_lon = lon
    # Time for EKF to register new origin
    time.sleep(0.5)

def wait_for_arming(master,state):
    # 2. Check/Set Arming
    with state.lock:
        armed = state.is_armed
        
    while not state.is_armed:
        with state.lock:
            armed = state.is_armed
        print("Please ARM with RC Arming switch")
        time.sleep(5)
        #master.mav.command_long_send(
        #    master.target_system, master.target_component,
        #    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        #)
        #while True: # Blocking loop while motors aren't armed
        #    with state.lock:
        #        if state.is_armed: break
        #    time.sleep(0.1)
    print("Motors armed")

def prep_flight(master, state):
    """Waits for user to arme motors."""
    print("Pre-Flight Checks Initiated...")
    

    wait_for_arming(master,state)

    #wait for RC to enable guided mode
    #if current_mode != 'GUIDED':
        #master.set_mode('GUIDED')
    #    while True:
            #print("Please switch to GUIDED mode with RC switch")
    #        with state.lock:
    #            if state.flightmode == 'GUIDED': break
    #        time.sleep(5)

    #print("In guided mode")
    time.sleep(1)
    print("Pre-Flight Complete.")

def send_takeoff_command(master,state,target_alt):
    print(f"Climbing to {target_alt} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_alt
    )

def take_off(master, state, target_alt):

    # 1. Check/Set Mode
    with state.lock:
        current_mode = state.flightmode

    if current_mode != 'GUIDED':
        master.set_mode('GUIDED')
        while True:
            #print("Please switch to GUIDED mode with RC switch")
            with state.lock:
                if state.flightmode == 'GUIDED': break
            print("Waiting for autopilot to switch to guided mode")
            time.sleep(5)

    """takeoff to desired altitude"""
    send_takeoff_command(master,state,target_alt)

    while True:
        with state.lock:
            curr_alt = state.alt
            
        print(f"Current Alt: {curr_alt:.1f}m", end='\r')
        if curr_alt >= (target_alt - 1.0): # 1m buffer
            print(f"\nTakeoff Complete.")
            break
        time.sleep(0.1)

def nav_waypoint(master, state, target_lat, target_lon, target_alt):
    """Sends drone to desired waypoint. Drone must be explicitly prepped beforehand"""
    print(f"\nNavigating to: {target_lat}, {target_lon}")
    
    # Reset the ACK flag BEFORE sending the command
    with state.lock:
        state.mission_ack_received = False
        state.command_ack_received = False

    master.mav.mission_item_int_send(
        master.target_system, master.target_component,
        0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 2, 2, 0, 0, int(target_lat * 1e7), int(target_lon * 1e7), target_alt
    )

    start = time.time()
    while time.time() - start < 3: # 3 seconds to acknowledge
        with state.lock:
            if state.mission_ack_received:
                print("Waypoint Accepted (Mission ACK)")
                return True
        time.sleep(0.1)
        
    print("Waypoint ACK Timeout")
    return False

def land(master, state, rtl=False):
    """Commands a landing or Return-to-Launch, and monitors descent."""
    mode = 'RTL' if rtl else 'LAND'
    print(f"\nInitiating {mode} Sequence...")
    master.set_mode(mode)

    while True:
        with state.lock:
            curr_alt = state.alt
            armed = state.is_armed
            
        print(f"Descending... Alt: {curr_alt:.2f}m", end='\r')
        
        # Exit condition: Motors disarm automatically upon touching the ground
        if not armed or curr_alt < 0.1:
            print("\nTouchdown Confirmed. Motors Disarmed.")
            break
            
        time.sleep(0.2)


def print_telemetry(state):
    # Telemetry Output
    print(f"Alt: {state.alt:.1f}m | Mode: {state.flightmode} | USER COMMAND: {state.user_command} | BATTERY: {state.voltage_battery}",  end='\r')
    time.sleep(0.1)

# ==========================================
# 6. Thread 3: Mission (Main Loop)
# ==========================================
def test_mission_plan(master,state):
    prep_flight(master, state)

    #return
    take_off(master, state, 2) #take off to two meters
        
    delay = 5
    last = time.time()
    while (time.time()- last < delay):
        print('waiting 1 second at waypoint')
        time.sleep(1)
    print("landing")
    #nav_waypoint(master, state, home_lat + 0.0001, home_lon + 0.0001, 10.0)
    #nav_waypoint(master, state, home_lat, home_lon, 10.0)

    land(master, state, rtl=True)

def bci_mission(master,state):
    #prep_flight(master,state)

    #TODO: Move this logic into a safer state machine
    state.prev_command = "STANDBY"
    print("in bci_mission")
    while True:

        command_hover = (state.use_bci and state.bci_command == "HOVER") or (not state.use_bci and state.user_command == "HOVER")
        command_land =  (state.use_bci and state.bci_command == "LAND") or (not state.use_bci and state.user_command == "LAND")

        print(f"commands: hover: {command_hover} land: {command_land}")
        if command_hover:
            if "HOVER" != state.prev_command:
                print("Hovering hehe")

                state.prev_command = "HOVER"

                prep_flight(master,state)

                take_off(master,state,2)
            
        elif command_land:
            if "LAND" != state.prev_command:
                print("not Hovering hehe")

                state.prev_command = "LAND"
                land(master,state,2)

        else:
            pass

        time.sleep(0.25)

#listens only to user input
def demo_mission(master,state):

    state.prev_command = "STANDBY"
    while True:
        if state.user_command == "HOVER":
            if state.user_command != state.prev_command:
                print("Hovering hehe")

                state.prev_command = state.user_command
                take_off(master,state,2)
                
        else:
            if state.user_command != state.prev_command:
                print("not Hovering hehe")
                state.prev_command = state.user_command
                land(master,state,2)

def main():

    #connecting to the BCI headset:
    init_bci() #opens a serial connection to the BCI device

    # Instantiate the global state tracker FlightState object
    state = FlightState()
    # Instantiates the drone mavutil object and connects to physical drone
    print("Connecting to BCI-Drone Pipeline...")
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat() # Waits till heartbeat connects before proceeding
    print("Heartbeat Connected.")

    # Request STATUSTEXT messages
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,
        0, 0, 0, 0, 0, 0 # Interval at 0 means default stream rate
    )

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        0, 0, 0, 0, 0, 0
    )

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        0, 0, 0, 0, 0, 0
    )

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        0, 0, 0, 0, 0, 0
    )

    # Start the Background Threads
    # 'daemon=True' ensures threads will automatically terminate when  main script ends
    t_telem = threading.Thread(target=telemetry_thread, args=(master, state,), daemon=True)
    t_bci = threading.Thread(target=bci_thread, args=(state,), daemon=True)
    t_user = threading.Thread(target=user_input_thread, args=(state,), daemon = True)
   
    t_telem.start()
    t_bci.start()
    t_user.start()
    
    #handles drone state transitions based on user and BCI input
    print("Background Threads Running: Telemetry [ON], BCI [ON], USER INPUT [ON]")

    # Wait for initial GPS lock via the background thread
    print("Waiting for EKF/GPS Readiness and RC connection")
    time.sleep(1) #wait for state data to arrive
    while True:
        print_telemetry(state)

        with state.lock:
            if state.ready_to_fly: break 
            #if state.ekf_ready and state.rc_found: break
            print("not ready to fly")
        time.sleep(10)

        if not state.ekf_ready:
            print("EKF not ready")
        if not state.rc_found:
            print("No RC found")
        time.sleep(0.1)

    print("System Ready for Flight.")

    with state.lock:
        home_lat, home_lon, home_alt = state.lat, state.lon, state.alt
        state.home_lat = home_lat
        state.home_lon = home_lon
        
    # set_home(master, home_lat, home_lon, home_alt)

    #TODO: Remove
    try:

        #Safety
        #while True:
        #    print("spinning in while loop")
        #    time.sleep(2)

        # --- MISSION START ---
        #prep_flight(master,state)
        #demo_mission(master,state)
        print("starting BCI mission")
        bci_mission(master,state)

        print("\nMission Complete. Entering Monitor Mode. Press Ctrl+C to Land.")
        
        # --- ACTIVE SAFETY LOOP ---
        while True:
            dist = haversine_distance(home_lat, home_lon, state.lat, state.lon)
        
            # Telemetry Output
            print(f"Dist: {dist:.1f}m")
            print_telemetry(state)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[USER INTERRUPT].")
        land(master, state,rtl=False)
        print("Forcing Land...")

    except Exception as e:
        print(f"\n\n[PIPELINE ERROR] {e}")
        master.set_mode('LAND')

if __name__ == "__main__":
    main()