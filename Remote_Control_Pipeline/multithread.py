from pymavlink import mavutil
import math
import time
import threading
import os

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
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.flightmode = "UNKNOWN"
        self.is_armed = False
        self.ekf_ready = False

        # Navigation / Home Data (Initialized to None until EKF is ready)
        self.home_lat = None
        self.home_lon = None
        self.home_alt_amsl = None
        
        # Command Acknowledgments
        self.mission_ack_received = False
        self.command_ack = None # Will store a tuple: (command_id, result)
        
        # Sensor Data
        self.bci_command = "STANDBY" # Variable stub
        
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

                # Update Extended Kalmen Filter (EKF) status
                elif m_type == 'SYS_STATUS':
                    # 0x400000 is the bitmask for EKF/Positioning active
                    if msg.onboard_control_sensors_present & 0x400000:
                        state.ekf_ready = True
                
                # Update mission and command acks
                elif m_type == 'MISSION_ACK':
                    state.mission_ack_received = True
                    
                elif m_type == 'COMMAND_ACK':
                    state.command_ack = (msg.command, msg.result) # (command_id, result)
        
        # 3. Active Checks
        with state.lock:
            hb_gap = time.time() - state.last_heartbeat
            ekf_ready = state.ekf_ready
            curr_lat, curr_lon = state.lat, state.lon
            h_lat, h_lon = state.home_lat, state.home_lon
            breached = state.geofence_breached
            range = state.geofence_range
            
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

        time.sleep(0.01) # Yield
 

# ==========================================
# 4. Thread 2: BCI Sensor (Stub)
# ==========================================
def bci_thread(state):
    """Simulates reading incoming brainwave/eye-tracking data asynchronously."""
    while True:
        # Simulate processing time for an EEG/Eye-Tracker frame (e.g., 200ms)
        time.sleep(0.2) 
        
        # Placeholder for actual BCI SDK logic
        simulated_reading = "STANDBY" 
        
        # Safely update the shared state
        with state.lock:
            state.bci_command = simulated_reading

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

def prep_flight(master, state):
    """Safely transitions to GUIDED mode and ARMs the motors."""
    print("Pre-Flight Checks Initiated...")
    
    # 1. Check/Set Mode
    with state.lock:
        current_mode = state.flightmode
        
    if current_mode != 'GUIDED':
        print("Switching to GUIDED mode...")
        master.set_mode('GUIDED')
        while True:
            with state.lock:
                if state.flightmode == 'GUIDED': break
            time.sleep(0.1)

    # 2. Check/Set Arming
    with state.lock:
        armed = state.is_armed
        
    if not armed:
        print("Arming Motors...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )
        while True: # Blocking loop while motors aren't armed
            with state.lock:
                if state.is_armed: break
            time.sleep(0.1)
    
    time.sleep(1)
    print("Pre-Flight Complete.")

def take_off(master, state, target_alt):
    """Initiates flight prep and takeoff to desired altitude"""
    prep_flight(master, state)
    print(f"Climbing to {target_alt} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_alt
    )

    while True:
        with state.lock:
            curr_alt = state.alt
            
        print(f"Takeoff Alt: {curr_alt:.1f}m", end='\r')
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

# ==========================================
# 6. Thread 3: Mission (Main Loop)
# ==========================================
def main():
    # Instantiate the global state tracker FlightState object
    state = FlightState()
    # Instantiates the drone mavutil object and connects to physical drone
    print("Connecting to BCI-Drone Pipeline...")
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat() # Waits till heartbeat connects before proceeding
    print("Heartbeat Connected.")

    # Start the Background Threads
    # 'daemon=True' ensures threads will automatically terminate when  main script ends
    t_telem = threading.Thread(target=telemetry_thread, args=(master, state,), daemon=True)
    t_bci = threading.Thread(target=bci_thread, args=(state,), daemon=True)
    
    t_telem.start()
    t_bci.start()
    print("Background Threads Running: Telemetry [ON], BCI [ON]")

    # Wait for initial GPS lock via the background thread
    print("Waiting for EKF/GPS Readiness...")
    while True:
        with state.lock:
            if state.ekf_ready: break
        time.sleep(1)
    print("System Ready for Flight.")

    with state.lock:
        home_lat, home_lon, home_alt = state.lat, state.lon, state.alt
        state.home_lat = home_lat
        state.home_lon = home_lon
        
    # set_home(master, home_lat, home_lon, home_alt)

    try:
        # --- MISSION START ---
        take_off(master, state, 10.0)
        nav_waypoint(master, state, home_lat + 0.0001, home_lon + 0.0001, 10.0)
        land(master, state, rtl=True)

        print("\nMission Complete. Entering Monitor Mode. Press Ctrl+C to Land.")
        
        # --- ACTIVE SAFETY LOOP ---
        while True:
            dist = haversine_distance(home_lat, home_lon, state.lat, state.lon)
        
            # Telemetry Output
            print(f"Dist: {dist:.1f}m | Alt: {state.alt:.1f}m | Mode: {state.flightmode} | BCI: {state.bci_command}", end='\r')
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[USER INTERRUPT].")
        land(master, rtl=False)
        print("Forcing Land...")

    except Exception as e:
        print(f"\n\n[PIPELINE ERROR] {e}")
        master.set_mode('LAND')

if __name__ == "__main__":
    main()