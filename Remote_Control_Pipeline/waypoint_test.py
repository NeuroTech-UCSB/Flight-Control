import threading
from pymavlink import mavutil
import time
import math

# --- 1. THE POST OFFICE (Global State) ---
# This dictionary will hold the latest info so functions don't have to "read" the port
drone_state = {
    "last_heartbeat": time.time(),
    "lat": 0.0,
    "lon": 0.0,
    "alt": 0.0,
    "mode": "UNKNOWN",
    "armed": False,
    "is_running": True
}

# --- 2. THE TELEMETRY THREAD (The only one allowed to read) ---
def telemetry_reader(master):
    global drone_state
    print("[Thread] Post Office is Open. Reading all telemetry...")
    while drone_state["is_running"]:
        msg = master.recv_match(blocking=False)
        if msg:
            m_type = msg.get_type()
            if m_type == 'HEARTBEAT':
                drone_state["last_heartbeat"] = time.time()
                drone_state["mode"] = master.flightmode
                drone_state["armed"] = master.motors_armed()
            elif m_type == 'GLOBAL_POSITION_INT':
                drone_state["lat"] = msg.lat / 1e7
                drone_state["lon"] = msg.lon / 1e7
                drone_state["alt"] = msg.relative_alt # mm
        time.sleep(0.01) # 100Hz loop

# --- 3. REFACTORED FLIGHT FUNCTIONS ---
def wait_for_arm(target_state=True, timeout=10):
    """Checks the Post Office for armed status instead of reading the port."""
    start = time.time()
    while drone_state["armed"] != target_state:
        if time.time() - start > timeout:
            raise Exception("Arming timeout - check Pre-arm messages in SITL")
        time.sleep(0.2)
    print("Armed confirmation received from Post Office!")

def take_off_safe(master, altitude):
    print(f"🚀 Initiating Takeoff to {altitude}m...")
    
    # Mode Change
    master.set_mode('GUIDED')
    
    # Arm
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    wait_for_arm(True)

    # Takeoff
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
    )

    # Watch altitude in the Post Office
    while (drone_state["alt"] / 1000.0) < (altitude - 1.0):
        print(f"Climbing... {drone_state['alt']/1000.0:.1f}m", end='\r')
        time.sleep(0.2)
    print("\nTarget Altitude Reached.")

# --- 4. MAIN EXECUTION ---
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

# Start the Telemetry Thread
t = threading.Thread(target=telemetry_reader, args=(master,), daemon=True)
t.start()

try:
    take_off_safe(master, 15)
    
    while True:
        # Check Watchdog
        if time.time() - drone_state["last_heartbeat"] > 3.0:
            raise Exception(f"Heartbeat Lost! {time.time() - drone_state['last_heartbeat']:.2f}s")
        
        print(f"Telemetry: Alt={drone_state['alt']/1000.0:.1f}m | Mode={drone_state['mode']}", end='\r')
        time.sleep(0.1)

except Exception as e:
    print(f"\n🛑 Pipeline Error: {e}")
    drone_state["is_running"] = False
    master.set_mode('LAND')