from pymavlink import mavutil
import time

print("Connecting to drone...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# --- NEW: SEND COMMANDS ---
# 1. Change mode to GUIDED
print("Switching to GUIDED mode...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4) # 4 is the code for GUIDED mode in ArduCopter

# 2. Arm the motors
print("Arming motors...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)

# 3. Takeoff to 5 meters
print("Taking off!")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 5)

# --- CONTINUOUS TELEMETRY ---
print("Streaming altitude (Press Ctrl+C to stop)...")
try:
    while True:
        msg_alt = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        altitude = msg_alt.relative_alt / 1000.0
        print(f"Altitude: {altitude:.2f}m")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Stopping telemetry.")