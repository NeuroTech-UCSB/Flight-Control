from pymavlink import mavutil

m = mavutil.mavlink_connection("udpout:192.168.2.1:14550", source_system=255)

# Register endpoint + identify as GCS
m.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0
)

m.wait_heartbeat(timeout=10)
print("Connected to sys", m.target_system, "comp", m.target_component)

# Arm command (bench test only, props off)
m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
print("ARM sent")
