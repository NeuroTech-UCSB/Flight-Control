from pymavlink import mavutil
import time


# Adjust if your ESP32/DroneBridge link is exposing MAVLink elsewhere.
CONNECTION_STRING = "udpin:0.0.0.0:14550"
TAKEOFF_ALT_M = 3
HOVER_SECONDS = 3


last_heartbeat = time.time()
last_alt_mm = 0
last_lat = 0.0
last_lon = 0.0
current_mode = "UNKNOWN"
active_master = None
last_status_text = ""
armed_state = False
gps_fix_type = 0
gps_satellites = 0
gps_eph = None
battery_mv = None
battery_remaining = None
ekf_flags = None


def sync(master, msg_types=None, timeout=0.2):
    """Single MAVLink reader that also keeps state variables fresh."""
    global last_heartbeat, last_alt_mm, last_lat, last_lon, current_mode
    global last_status_text, armed_state, gps_fix_type, gps_satellites
    global gps_eph, battery_mv, battery_remaining, ekf_flags

    if msg_types is None:
        request = None
    elif isinstance(msg_types, str):
        request = [msg_types, "HEARTBEAT"]
    else:
        request = list(set(msg_types + ["HEARTBEAT"]))

    msg = master.recv_match(type=request, blocking=True, timeout=timeout)
    if not msg:
        return None

    last_heartbeat = time.time()
    msg_type = msg.get_type()
    if msg_type == "HEARTBEAT":
        current_mode = master.flightmode
        armed_state = master.motors_armed()
    elif msg_type == "GLOBAL_POSITION_INT":
        last_alt_mm = msg.relative_alt
        last_lat = msg.lat / 1e7
        last_lon = msg.lon / 1e7
    elif msg_type == "GPS_RAW_INT":
        gps_fix_type = getattr(msg, "fix_type", 0)
        gps_satellites = getattr(msg, "satellites_visible", 0)
        gps_eph = getattr(msg, "eph", None)
    elif msg_type == "SYS_STATUS":
        battery_mv = getattr(msg, "voltage_battery", None)
        battery_remaining = getattr(msg, "battery_remaining", None)
    elif msg_type == "EKF_STATUS_REPORT":
        ekf_flags = getattr(msg, "flags", None)
    elif msg_type == "STATUSTEXT":
        last_status_text = getattr(msg, "text", "") or ""

    return msg


def status_summary():
    hb_age = time.time() - last_heartbeat
    alt_m = last_alt_mm / 1000.0
    gps_bits = f"fix={gps_fix_type} sats={gps_satellites}"
    if gps_eph not in (None, 65535):
        gps_bits += f" hdop={gps_eph / 100.0:.2f}"

    if battery_mv in (None, 65535):
        batt_bits = "n/a"
    else:
        batt_v = battery_mv / 1000.0
        if battery_remaining is None or battery_remaining == -1:
            batt_bits = f"{batt_v:.2f}V"
        else:
            batt_bits = f"{batt_v:.2f}V {battery_remaining}%"

    pos_bits = f"{last_lat:.6f},{last_lon:.6f}"
    ekf_bits = f"0x{ekf_flags:X}" if ekf_flags is not None else "n/a"
    text_bits = f" | {last_status_text}" if last_status_text else ""

    return (
        f"Mode={current_mode} Armed={armed_state} Alt={alt_m:.2f}m "
        f"GPS[{gps_bits}] Pos={pos_bits} Batt={batt_bits} EKF={ekf_bits} "
        f"HB={hb_age:.1f}s{text_bits}"
    )


def show_status_monitor(master, duration=5):
    """Print startup status so pre-arm failures are easier to diagnose."""
    print(f"Telemetry status monitor ({duration}s)...")
    deadline = time.time() + duration
    watched = [
        "GPS_RAW_INT",
        "GLOBAL_POSITION_INT",
        "SYS_STATUS",
        "EKF_STATUS_REPORT",
        "STATUSTEXT",
    ]
    last_print = 0.0
    while time.time() < deadline:
        sync(master, msg_types=watched, timeout=0.3)
        now = time.time()
        if now - last_print >= 0.5:
            print(status_summary(), end="\r")
            last_print = now
    print()


def msg_src(msg):
    """Return (system_id, component_id) if available."""
    try:
        return msg.get_srcSystem(), msg.get_srcComponent()
    except Exception:
        return None, None


def select_autopilot_target(master, timeout=10):
    """
    DroneBridge/routers may emit their own heartbeat. Lock commands to the
    actual autopilot heartbeat (typically component 1) before sending commands.
    """
    print("Searching for autopilot heartbeat...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = sync(master, msg_types="HEARTBEAT", timeout=0.5)
        if not msg or msg.get_type() != "HEARTBEAT":
            continue

        src_sys, src_comp = msg_src(msg)
        autopilot = getattr(msg, "autopilot", None)
        mav_type = getattr(msg, "type", None)

        is_autopilot_hb = (
            src_comp == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            or (
                autopilot is not None
                and autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                and mav_type != mavutil.mavlink.MAV_TYPE_GCS
            )
        )

        if is_autopilot_hb:
            master.target_system = src_sys
            master.target_component = src_comp
            print(
                f"Using autopilot target system {master.target_system} component {master.target_component}"
            )
            return

    raise TimeoutError("Could not find autopilot heartbeat (only bridge/GCS heartbeats seen)")


def wait_for_mode(master, target_mode, timeout=10):
    start = time.time()
    while current_mode != target_mode:
        sync(master)
        if time.time() - start > timeout:
            raise TimeoutError(f"Timed out waiting for mode {target_mode}, got {current_mode}")


def arm(master, timeout=10):
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    start = time.time()
    while not master.motors_armed():
        msg = sync(master, msg_types=["COMMAND_ACK", "STATUSTEXT"])
        if msg and msg.get_type() == "COMMAND_ACK":
            src_sys, src_comp = msg_src(msg)

            if msg.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                continue

            # Ignore ACKs from bridge/router/non-autopilot components.
            if (
                src_sys is not None
                and src_comp is not None
                and (src_sys != master.target_system or src_comp != master.target_component)
            ):
                print(
                    f"Ignoring ARM ACK from system {src_sys} component {src_comp} (result={msg.result})"
                )
                continue

            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                detail = f"Arming rejected, ACK result={msg.result}"
                if last_status_text:
                    detail += f" | STATUSTEXT: {last_status_text}"
                detail += f" | {status_summary()}"
                raise RuntimeError(detail)
        if time.time() - start > timeout:
            detail = "Timed out waiting for arm"
            if last_status_text:
                detail += f" | Last STATUSTEXT: {last_status_text}"
            detail += f" | {status_summary()}"
            raise TimeoutError(detail)

    print("Armed.")


def takeoff(master, altitude_m):
    print(f"Takeoff to {altitude_m}m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude_m,
    )

    start = time.time()
    while True:
        sync(master, msg_types="GLOBAL_POSITION_INT")
        alt_m = last_alt_mm / 1000.0
        print(f"Altitude: {alt_m:.2f}m", end="\r")
        if alt_m >= max(0.5, altitude_m - 0.5):
            print(f"\nReached takeoff altitude ({alt_m:.2f}m).")
            return
        if time.time() - start > 30:
            raise TimeoutError("Timed out waiting for takeoff altitude")


def land(master):
    print("Switching to LAND mode...")
    master.set_mode("LAND")


def disarm(master):
    print("Disarming motors (emergency fallback)...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def emergency_land_or_disarm(master, reason):
    """Best-effort exit failsafe: LAND first, then disarm if LAND does not engage."""
    if master is None:
        print(f"Failsafe skipped: no MAVLink connection ({reason})")
        return

    print(f"\nFailsafe triggered: {reason}")

    try:
        # If we never armed, there is nothing meaningful to land/disarm.
        if not master.motors_armed():
            print("Motors already disarmed.")
            return
    except Exception:
        # Keep trying; some links may be stale while we still have a port.
        pass

    try:
        start_alt_mm = last_alt_mm
        land(master)

        # Give LAND a short window to take effect (mode change or visible descent).
        deadline = time.time() + 5
        while time.time() < deadline:
            sync(master, msg_types=["GLOBAL_POSITION_INT", "COMMAND_ACK"], timeout=0.3)
            alt_drop_mm = max(0, start_alt_mm - last_alt_mm)
            if current_mode == "LAND" or alt_drop_mm > 300:
                print(
                    f"LAND engaged (mode={current_mode}, alt={last_alt_mm/1000.0:.2f}m)."
                )
                return
            time.sleep(0.1)

        print("LAND did not engage in time.")
    except Exception as exc:
        print(f"LAND attempt failed: {exc}")

    try:
        disarm(master)
        # Briefly wait for arm state to clear.
        deadline = time.time() + 3
        while time.time() < deadline:
            sync(master, msg_types="HEARTBEAT", timeout=0.2)
            if not master.motors_armed():
                print("Motors disarmed.")
                return
            time.sleep(0.1)
        print("Disarm command sent; could not confirm disarm state.")
    except Exception as exc:
        print(f"Disarm fallback failed: {exc}")


def main():
    global current_mode, active_master

    print(f"Connecting to drone via {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    active_master = master
    master.wait_heartbeat()
    print(
        "Heartbeat from system %u component %u"
        % (master.target_system, master.target_component)
    )
    select_autopilot_target(master)
    show_status_monitor(master, duration=5)

    # Prime state values.
    current_mode = master.flightmode or "UNKNOWN"
    sync(master, msg_types=["GLOBAL_POSITION_INT", "GPS_RAW_INT", "SYS_STATUS"])

    print("Switching to GUIDED mode...")
    master.set_mode("GUIDED")
    wait_for_mode(master, "GUIDED")
    print("GUIDED mode confirmed.")

    arm(master)
    takeoff(master, TAKEOFF_ALT_M)

    print(f"Holding for {HOVER_SECONDS} seconds...")
    hold_end = time.time() + HOVER_SECONDS
    while time.time() < hold_end:
        sync(master, msg_types="GLOBAL_POSITION_INT", timeout=0.2)
        if time.time() - last_heartbeat > 5:
            raise RuntimeError("Heartbeat timeout during hover")
        print(status_summary(), end="\r")
        time.sleep(0.1)
    print()

    land(master)

    print("Monitoring descent (Ctrl+C to stop)...")
    while True:
        sync(master, msg_types="GLOBAL_POSITION_INT", timeout=0.2)
        if time.time() - last_heartbeat > 5:
            raise RuntimeError("Heartbeat timeout during landing")
        print(status_summary(), end="\r")
        time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        emergency_land_or_disarm(active_master, "user interrupt (Ctrl+C)")
        print("Interrupted by user.")
    except Exception as exc:
        print(f"\nTakeoff test failed: {exc}")
        emergency_land_or_disarm(active_master, f"runtime error: {exc}")
