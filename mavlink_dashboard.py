#!/usr/bin/env python3

import math
import threading
import time
from typing import Any

from flask import Flask, jsonify, render_template
from pymavlink import mavutil


SERIAL_DEVICE = "/dev/serial0"
SERIAL_BAUD = 57600
HEARTBEAT_TIMEOUT_SECONDS = 5.0

app = Flask(__name__)

state_lock = threading.Lock()

vehicle_state: dict[str, Any] = {
    "connected": False,
    "last_heartbeat_age_s": None,
    "system_id": None,
    "component_id": None,
    "vehicle_type": "Unknown",
    "autopilot": "Unknown",
    "flight_mode": "Unknown",
    "armed": False,
    "system_status": "Unknown",

    "roll_deg": None,
    "pitch_deg": None,
    "yaw_deg": None,

    "heading_deg": None,
    "groundspeed_mps": None,
    "airspeed_mps": None,
    "climb_mps": None,
    "altitude_m": None,
    "relative_altitude_m": None,

    "gps_fix": "No GPS",
    "gps_fix_type": 0,
    "satellites": 0,
    "latitude": None,
    "longitude": None,
    "horizontal_accuracy_m": None,
    "vertical_accuracy_m": None,

    "battery_voltage_v": None,
    "battery_current_a": None,
    "battery_remaining_pct": None,

    "autopilot_load_pct": None,
    "communication_errors": 0,

    "message_count": 0,
    "last_message_type": None,
    "last_error": None,
}


GPS_FIX_NAMES = {
    0: "No GPS",
    1: "No Fix",
    2: "2D Fix",
    3: "3D Fix",
    4: "DGPS",
    5: "RTK Float",
    6: "RTK Fixed",
    7: "Static Fixed",
    8: "PPP",
}


SYSTEM_STATUS_NAMES = {
    mavutil.mavlink.MAV_STATE_UNINIT: "Uninitialized",
    mavutil.mavlink.MAV_STATE_BOOT: "Booting",
    mavutil.mavlink.MAV_STATE_CALIBRATING: "Calibrating",
    mavutil.mavlink.MAV_STATE_STANDBY: "Standby",
    mavutil.mavlink.MAV_STATE_ACTIVE: "Active",
    mavutil.mavlink.MAV_STATE_CRITICAL: "Critical",
    mavutil.mavlink.MAV_STATE_EMERGENCY: "Emergency",
    mavutil.mavlink.MAV_STATE_POWEROFF: "Power Off",
    mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION: "Flight Termination",
}


def valid_mavlink_value(value: int, invalid_value: int) -> bool:
    return value != invalid_value


def request_message_rates(master: mavutil.mavfile) -> None:
    rates_hz = {
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 1,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 5,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 2,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 2,
    }

    for message_id, rate_hz in rates_hz.items():
        interval_us = int(1_000_000 / rate_hz)

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )


def process_message(master: mavutil.mavfile, msg: Any) -> None:
    message_type = msg.get_type()

    with state_lock:
        vehicle_state["message_count"] += 1
        vehicle_state["last_message_type"] = message_type
        vehicle_state["last_error"] = None

        if message_type == "HEARTBEAT":
            master.last_heartbeat_time = time.monotonic()

            vehicle_state["connected"] = True
            vehicle_state["system_id"] = master.target_system
            vehicle_state["component_id"] = master.target_component
            vehicle_state["flight_mode"] = mavutil.mode_string_v10(msg)
            vehicle_state["armed"] = bool(
                msg.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            vehicle_state["system_status"] = SYSTEM_STATUS_NAMES.get(
                msg.system_status,
                f"Status {msg.system_status}",
            )

            try:
                vehicle_state["vehicle_type"] = (
                    mavutil.mavlink.enums["MAV_TYPE"][msg.type].name
                    .replace("MAV_TYPE_", "")
                    .replace("_", " ")
                    .title()
                )
            except (KeyError, AttributeError):
                vehicle_state["vehicle_type"] = f"Type {msg.type}"

            try:
                vehicle_state["autopilot"] = (
                    mavutil.mavlink.enums["MAV_AUTOPILOT"][msg.autopilot].name
                    .replace("MAV_AUTOPILOT_", "")
                    .replace("_", " ")
                    .title()
                )
            except (KeyError, AttributeError):
                vehicle_state["autopilot"] = f"Autopilot {msg.autopilot}"

        elif message_type == "ATTITUDE":
            vehicle_state["roll_deg"] = math.degrees(msg.roll)
            vehicle_state["pitch_deg"] = math.degrees(msg.pitch)
            vehicle_state["yaw_deg"] = (
                math.degrees(msg.yaw) + 360.0
            ) % 360.0

        elif message_type == "VFR_HUD":
            vehicle_state["heading_deg"] = msg.heading
            vehicle_state["groundspeed_mps"] = msg.groundspeed
            vehicle_state["airspeed_mps"] = msg.airspeed
            vehicle_state["climb_mps"] = msg.climb
            vehicle_state["altitude_m"] = msg.alt

        elif message_type == "GLOBAL_POSITION_INT":
            if msg.lat != 0 or msg.lon != 0:
                vehicle_state["latitude"] = msg.lat / 10_000_000.0
                vehicle_state["longitude"] = msg.lon / 10_000_000.0

            vehicle_state["relative_altitude_m"] = (
                msg.relative_alt / 1000.0
            )

            if msg.hdg != 65535:
                vehicle_state["heading_deg"] = msg.hdg / 100.0

        elif message_type == "GPS_RAW_INT":
            fix_type = int(msg.fix_type)

            vehicle_state["gps_fix_type"] = fix_type
            vehicle_state["gps_fix"] = GPS_FIX_NAMES.get(
                fix_type,
                f"Fix Type {fix_type}",
            )
            vehicle_state["satellites"] = int(msg.satellites_visible)

            if fix_type >= 2 and (msg.lat != 0 or msg.lon != 0):
                vehicle_state["latitude"] = msg.lat / 10_000_000.0
                vehicle_state["longitude"] = msg.lon / 10_000_000.0

            h_acc = getattr(msg, "h_acc", 0)
            v_acc = getattr(msg, "v_acc", 0)

            vehicle_state["horizontal_accuracy_m"] = (
                h_acc / 1000.0 if h_acc > 0 else None
            )
            vehicle_state["vertical_accuracy_m"] = (
                v_acc / 1000.0 if v_acc > 0 else None
            )

        elif message_type == "SYS_STATUS":
            vehicle_state["autopilot_load_pct"] = msg.load / 10.0
            vehicle_state["communication_errors"] = msg.errors_comm

            vehicle_state["battery_voltage_v"] = (
                msg.voltage_battery / 1000.0
                if valid_mavlink_value(msg.voltage_battery, 65535)
                and msg.voltage_battery >= 1000
                else None
            )

            vehicle_state["battery_current_a"] = (
                msg.current_battery / 100.0
                if valid_mavlink_value(msg.current_battery, -1)
                else None
            )

            vehicle_state["battery_remaining_pct"] = (
                msg.battery_remaining
                if msg.battery_remaining >= 0
                else None
            )


def mavlink_worker() -> None:
    while True:
        master = None

        try:
            with state_lock:
                vehicle_state["connected"] = False
                vehicle_state["last_error"] = None

            print(
                f"Opening MAVLink connection on "
                f"{SERIAL_DEVICE} at {SERIAL_BAUD} baud..."
            )

            master = mavutil.mavlink_connection(
                SERIAL_DEVICE,
                baud=SERIAL_BAUD,
                autoreconnect=True,
            )

            heartbeat = master.wait_heartbeat(timeout=15)

            if heartbeat is None:
                raise TimeoutError("No heartbeat received within 15 seconds")

            master.last_heartbeat_time = time.monotonic()

            print(
                f"MAVLink connected: system {master.target_system}, "
                f"component {master.target_component}"
            )

            request_message_rates(master)
            process_message(master, heartbeat)

            while True:
                msg = master.recv_match(blocking=True, timeout=1)

                now = time.monotonic()
                heartbeat_age = now - master.last_heartbeat_time

                with state_lock:
                    vehicle_state["last_heartbeat_age_s"] = heartbeat_age
                    vehicle_state["connected"] = (
                        heartbeat_age <= HEARTBEAT_TIMEOUT_SECONDS
                    )

                if msg is not None:
                    process_message(master, msg)

        except Exception as exc:
            print(f"MAVLink error: {exc}")

            with state_lock:
                vehicle_state["connected"] = False
                vehicle_state["last_error"] = str(exc)

            time.sleep(3)

        finally:
            if master is not None:
                master.close()


@app.route("/")
def index():
    return render_template("mavlink_dashboard.html")


@app.route("/api/mavlink/status")
def api_status():
    with state_lock:
        return jsonify(dict(vehicle_state))


if __name__ == "__main__":
    worker = threading.Thread(
        target=mavlink_worker,
        daemon=True,
        name="mavlink-worker",
    )
    worker.start()

    app.run(
        host="0.0.0.0",
        port=5001,
        debug=False,
        threaded=True,
    )
