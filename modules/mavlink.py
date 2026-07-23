#!/usr/bin/env python3

import math
import threading
import time
from copy import deepcopy
from typing import Any

from pymavlink import mavutil


DEFAULT_SERIAL_DEVICE = "/dev/serial0"
DEFAULT_SERIAL_BAUD = 57600
DEFAULT_HEARTBEAT_TIMEOUT_SECONDS = 5.0
DEFAULT_RECONNECT_DELAY_SECONDS = 3.0


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


def initial_vehicle_state() -> dict[str, Any]:
    return {
        "connected": False,
        "service_running": False,
        "serial_device": None,
        "serial_baud": None,
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


def valid_mavlink_value(value: int, invalid_value: int) -> bool:
    return value != invalid_value


class MavlinkModule:
    def __init__(
        self,
        serial_device: str = DEFAULT_SERIAL_DEVICE,
        serial_baud: int = DEFAULT_SERIAL_BAUD,
        heartbeat_timeout_seconds: float = (
            DEFAULT_HEARTBEAT_TIMEOUT_SECONDS
        ),
        reconnect_delay_seconds: float = (
            DEFAULT_RECONNECT_DELAY_SECONDS
        ),
    ) -> None:
        self.serial_device = serial_device
        self.serial_baud = serial_baud
        self.heartbeat_timeout_seconds = heartbeat_timeout_seconds
        self.reconnect_delay_seconds = reconnect_delay_seconds

        self._state_lock = threading.Lock()
        self._state = initial_vehicle_state()
        self._state["serial_device"] = serial_device
        self._state["serial_baud"] = serial_baud

        self._worker_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._master: mavutil.mavfile | None = None

    def start(self) -> None:
        if self._worker_thread is not None:
            if self._worker_thread.is_alive():
                return

        self._stop_event.clear()

        self._worker_thread = threading.Thread(
            target=self._worker,
            daemon=True,
            name="mavlink-worker",
        )
        self._worker_thread.start()

    def stop(self) -> None:
        self._stop_event.set()

        master = self._master
        if master is not None:
            try:
                master.close()
            except Exception:
                pass

        worker = self._worker_thread
        if worker is not None and worker.is_alive():
            worker.join(timeout=3.0)

        with self._state_lock:
            self._state["connected"] = False
            self._state["service_running"] = False

    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            return deepcopy(self._state)

    def _request_message_rates(
        self,
        master: mavutil.mavfile,
    ) -> None:
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

    def _process_message(
        self,
        master: mavutil.mavfile,
        msg: Any,
    ) -> None:
        message_type = msg.get_type()

        with self._state_lock:
            self._state["message_count"] += 1
            self._state["last_message_type"] = message_type
            self._state["last_error"] = None

            if message_type == "HEARTBEAT":
                master.last_heartbeat_time = time.monotonic()

                self._state["connected"] = True
                self._state["system_id"] = master.target_system
                self._state["component_id"] = master.target_component
                self._state["flight_mode"] = mavutil.mode_string_v10(msg)
                self._state["armed"] = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self._state["system_status"] = (
                    SYSTEM_STATUS_NAMES.get(
                        msg.system_status,
                        f"Status {msg.system_status}",
                    )
                )

                try:
                    self._state["vehicle_type"] = (
                        mavutil.mavlink.enums["MAV_TYPE"][msg.type]
                        .name
                        .replace("MAV_TYPE_", "")
                        .replace("_", " ")
                        .title()
                    )
                except (KeyError, AttributeError):
                    self._state["vehicle_type"] = (
                        f"Type {msg.type}"
                    )

                try:
                    self._state["autopilot"] = (
                        mavutil.mavlink.enums[
                            "MAV_AUTOPILOT"
                        ][msg.autopilot]
                        .name
                        .replace("MAV_AUTOPILOT_", "")
                        .replace("_", " ")
                        .title()
                    )
                except (KeyError, AttributeError):
                    self._state["autopilot"] = (
                        f"Autopilot {msg.autopilot}"
                    )

            elif message_type == "ATTITUDE":
                self._state["roll_deg"] = math.degrees(msg.roll)
                self._state["pitch_deg"] = math.degrees(msg.pitch)
                self._state["yaw_deg"] = (
                    math.degrees(msg.yaw) + 360.0
                ) % 360.0

            elif message_type == "VFR_HUD":
                self._state["heading_deg"] = msg.heading
                self._state["groundspeed_mps"] = msg.groundspeed
                self._state["airspeed_mps"] = msg.airspeed
                self._state["climb_mps"] = msg.climb
                self._state["altitude_m"] = msg.alt

            elif message_type == "GLOBAL_POSITION_INT":
                if msg.lat != 0 or msg.lon != 0:
                    self._state["latitude"] = (
                        msg.lat / 10_000_000.0
                    )
                    self._state["longitude"] = (
                        msg.lon / 10_000_000.0
                    )

                self._state["relative_altitude_m"] = (
                    msg.relative_alt / 1000.0
                )

                if msg.hdg != 65535:
                    self._state["heading_deg"] = msg.hdg / 100.0

            elif message_type == "GPS_RAW_INT":
                fix_type = int(msg.fix_type)

                self._state["gps_fix_type"] = fix_type
                self._state["gps_fix"] = GPS_FIX_NAMES.get(
                    fix_type,
                    f"Fix Type {fix_type}",
                )
                self._state["satellites"] = int(
                    msg.satellites_visible
                )

                if (
                    fix_type >= 2
                    and (msg.lat != 0 or msg.lon != 0)
                ):
                    self._state["latitude"] = (
                        msg.lat / 10_000_000.0
                    )
                    self._state["longitude"] = (
                        msg.lon / 10_000_000.0
                    )

                h_acc = getattr(msg, "h_acc", 0)
                v_acc = getattr(msg, "v_acc", 0)

                self._state["horizontal_accuracy_m"] = (
                    h_acc / 1000.0 if h_acc > 0 else None
                )
                self._state["vertical_accuracy_m"] = (
                    v_acc / 1000.0 if v_acc > 0 else None
                )

            elif message_type == "SYS_STATUS":
                self._state["autopilot_load_pct"] = (
                    msg.load / 10.0
                )
                self._state["communication_errors"] = (
                    msg.errors_comm
                )

                self._state["battery_voltage_v"] = (
                    msg.voltage_battery / 1000.0
                    if valid_mavlink_value(
                        msg.voltage_battery,
                        65535,
                    )
                    and msg.voltage_battery >= 1000
                    else None
                )

                self._state["battery_current_a"] = (
                    msg.current_battery / 100.0
                    if valid_mavlink_value(
                        msg.current_battery,
                        -1,
                    )
                    else None
                )

                self._state["battery_remaining_pct"] = (
                    msg.battery_remaining
                    if msg.battery_remaining >= 0
                    else None
                )

    def _worker(self) -> None:
        with self._state_lock:
            self._state["service_running"] = True

        while not self._stop_event.is_set():
            master = None

            try:
                with self._state_lock:
                    self._state["connected"] = False
                    self._state["last_error"] = None

                print(
                    "Opening MAVLink connection on "
                    f"{self.serial_device} at "
                    f"{self.serial_baud} baud..."
                )

                master = mavutil.mavlink_connection(
                    self.serial_device,
                    baud=self.serial_baud,
                    autoreconnect=True,
                )
                self._master = master

                heartbeat = master.wait_heartbeat(timeout=15)

                if heartbeat is None:
                    raise TimeoutError(
                        "No heartbeat received within 15 seconds"
                    )

                master.last_heartbeat_time = time.monotonic()

                print(
                    "MAVLink connected: "
                    f"system {master.target_system}, "
                    f"component {master.target_component}"
                )

                self._request_message_rates(master)
                self._process_message(master, heartbeat)

                while not self._stop_event.is_set():
                    msg = master.recv_match(
                        blocking=True,
                        timeout=1,
                    )

                    now = time.monotonic()
                    heartbeat_age = (
                        now - master.last_heartbeat_time
                    )

                    with self._state_lock:
                        self._state[
                            "last_heartbeat_age_s"
                        ] = heartbeat_age
                        self._state["connected"] = (
                            heartbeat_age
                            <= self.heartbeat_timeout_seconds
                        )

                    if msg is not None:
                        self._process_message(master, msg)

            except Exception as exc:
                if not self._stop_event.is_set():
                    print(f"MAVLink error: {exc}")

                    with self._state_lock:
                        self._state["connected"] = False
                        self._state["last_error"] = str(exc)

                    self._stop_event.wait(
                        self.reconnect_delay_seconds
                    )

            finally:
                if master is not None:
                    try:
                        master.close()
                    except Exception:
                        pass

                self._master = None

        with self._state_lock:
            self._state["connected"] = False
            self._state["service_running"] = False


mavlink = MavlinkModule()
