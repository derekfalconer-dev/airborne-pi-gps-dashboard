#!/usr/bin/env python3

from __future__ import annotations

import glob
import os
import threading
import time

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import serial
from pyubx2 import (
    NMEA_PROTOCOL,
    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXReader,
)

from modules.crtn import (
    load_crtn_stations,
    nearest_crtn_stations,
)
from modules.ntrip import NtripClient


PROJECT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_CRTN_CSV_PATH = (
    PROJECT_DIR / "data" / "crtn_mountpoint_list.csv"
)

DEFAULT_BAUD = 115200
DEFAULT_RECONNECT_DELAY_SECONDS = 2.0
DEFAULT_MESSAGE_TIMEOUT_SECONDS = 5.0


FIX_NAMES = {
    0: "No fix",
    1: "Standalone GNSS",
    2: "Differential GNSS",
    3: "PPS fix",
    4: "RTK fixed",
    5: "RTK float",
    6: "Dead reckoning",
}


def initial_gnss_state() -> dict[str, Any]:
    return {
        "service_running": False,
        "receiver_connected": False,
        "serial_port": None,
        "serial_baud": None,

        "last_message_utc": None,
        "message_age_seconds": None,

        "fix_quality": 0,
        "fix_name": "No fix",
        "latitude": None,
        "longitude": None,
        "altitude_msl_m": None,
        "satellites": 0,
        "hdop": None,
        "speed_mps": None,
        "speed_mph": None,
        "course_deg": None,
        "correction_age_s": None,

        "bytes_from_receiver": 0,

        "selected_station_code": None,
        "selected_station_name": None,
        "selected_mountpoint": None,
        "selected_station_distance_km": None,
        "selected_ntrip_host": None,
        "selected_ntrip_port": None,
        "station_candidates": [],

        "horizontal_accuracy_m": None,
        "vertical_accuracy_m": None,
        "speed_accuracy_mps": None,
        "heading_accuracy_deg": None,
        "position_dop": None,
        "carrier_solution": None,
        "ubx_nav_pvt_received": False,

        "ntrip_connected": False,
        "ntrip_mountpoint": None,
        "rtcm_bytes_received": 0,
        "rtcm_bytes_written": 0,
        "last_rtcm_utc": None,
        "ntrip_error": None,

        "error": None,
    }


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def safe_float(value: Any) -> float | None:
    if value in (None, ""):
        return None

    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def safe_int(value: Any) -> int | None:
    if value in (None, ""):
        return None

    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def signed_coordinate(
    value: Any,
    direction: Any,
) -> float | None:
    coordinate = safe_float(value)

    if coordinate is None:
        return None

    if direction in ("S", "W"):
        return -abs(coordinate)

    return coordinate


def find_receiver() -> str:
    """Find a likely u-blox USB serial device."""

    for device in sorted(glob.glob("/dev/serial/by-id/*")):
        name = device.lower()

        if "u-blox" in name or "ublox" in name:
            return device

    devices = sorted(glob.glob("/dev/ttyACM*"))

    if len(devices) == 1:
        return devices[0]

    if len(devices) > 1:
        raise RuntimeError(
            "Multiple /dev/ttyACM devices found. "
            "Specify the GNSS port explicitly."
        )

    raise RuntimeError("No u-blox USB receiver found.")


class GnssModule:
    def __init__(
        self,
        serial_port: str | None = None,
        serial_baud: int = DEFAULT_BAUD,
        crtn_csv_path: Path = DEFAULT_CRTN_CSV_PATH,
        reconnect_delay_seconds: float = (
            DEFAULT_RECONNECT_DELAY_SECONDS
        ),
        message_timeout_seconds: float = (
            DEFAULT_MESSAGE_TIMEOUT_SECONDS
        ),
    ) -> None:
        self.serial_port = serial_port
        self.serial_baud = serial_baud
        self.crtn_csv_path = crtn_csv_path
        self.reconnect_delay_seconds = reconnect_delay_seconds
        self.message_timeout_seconds = message_timeout_seconds

        self._state_lock = threading.Lock()
        self._state = initial_gnss_state()
        self._state["serial_baud"] = serial_baud

        self._position_history: deque[dict[str, Any]] = deque(
            maxlen=300
        )

        self._stations = load_crtn_stations(crtn_csv_path)
        self._station_selected = False

        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._monitor_thread: threading.Thread | None = None
        self._serial_stream: serial.Serial | None = None
        self._ntrip_client: NtripClient | None = None

    def start(self) -> None:
        if (
            self._reader_thread is not None
            and self._reader_thread.is_alive()
        ):
            return

        self._stop_event.clear()

        with self._state_lock:
            self._state["service_running"] = True

        self._reader_thread = threading.Thread(
            target=self._reader_worker,
            daemon=True,
            name="gnss-reader",
        )

        self._monitor_thread = threading.Thread(
            target=self._age_monitor,
            daemon=True,
            name="gnss-age-monitor",
        )

        self._reader_thread.start()
        self._monitor_thread.start()

    def stop(self) -> None:
        self._stop_event.set()

        ntrip_client = self._ntrip_client
        if ntrip_client is not None:
            ntrip_client.stop()

        stream = self._serial_stream
        if stream is not None:
            try:
                stream.close()
            except Exception:
                pass

        for thread in (
            self._reader_thread,
            self._monitor_thread,
        ):
            if thread is not None and thread.is_alive():
                thread.join(timeout=3.0)

        with self._state_lock:
            self._state["service_running"] = False
            self._state["receiver_connected"] = False

    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            result = deepcopy(self._state)
            result["history"] = list(self._position_history)

        return result

    def _select_nearest_crtn_station(
        self,
        latitude: float,
        longitude: float,
    ) -> bool:
        candidates = nearest_crtn_stations(
            rover_latitude=latitude,
            rover_longitude=longitude,
            stations=self._stations,
            limit=5,
            max_distance_km=100.0,
        )

        if not candidates:
            with self._state_lock:
                self._state["error"] = (
                    "No suitable CRTN station found "
                    "within 100 km."
                )

            return False

        selected = candidates[0]

        with self._state_lock:
            self._state.update(
                {
                    "selected_station_code": selected.code,
                    "selected_station_name": (
                        selected.station_name
                    ),
                    "selected_mountpoint": selected.mountpoint,
                    "selected_station_distance_km": round(
                        selected.distance_km,
                        2,
                    ),
                    "selected_ntrip_host": selected.host,
                    "selected_ntrip_port": selected.port,
                    "station_candidates": [
                        {
                            "code": station.code,
                            "name": station.station_name,
                            "distance_km": round(
                                station.distance_km,
                                2,
                            ),
                            "mountpoint": station.mountpoint,
                            "host": station.host,
                            "port": station.port,
                        }
                        for station in candidates
                    ],
                }
            )

        return True

    def _update_from_nav_pvt(self, message: Any) -> None:
        h_acc_raw = safe_float(
            getattr(message, "hAcc", None)
        )
        v_acc_raw = safe_float(
            getattr(message, "vAcc", None)
        )
        s_acc_raw = safe_float(
            getattr(message, "sAcc", None)
        )
        head_acc_raw = safe_float(
            getattr(message, "headAcc", None)
        )
        p_dop_raw = safe_float(
            getattr(message, "pDOP", None)
        )
        carr_soln_raw = safe_int(
            getattr(message, "carrSoln", None)
        )

        carrier_names = {
            0: "None",
            1: "RTK float",
            2: "RTK fixed",
        }

        with self._state_lock:
            self._state.update(
                {
                    "horizontal_accuracy_m": (
                        h_acc_raw / 1000.0
                        if h_acc_raw is not None
                        else None
                    ),
                    "vertical_accuracy_m": (
                        v_acc_raw / 1000.0
                        if v_acc_raw is not None
                        else None
                    ),
                    "speed_accuracy_mps": (
                        s_acc_raw / 1000.0
                        if s_acc_raw is not None
                        else None
                    ),
                    "heading_accuracy_deg": head_acc_raw,
                    "position_dop": p_dop_raw,
                    "carrier_solution": carrier_names.get(
                        carr_soln_raw,
                        (
                            f"Unknown ({carr_soln_raw})"
                            if carr_soln_raw is not None
                            else None
                        ),
                    ),
                    "ubx_nav_pvt_received": True,
                }
            )

    def _update_from_gga(self, message: Any) -> None:
        latitude = signed_coordinate(
            getattr(message, "lat", None),
            getattr(message, "NS", ""),
        )
        longitude = signed_coordinate(
            getattr(message, "lon", None),
            getattr(message, "EW", ""),
        )

        quality = (
            safe_int(getattr(message, "quality", 0)) or 0
        )
        satellites = (
            safe_int(getattr(message, "numSV", 0)) or 0
        )
        hdop = safe_float(
            getattr(message, "HDOP", None)
        )
        altitude = safe_float(
            getattr(message, "alt", None)
        )
        correction_age = safe_float(
            getattr(message, "diffAge", None)
        )

        now = utc_now_iso()

        with self._state_lock:
            self._state.update(
                {
                    "receiver_connected": True,
                    "last_message_utc": now,
                    "message_age_seconds": 0,
                    "fix_quality": quality,
                    "fix_name": FIX_NAMES.get(
                        quality,
                        f"Unknown ({quality})",
                    ),
                    "latitude": latitude,
                    "longitude": longitude,
                    "altitude_msl_m": altitude,
                    "satellites": satellites,
                    "hdop": hdop,
                    "correction_age_s": correction_age,
                    "error": None,
                }
            )

            if (
                quality > 0
                and latitude is not None
                and longitude is not None
            ):
                self._position_history.append(
                    {
                        "time": now,
                        "lat": latitude,
                        "lon": longitude,
                        "alt": altitude,
                        "fix": quality,
                    }
                )

        if (
            not self._station_selected
            and quality > 0
            and latitude is not None
            and longitude is not None
        ):
            self._station_selected = (
                self._select_nearest_crtn_station(
                    latitude,
                    longitude,
                )
            )

    def _update_from_rmc(self, message: Any) -> None:
        speed_knots = safe_float(
            getattr(message, "spd", None)
        )
        course = safe_float(
            getattr(message, "cog", None)
        )

        speed_mps = None
        speed_mph = None

        if speed_knots is not None:
            speed_mps = speed_knots * 0.514444
            speed_mph = speed_knots * 1.150779

        with self._state_lock:
            self._state["speed_mps"] = speed_mps
            self._state["speed_mph"] = speed_mph
            self._state["course_deg"] = course

    def _process_message(self, message: Any) -> None:
        identity = getattr(message, "identity", "")

        if identity == "NAV-PVT":
            self._update_from_nav_pvt(message)

        elif identity.endswith("GGA"):
            self._update_from_gga(message)

        elif identity.endswith("RMC"):
            self._update_from_rmc(message)

    def _start_ntrip(
        self,
        stream: serial.Serial,
    ) -> NtripClient | None:
        username = os.environ.get("CRTN_USER")
        password = os.environ.get("CRTN_PASSWORD")

        if not username or not password:
            with self._state_lock:
                self._state["ntrip_connected"] = False
                self._state["ntrip_error"] = (
                    "CRTN credentials are not configured"
                )

            return None

        client = NtripClient(
            serial_stream=stream,
            shared_state=self._state,
            state_lock=self._state_lock,
            username=username,
            password=password,
        )
        client.start()

        return client

    def _reader_worker(self) -> None:
        while not self._stop_event.is_set():
            ntrip_client = None
            stream = None

            try:
                port = self.serial_port or find_receiver()

                print(
                    "Opening GNSS connection on "
                    f"{port} at {self.serial_baud} baud..."
                )

                stream = serial.Serial(
                    port=port,
                    baudrate=self.serial_baud,
                    timeout=1,
                )
                self._serial_stream = stream

                with self._state_lock:
                    self._state["serial_port"] = port
                    self._state["receiver_connected"] = True
                    self._state["error"] = None

                reader = UBXReader(
                    stream,
                    protfilter=(
                        UBX_PROTOCOL
                        | NMEA_PROTOCOL
                        | RTCM3_PROTOCOL
                    ),
                    quitonerror=0,
                )

                ntrip_client = self._start_ntrip(stream)
                self._ntrip_client = ntrip_client

                while not self._stop_event.is_set():
                    raw, message = reader.read()

                    if raw:
                        with self._state_lock:
                            self._state[
                                "bytes_from_receiver"
                            ] += len(raw)

                    if message is not None:
                        self._process_message(message)

            except Exception as exc:
                if not self._stop_event.is_set():
                    print(f"GNSS error: {exc}")

                    with self._state_lock:
                        self._state["receiver_connected"] = False
                        self._state["error"] = str(exc)

                    self._stop_event.wait(
                        self.reconnect_delay_seconds
                    )

            finally:
                if ntrip_client is not None:
                    ntrip_client.stop()

                self._ntrip_client = None

                if stream is not None:
                    try:
                        stream.close()
                    except Exception:
                        pass

                self._serial_stream = None

        with self._state_lock:
            self._state["receiver_connected"] = False
            self._state["service_running"] = False

    def _age_monitor(self) -> None:
        while not self._stop_event.is_set():
            with self._state_lock:
                last = self._state.get("last_message_utc")

                if last:
                    try:
                        last_time = datetime.fromisoformat(last)

                        age = (
                            datetime.now(timezone.utc)
                            - last_time
                        ).total_seconds()

                        self._state[
                            "message_age_seconds"
                        ] = round(age, 1)

                        if age > self.message_timeout_seconds:
                            self._state[
                                "receiver_connected"
                            ] = False

                    except ValueError:
                        pass

            self._stop_event.wait(1.0)


gnss = GnssModule()
