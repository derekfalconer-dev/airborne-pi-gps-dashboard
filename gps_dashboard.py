#!/usr/bin/env python3

from __future__ import annotations

import argparse
import glob
import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any

import serial
from flask import Flask, jsonify, render_template_string
from pyubx2 import NMEA_PROTOCOL, RTCM3_PROTOCOL, UBX_PROTOCOL, UBXReader


app = Flask(__name__)

state_lock = threading.Lock()

gps_state: dict[str, Any] = {
    "receiver_connected": False,
    "serial_port": None,
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

    # NTRIP placeholders for the next step.
    "ntrip_connected": False,
    "ntrip_mountpoint": None,
    "rtcm_bytes_received": 0,
    "rtcm_bytes_written": 0,
    "ntrip_error": None,

    "error": None,
}

position_history: deque[dict[str, Any]] = deque(maxlen=300)


FIX_NAMES = {
    0: "No fix",
    1: "Standalone GNSS",
    2: "Differential GNSS",
    3: "PPS fix",
    4: "RTK fixed",
    5: "RTK float",
    6: "Dead reckoning",
}


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
            "Multiple /dev/ttyACM devices found. Specify --port explicitly."
        )

    raise RuntimeError("No u-blox USB receiver found.")


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


def signed_coordinate(value: Any, direction: Any) -> float | None:
    coordinate = safe_float(value)

    if coordinate is None:
        return None

    if direction in ("S", "W"):
        return -abs(coordinate)

    return coordinate


def update_from_gga(message: Any) -> None:
    latitude = signed_coordinate(
        getattr(message, "lat", None),
        getattr(message, "NS", ""),
    )
    longitude = signed_coordinate(
        getattr(message, "lon", None),
        getattr(message, "EW", ""),
    )

    quality = safe_int(getattr(message, "quality", 0)) or 0
    satellites = safe_int(getattr(message, "numSV", 0)) or 0
    hdop = safe_float(getattr(message, "HDOP", None))
    altitude = safe_float(getattr(message, "alt", None))
    correction_age = safe_float(getattr(message, "diffAge", None))

    now = utc_now_iso()

    with state_lock:
        gps_state.update(
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
            position_history.append(
                {
                    "time": now,
                    "lat": latitude,
                    "lon": longitude,
                    "alt": altitude,
                    "fix": quality,
                }
            )


def update_from_rmc(message: Any) -> None:
    # NMEA RMC speed is normally in knots.
    speed_knots = safe_float(getattr(message, "spd", None))
    course = safe_float(getattr(message, "cog", None))

    speed_mps = None
    speed_mph = None

    if speed_knots is not None:
        speed_mps = speed_knots * 0.514444
        speed_mph = speed_knots * 1.150779

    with state_lock:
        gps_state["speed_mps"] = speed_mps
        gps_state["speed_mph"] = speed_mph
        gps_state["course_deg"] = course


def serial_reader(port: str, baud: int) -> None:
    while True:
        try:
            with serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1,
            ) as stream:

                with state_lock:
                    gps_state["serial_port"] = port
                    gps_state["receiver_connected"] = True
                    gps_state["error"] = None

                reader = UBXReader(
                    stream,
                    protfilter=(
                        UBX_PROTOCOL
                        | NMEA_PROTOCOL
                        | RTCM3_PROTOCOL
                    ),
                    quitonerror=0,
                )

                while True:
                    raw, message = reader.read()

                    if raw:
                        with state_lock:
                            gps_state["bytes_from_receiver"] += len(raw)

                    if message is None:
                        continue

                    identity = getattr(message, "identity", "")

                    if identity.endswith("GGA"):
                        update_from_gga(message)

                    elif identity.endswith("RMC"):
                        update_from_rmc(message)

        except Exception as exc:
            with state_lock:
                gps_state["receiver_connected"] = False
                gps_state["error"] = str(exc)

            time.sleep(2)


def age_monitor() -> None:
    while True:
        with state_lock:
            last = gps_state.get("last_message_utc")

            if last:
                try:
                    last_time = datetime.fromisoformat(last)
                    age = (
                        datetime.now(timezone.utc) - last_time
                    ).total_seconds()

                    gps_state["message_age_seconds"] = round(age, 1)

                    if age > 5:
                        gps_state["receiver_connected"] = False

                except ValueError:
                    pass

        time.sleep(1)


@app.route("/")
def index():
    return render_template_string(DASHBOARD_HTML)


@app.route("/api/status")
def api_status():
    with state_lock:
        result = dict(gps_state)
        result["history"] = list(position_history)

    return jsonify(result)


DASHBOARD_HTML = r"""
<!doctype html>
<html lang="en">
<head>
    <meta charset="utf-8">
    <meta
        name="viewport"
        content="width=device-width, initial-scale=1"
    >

    <title>ZED-F9P Dashboard</title>

    <link
        rel="stylesheet"
        href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    >

    <style>
        :root {
            color-scheme: dark;
            font-family:
                system-ui,
                -apple-system,
                BlinkMacSystemFont,
                "Segoe UI",
                sans-serif;
        }

        body {
            margin: 0;
            background: #10151c;
            color: #eef3f8;
        }

        header {
            padding: 18px 24px;
            background: #17202a;
            border-bottom: 1px solid #2c3947;
        }

        header h1 {
            margin: 0;
            font-size: 24px;
        }

        header p {
            margin: 4px 0 0;
            color: #9fb0c1;
        }

        main {
            max-width: 1400px;
            margin: auto;
            padding: 20px;
        }

        .status-banner {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 16px;
            margin-bottom: 18px;
            padding: 14px 18px;
            border-radius: 10px;
            background: #17202a;
            border: 1px solid #2c3947;
        }

        .status-dot {
            display: inline-block;
            width: 12px;
            height: 12px;
            margin-right: 8px;
            border-radius: 50%;
            background: #d9534f;
        }

        .connected {
            background: #36c275;
        }

        .grid {
            display: grid;
            grid-template-columns:
                repeat(auto-fit, minmax(180px, 1fr));
            gap: 14px;
            margin-bottom: 18px;
        }

        .card {
            background: #17202a;
            border: 1px solid #2c3947;
            border-radius: 10px;
            padding: 16px;
        }

        .label {
            color: #91a3b5;
            font-size: 13px;
            text-transform: uppercase;
            letter-spacing: 0.06em;
        }

        .value {
            margin-top: 7px;
            font-size: 24px;
            font-weight: 700;
            overflow-wrap: anywhere;
        }

        .small-value {
            font-size: 17px;
        }

        .map-card {
            background: #17202a;
            border: 1px solid #2c3947;
            border-radius: 10px;
            overflow: hidden;
        }

        .map-header {
            padding: 14px 16px;
        }

        #map {
            height: 500px;
            background: #202a34;
        }

        .fix-0 {
            color: #e66b65;
        }

        .fix-1,
        .fix-2 {
            color: #f0c75e;
        }

        .fix-5 {
            color: #66b8ff;
        }

        .fix-4 {
            color: #49d17f;
        }

        .error {
            margin-top: 12px;
            color: #ff837d;
            font-family: monospace;
        }

        @media (max-width: 700px) {
            #map {
                height: 360px;
            }

            .value {
                font-size: 20px;
            }
        }
    </style>
</head>

<body>
<header>
    <h1>ZED-F9P GNSS Dashboard</h1>
    <p>Live receiver and RTK debugging</p>
</header>

<main>
    <div class="status-banner">
        <div>
            <span id="status-dot" class="status-dot"></span>
            <strong id="connection">Waiting for receiver</strong>
        </div>

        <div id="message-age">No messages yet</div>
    </div>

    <section class="grid">
        <div class="card">
            <div class="label">Fix</div>
            <div id="fix" class="value">—</div>
        </div>

        <div class="card">
            <div class="label">Satellites</div>
            <div id="satellites" class="value">—</div>
        </div>

        <div class="card">
            <div class="label">HDOP</div>
            <div id="hdop" class="value">—</div>
        </div>

        <div class="card">
            <div class="label">Altitude MSL</div>
            <div id="altitude" class="value">—</div>
        </div>

        <div class="card">
            <div class="label">Speed</div>
            <div id="speed" class="value">—</div>
        </div>

        <div class="card">
            <div class="label">Course</div>
            <div id="course" class="value">—</div>
        </div>

        <div class="card">
            <div class="label">Latitude</div>
            <div id="latitude" class="value small-value">—</div>
        </div>

        <div class="card">
            <div class="label">Longitude</div>
            <div id="longitude" class="value small-value">—</div>
        </div>

        <div class="card">
            <div class="label">Correction age</div>
            <div id="correction-age" class="value">—</div>
        </div>

        <div class="card">
            <div class="label">NTRIP</div>
            <div id="ntrip" class="value small-value">Not connected</div>
        </div>

        <div class="card">
            <div class="label">RTCM received</div>
            <div id="rtcm-received" class="value small-value">0 bytes</div>
        </div>

        <div class="card">
            <div class="label">Receiver data</div>
            <div id="receiver-bytes" class="value small-value">0 bytes</div>
        </div>
    </section>

    <section class="map-card">
        <div class="map-header">
            <strong>Position and recent breadcrumb trail</strong>
            <div id="error" class="error"></div>
        </div>

        <div id="map"></div>
    </section>
</main>

<script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js">
</script>

<script>
    const map = L.map("map").setView([32.94, -117.02], 14);

    L.tileLayer(
        "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
        {
            maxZoom: 20,
            attribution: "&copy; OpenStreetMap contributors"
        }
    ).addTo(map);

    const marker = L.circleMarker(
        [32.94, -117.02],
        {
            radius: 8,
            weight: 3
        }
    ).addTo(map);

    const trail = L.polyline([], {
        weight: 3
    }).addTo(map);

    let mapCentered = false;

    function text(id, value) {
        document.getElementById(id).textContent = value;
    }

    function numberOrDash(value, decimals = 2) {
        if (value === null || value === undefined) {
            return "—";
        }

        return Number(value).toFixed(decimals);
    }

    function byteCount(bytes) {
        if (!bytes) {
            return "0 bytes";
        }

        if (bytes < 1024) {
            return `${bytes} bytes`;
        }

        if (bytes < 1024 * 1024) {
            return `${(bytes / 1024).toFixed(1)} KB`;
        }

        return `${(bytes / 1024 / 1024).toFixed(1)} MB`;
    }

    async function refresh() {
        try {
            const response = await fetch(
                "/api/status",
                {cache: "no-store"}
            );

            const data = await response.json();

            const connected = data.receiver_connected === true;

            text(
                "connection",
                connected
                    ? `Connected: ${data.serial_port}`
                    : "Receiver disconnected"
            );

            document
                .getElementById("status-dot")
                .classList.toggle("connected", connected);

            if (data.message_age_seconds !== null) {
                text(
                    "message-age",
                    `Last message ${data.message_age_seconds}s ago`
                );
            }

            const fix = document.getElementById("fix");
            fix.textContent = data.fix_name || "—";
            fix.className = `value fix-${data.fix_quality}`;

            text("satellites", data.satellites ?? "—");
            text("hdop", numberOrDash(data.hdop, 2));

            text(
                "altitude",
                data.altitude_msl_m === null
                    ? "—"
                    : `${Number(data.altitude_msl_m).toFixed(2)} m`
            );

            text(
                "speed",
                data.speed_mph === null
                    ? "—"
                    : `${Number(data.speed_mph).toFixed(2)} mph`
            );

            text(
                "course",
                data.course_deg === null
                    ? "—"
                    : `${Number(data.course_deg).toFixed(1)}°`
            );

            text(
                "latitude",
                numberOrDash(data.latitude, 9)
            );

            text(
                "longitude",
                numberOrDash(data.longitude, 9)
            );

            text(
                "correction-age",
                data.correction_age_s === null
                    ? "—"
                    : `${data.correction_age_s} s`
            );

            text(
                "ntrip",
                data.ntrip_connected
                    ? `Connected: ${data.ntrip_mountpoint}`
                    : "Not connected"
            );

            text(
                "rtcm-received",
                byteCount(data.rtcm_bytes_received)
            );

            text(
                "receiver-bytes",
                byteCount(data.bytes_from_receiver)
            );

            text("error", data.error || data.ntrip_error || "");

            if (
                data.latitude !== null
                && data.longitude !== null
            ) {
                const point = [
                    data.latitude,
                    data.longitude
                ];

                marker.setLatLng(point);

                if (!mapCentered) {
                    map.setView(point, 18);
                    mapCentered = true;
                }
            }

            if (Array.isArray(data.history)) {
                const points = data.history.map(
                    item => [item.lat, item.lon]
                );

                trail.setLatLngs(points);
            }

        } catch (error) {
            text("connection", "Dashboard API unavailable");
            text("error", String(error));
        }
    }

    refresh();
    setInterval(refresh, 1000);
</script>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ZED-F9P Flask debugging dashboard."
    )

    parser.add_argument(
        "--port",
        help="Serial port. Defaults to automatic detection.",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
    )

    parser.add_argument(
        "--host",
        default="0.0.0.0",
    )

    parser.add_argument(
        "--web-port",
        type=int,
        default=5000,
    )

    args = parser.parse_args()

    port = args.port or find_receiver()

    reader_thread = threading.Thread(
        target=serial_reader,
        args=(port, args.baud),
        daemon=True,
    )
    reader_thread.start()

    monitor_thread = threading.Thread(
        target=age_monitor,
        daemon=True,
    )
    monitor_thread.start()

    app.run(
        host=args.host,
        port=args.web_port,
        debug=False,
        threaded=True,
    )


if __name__ == "__main__":
    main()
