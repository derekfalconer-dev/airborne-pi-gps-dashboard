#!/usr/bin/env python3

from __future__ import annotations

import argparse
import glob
import sys
from datetime import datetime

import serial
from pyubx2 import (
    UBXReader,
    UBX_PROTOCOL,
    NMEA_PROTOCOL,
    RTCM3_PROTOCOL,
)


def find_receiver() -> str:
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))

    for device in by_id:
        lower = device.lower()
        if "u-blox" in lower or "ublox" in lower:
            return device

    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[0]

    raise RuntimeError("No u-blox receiver found.")


def nmea_coordinate(value, direction):
    """
    pyubx2 normally converts NMEA latitude and longitude to decimal
    degrees, but this safely handles missing values.
    """
    if value in (None, ""):
        return None

    coordinate = float(value)

    if direction in ("S", "W"):
        coordinate = -abs(coordinate)

    return coordinate


def clear_screen():
    print("\033[2J\033[H", end="")


def print_gga(message):
    latitude = nmea_coordinate(
        getattr(message, "lat", None),
        getattr(message, "NS", ""),
    )
    longitude = nmea_coordinate(
        getattr(message, "lon", None),
        getattr(message, "EW", ""),
    )

    quality = int(getattr(message, "quality", 0) or 0)
    satellites = int(getattr(message, "numSV", 0) or 0)

    hdop = getattr(message, "HDOP", None)
    altitude = getattr(message, "alt", None)
    correction_age = getattr(message, "diffAge", None)

    quality_names = {
        0: "No fix",
        1: "Standalone GNSS",
        2: "Differential GNSS",
        4: "RTK fixed",
        5: "RTK float",
        6: "Dead reckoning",
    }

    clear_screen()

    print("ZED-F9P Navigation Status")
    print("=" * 48)
    print(
        f"Fix:                 "
        f"{quality_names.get(quality, f'Unknown ({quality})')}"
    )
    print(f"Satellites used:     {satellites}")

    if hdop not in (None, ""):
        print(f"Horizontal DOP:      {float(hdop):.2f}")

    if latitude is not None:
        print(f"Latitude:            {latitude:.9f}°")

    if longitude is not None:
        print(f"Longitude:           {longitude:.9f}°")

    if altitude not in (None, ""):
        print(f"Altitude MSL:        {float(altitude):.3f} m")

    if correction_age not in (None, ""):
        print(f"RTCM correction age: {correction_age} s")
    else:
        print("RTCM corrections:    None reported")

    print("=" * 48)

    if quality == 0:
        print("Receiver connected, waiting for a GNSS fix.")
    elif quality == 1:
        print("Valid standalone GNSS fix; RTK corrections not active.")
    elif quality == 2:
        print("Differential GNSS solution.")
    elif quality == 4:
        print("RTK FIXED solution.")
    elif quality == 5:
        print("RTK FLOAT solution.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    try:
        port = args.port or find_receiver()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1

    print(f"Opening receiver on {port}", flush=True)
    print("Waiting for GGA navigation messages...", flush=True)

    try:
        with serial.Serial(port, args.baud, timeout=1) as stream:
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

                if message is None:
                    continue

                identity = getattr(message, "identity", "")

                if identity.endswith("GGA"):
                    print_gga(message)

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
