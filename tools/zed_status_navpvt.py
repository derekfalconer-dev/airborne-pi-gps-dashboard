#!/usr/bin/env python3

"""
Read UBX-NAV-PVT navigation solutions from a u-blox ZED-F9P.

Reports:
- UTC time
- Position fix type
- RTK float/fixed status
- Latitude and longitude
- Height above mean sea level
- Horizontal and vertical accuracy estimates
- Ground speed
- Course over ground
- Number of satellites
- Position DOP
"""

from __future__ import annotations

import argparse
import glob
import sys
import time
from pathlib import Path

import serial
from pyubx2 import (
    UBXReader,
    UBX_PROTOCOL,
    NMEA_PROTOCOL,
)


FIX_TYPES = {
    0: "No fix",
    1: "Dead reckoning only",
    2: "2D fix",
    3: "3D fix",
    4: "GNSS + dead reckoning",
    5: "Time-only fix",
}

CARRIER_SOLUTIONS = {
    0: "No RTK solution",
    1: "RTK float",
    2: "RTK fixed",
}


def find_receiver() -> str:
    """Locate a likely u-blox USB serial device."""

    by_id = sorted(glob.glob("/dev/serial/by-id/*"))

    # Prefer device names that clearly look like u-blox hardware.
    for device in by_id:
        name = Path(device).name.lower()
        if "u-blox" in name or "ublox" in name or "zed" in name:
            return device

    # If there is exactly one persistent USB serial device, use it.
    if len(by_id) == 1:
        return by_id[0]

    # Fall back to common USB CDC serial names.
    acm_devices = sorted(glob.glob("/dev/ttyACM*"))
    if len(acm_devices) == 1:
        return acm_devices[0]

    if len(acm_devices) > 1:
        raise RuntimeError(
            "Multiple /dev/ttyACM devices found. "
            "Specify the receiver explicitly with --port."
        )

    raise RuntimeError(
        "No ZED-F9P serial device found. Check the USB data cable and run:\n"
        "  lsusb\n"
        "  ls -l /dev/serial/by-id/\n"
        "  dmesg | tail -30"
    )


def value(message, field: str, default=None):
    """Safely read a pyubx2 message attribute."""
    return getattr(message, field, default)


def format_utc(message) -> str:
    year = value(message, "year")
    month = value(message, "month")
    day = value(message, "day")
    hour = value(message, "hour")
    minute = value(message, "min")
    second = value(message, "second")

    if None in (year, month, day, hour, minute, second):
        return "Unknown"

    return (
        f"{year:04d}-{month:02d}-{day:02d} "
        f"{hour:02d}:{minute:02d}:{second:02d} UTC"
    )


def print_solution(message) -> None:
    fix_type_number = int(value(message, "fixType", 0))
    carrier_number = int(value(message, "carrSoln", 0))

    gnss_fix_ok = bool(value(message, "gnssFixOk", 0))
    differential = bool(value(message, "diffSoln", 0))

    latitude = value(message, "lat")
    longitude = value(message, "lon")
    altitude_msl = value(message, "hMSL")
    horizontal_accuracy = value(message, "hAcc")
    vertical_accuracy = value(message, "vAcc")
    ground_speed = value(message, "gSpeed")
    heading = value(message, "headMot")
    satellites = value(message, "numSV")
    pdop = value(message, "pDOP")

    print("\033[2J\033[H", end="")  # Clear terminal.
    print("ZED-F9P Navigation Solution")
    print("=" * 45)
    print(f"Receiver time:       {format_utc(message)}")
    print(
        f"Navigation fix:      "
        f"{FIX_TYPES.get(fix_type_number, f'Unknown ({fix_type_number})')}"
    )
    print(f"Fix valid:           {'YES' if gnss_fix_ok else 'NO'}")
    print(
        f"Carrier solution:    "
        f"{CARRIER_SOLUTIONS.get(carrier_number, f'Unknown ({carrier_number})')}"
    )
    print(f"Differential data:   {'YES' if differential else 'NO'}")
    print(f"Satellites used:     {satellites}")
    print(f"Position DOP:        {pdop}")

    if latitude is not None and longitude is not None:
        print(f"Latitude:            {latitude:.9f} deg")
        print(f"Longitude:           {longitude:.9f} deg")

    if altitude_msl is not None:
        print(f"Altitude MSL:        {altitude_msl:.3f} m")

    if horizontal_accuracy is not None:
        print(f"Horizontal accuracy: {horizontal_accuracy:.3f} m")

    if vertical_accuracy is not None:
        print(f"Vertical accuracy:   {vertical_accuracy:.3f} m")

    if ground_speed is not None:
        print(
            f"Ground speed:        {ground_speed:.3f} m/s "
            f"({ground_speed * 2.236936:.2f} mph)"
        )

    if heading is not None:
        print(f"Course over ground:  {heading:.2f} deg")

    print("=" * 45)

    if not gnss_fix_ok:
        print("Waiting for a valid GNSS fix...")
    elif carrier_number == 0:
        print("Standard GNSS fix; no usable RTK correction solution.")
    elif carrier_number == 1:
        print("RTK FLOAT: corrections received, ambiguities not fixed.")
    elif carrier_number == 2:
        print("RTK FIXED: centimeter-class carrier solution available.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Display ZED-F9P navigation and RTK status."
    )
    parser.add_argument(
        "--port",
        help="Serial port, such as /dev/ttyACM0 or /dev/serial/by-id/...",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. USB CDC generally ignores this value.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="Seconds before warning that no NAV-PVT message was received.",
    )
    args = parser.parse_args()

    try:
        port = args.port or find_receiver()
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Opening receiver on {port}")

    try:
        stream = serial.Serial(
            port=port,
            baudrate=args.baud,
            timeout=1,
        )
    except serial.SerialException as exc:
        print(f"Could not open {port}: {exc}", file=sys.stderr)
        print(
            "Check permissions and whether gpsd or another program "
            "already has the port open.",
            file=sys.stderr,
        )
        return 1

    reader = UBXReader(
        stream,
        protfilter=UBX_PROTOCOL | NMEA_PROTOCOL,
        quitonerror=0,
    )

    last_pvt = time.monotonic()

    try:
        while True:
            raw_data, parsed = reader.read()

            if parsed is None:
                if time.monotonic() - last_pvt > args.timeout:
                    print(
                        "\rConnected, but no navigation message received...",
                        end="",
                        flush=True,
                    )
                continue

            if getattr(parsed, "identity", "") == "NAV-PVT":
                last_pvt = time.monotonic()
                print_solution(parsed)

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except serial.SerialException as exc:
        print(f"\nSerial connection failed: {exc}", file=sys.stderr)
        return 1
    finally:
        stream.close()


if __name__ == "__main__":
    raise SystemExit(main())
