#!/usr/bin/env python3

from __future__ import annotations

import glob
import sys
import time

import serial
from pyubx2 import SET, UBXMessage


def find_receiver() -> str:
    for device in sorted(glob.glob("/dev/serial/by-id/*")):
        name = device.lower()

        if "u-blox" in name or "ublox" in name:
            return device

    devices = sorted(glob.glob("/dev/ttyACM*"))

    if len(devices) == 1:
        return devices[0]

    raise RuntimeError("Could not uniquely identify the u-blox receiver.")


def main() -> int:
    port = find_receiver()

    # Enable UBX-NAV-PVT on USB once per navigation solution.
    message = UBXMessage.config_set(
        layers=1,
        transaction=0,
        cfgData=[
            ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
        ],
    )

    print(f"Opening {port}...")

    try:
        with serial.Serial(
            port=port,
            baudrate=115200,
            timeout=2,
        ) as stream:
            stream.write(message.serialize())
            stream.flush()
            time.sleep(1)

    except OSError as exc:
        print(f"Could not configure receiver: {exc}", file=sys.stderr)
        return 1

    print("UBX-NAV-PVT output enabled on USB.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
