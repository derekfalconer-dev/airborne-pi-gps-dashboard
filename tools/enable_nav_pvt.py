#!/usr/bin/env python3

from __future__ import annotations

import glob
import sys
import time

import serial
from pyubx2 import UBXMessage, UBXReader


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

    # Re-enable USB input/output protocols and the messages required
    # by the dashboard. Apply to RAM only.
    message = UBXMessage.config_set(
        layers=1,
        transaction=0,
        cfgData=[
            ("CFG_USBINPROT_UBX", 1),
            ("CFG_USBINPROT_NMEA", 1),
            ("CFG_USBINPROT_RTCM3X", 1),

            ("CFG_USBOUTPROT_UBX", 1),
            ("CFG_USBOUTPROT_NMEA", 1),
            ("CFG_USBOUTPROT_RTCM3X", 1),

            ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
            ("CFG_MSGOUT_NMEA_ID_GGA_USB", 1),
            ("CFG_MSGOUT_NMEA_ID_RMC_USB", 1),
        ],
    )

    print(f"Opening {port}...")

    try:
        with serial.Serial(
            port=port,
            baudrate=115200,
            timeout=2,
        ) as stream:
            stream.reset_input_buffer()
            stream.write(message.serialize())
            stream.flush()

            reader = UBXReader(stream, quitonerror=0)

            deadline = time.monotonic() + 5

            while time.monotonic() < deadline:
                raw, parsed = reader.read()

                if parsed is None:
                    continue

                print(f"Received: {parsed.identity}")

                if parsed.identity == "ACK-ACK":
                    print("Receiver acknowledged configuration.")
                    time.sleep(1)
                    return 0

                if parsed.identity == "ACK-NAK":
                    print(
                        "Receiver rejected configuration.",
                        file=sys.stderr,
                    )
                    return 1

    except Exception as exc:
        print(f"Could not configure receiver: {exc}", file=sys.stderr)
        return 1

    print(
        "No acknowledgement received from receiver.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
