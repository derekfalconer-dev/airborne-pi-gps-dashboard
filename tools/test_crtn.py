#!/usr/bin/env python3

from __future__ import annotations

import base64
import os
import socket
import sys
import time


HOST = "132.239.152.4"
PORT = 2105
MOUNTPOINT = "CMRP_RTCM3"


def main() -> int:
    username = "REDACTED"
    password = "REDACTED"

    if not username or not password:
        print(
            "CRTN_USER or CRTN_PASSWORD is not set.",
            file=sys.stderr,
        )
        return 1

    encoded_credentials = base64.b64encode(
        f"{username}:{password}".encode("utf-8")
    ).decode("ascii")

    # NTRIP v1-style request. Many GNSS casters return:
    # ICY 200 OK
    request = (
        f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
        f"Host: {HOST}:{PORT}\r\n"
        "User-Agent: NTRIP AirbornePi/1.0\r\n"
        f"Authorization: Basic {encoded_credentials}\r\n"
        "Accept: */*\r\n"
        "Connection: close\r\n"
        "\r\n"
    ).encode("ascii")

    print(f"Connecting to {HOST}:{PORT}...")
    print(f"Requesting mountpoint {MOUNTPOINT}...")

    try:
        with socket.create_connection(
            (HOST, PORT),
            timeout=10,
        ) as sock:
            sock.settimeout(10)
            sock.sendall(request)

            received = bytearray()
            start = time.monotonic()

            # Collect enough data to include the response header and
            # the beginning of the RTCM stream.
            while len(received) < 2048:
                try:
                    chunk = sock.recv(2048)
                except socket.timeout:
                    break

                if not chunk:
                    break

                received.extend(chunk)

                if time.monotonic() - start > 10:
                    break

    except OSError as exc:
        print(f"Network connection failed: {exc}", file=sys.stderr)
        return 1

    if not received:
        print("Connected, but the caster returned no data.")
        return 1

    # NTRIP casters may use either HTTP headers or the older
    # single-line ICY response.
    first_line = bytes(received).split(b"\r\n", 1)[0]
    print(f"Response: {first_line.decode('ascii', errors='replace')}")

    if (
        b"200 OK" not in first_line
        and not bytes(received).startswith(b"ICY 200")
    ):
        text = bytes(received[:1000]).decode(
            "utf-8",
            errors="replace",
        )

        print("\nCaster response:")
        print(text)

        if b"401" in received:
            print(
                "\nAuthentication failed. Check username, password, "
                "capitalization, and account status."
            )
        elif b"404" in received:
            print(
                "\nMountpoint was not found. Check the mountpoint name."
            )
        elif b"SOURCETABLE" in received:
            print(
                "\nThe caster returned its source table instead of "
                "the requested stream."
            )

        return 1

    # Locate the beginning of the binary payload.
    raw = bytes(received)

    if raw.startswith(b"ICY 200"):
        # Typical NTRIP v1 response is one status line followed by data.
        separator = raw.find(b"\r\n")
        payload = raw[separator + 2:] if separator >= 0 else b""
    else:
        separator = raw.find(b"\r\n\r\n")
        payload = raw[separator + 4:] if separator >= 0 else b""

    print("Authentication accepted.")

    if payload:
        print(f"Received {len(payload)} initial correction bytes.")
        print(f"First bytes: {payload[:32].hex(' ')}")

        # RTCM3 messages generally begin with the 0xD3 preamble.
        if b"\xd3" in payload:
            print("RTCM3 preamble 0xD3 detected.")
            print("\nCRTN login and correction stream are working.")
            return 0

        print(
            "\nLogin succeeded, but no RTCM3 0xD3 preamble was "
            "found in this initial sample."
        )
        print(
            "The stream may simply need a few more seconds, or the "
            "response framing may differ."
        )
        return 0

    print(
        "Login succeeded, but no correction payload arrived during "
        "the test period."
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
