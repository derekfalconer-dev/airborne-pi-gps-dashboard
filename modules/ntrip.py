#!/usr/bin/env python3

from __future__ import annotations

import base64
import socket
import threading
import time
from typing import Any, BinaryIO


class NtripClient:
    """
    Connect to an NTRIP caster and inject RTCM bytes into a GNSS receiver.

    The serial stream is supplied by the caller. This class does not open
    or close the GNSS serial device.
    """

    def __init__(
        self,
        serial_stream: BinaryIO,
        shared_state: dict[str, Any],
        state_lock: threading.Lock,
        username: str,
        password: str,
        reconnect_delay: float = 5.0,
        socket_timeout: float = 8.0,
    ) -> None:
        self.serial_stream = serial_stream
        self.shared_state = shared_state
        self.state_lock = state_lock
        self.username = username
        self.password = password
        self.reconnect_delay = reconnect_delay
        self.socket_timeout = socket_timeout

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()

        self._thread = threading.Thread(
            target=self._run,
            name="ntrip-client",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def _set_state(self, **updates: Any) -> None:
        with self.state_lock:
            self.shared_state.update(updates)

    def _get_selected_station(
        self,
    ) -> tuple[str, int, str] | None:
        with self.state_lock:
            host = self.shared_state.get("selected_ntrip_host")
            port = self.shared_state.get("selected_ntrip_port")
            mountpoint = self.shared_state.get("selected_mountpoint")

        if not host or not port or not mountpoint:
            return None

        return str(host), int(port), str(mountpoint)

    def _build_request(
        self,
        host: str,
        port: int,
        mountpoint: str,
    ) -> bytes:
        credentials = base64.b64encode(
            f"{self.username}:{self.password}".encode("utf-8")
        ).decode("ascii")

        # NTRIP v1-style request. CRTN's caster may return "ICY 200 OK".
        request = (
            f"GET /{mountpoint} HTTP/1.0\r\n"
            f"Host: {host}:{port}\r\n"
            "User-Agent: NTRIP AirbornePi/1.0\r\n"
            f"Authorization: Basic {credentials}\r\n"
            "Accept: */*\r\n"
            "\r\n"
        )

        return request.encode("ascii")

    def _read_response_header(
        self,
        sock: socket.socket,
    ) -> tuple[bytes, bytes]:
        """
        Return (header, initial_payload).

        Supports:
        - ICY 200 OK
        - HTTP/1.x 200 OK
        """

        received = bytearray()

        while len(received) < 8192:
            chunk = sock.recv(1024)

            if not chunk:
                raise ConnectionError(
                    "Caster closed the connection before sending a response."
                )

            received.extend(chunk)
            raw = bytes(received)

            if raw.startswith(b"ICY "):
                line_end = raw.find(b"\r\n")

                if line_end >= 0:
                    header = raw[:line_end]
                    payload = raw[line_end + 2:]
                    return header, payload

            header_end = raw.find(b"\r\n\r\n")

            if header_end >= 0:
                header = raw[:header_end]
                payload = raw[header_end + 4:]
                return header, payload

        raise ConnectionError("NTRIP response header was too large.")

    def _validate_response(self, header: bytes) -> None:
        first_line = header.split(b"\r\n", 1)[0]
        display = first_line.decode("ascii", errors="replace")

        if b"200" in first_line:
            return

        if b"401" in first_line:
            raise PermissionError(
                f"NTRIP authentication rejected: {display}"
            )

        if b"404" in first_line:
            raise ConnectionError(
                f"NTRIP mountpoint not found: {display}"
            )

        raise ConnectionError(
            f"Unexpected NTRIP response: {display}"
        )

    def _inject_rtcm(self, data: bytes) -> None:
        if not data:
            return

        written = self.serial_stream.write(data)
        self.serial_stream.flush()

        with self.state_lock:
            self.shared_state["rtcm_bytes_received"] += len(data)
            self.shared_state["rtcm_bytes_written"] += written
            self.shared_state["last_rtcm_utc"] = time.time()

    def _connect_and_stream(
        self,
        host: str,
        port: int,
        mountpoint: str,
    ) -> None:
        self._set_state(
            ntrip_connected=False,
            ntrip_mountpoint=mountpoint,
            ntrip_error=None,
        )

        with socket.create_connection(
            (host, port),
            timeout=self.socket_timeout,
        ) as sock:
            sock.settimeout(self.socket_timeout)

            request = self._build_request(
                host,
                port,
                mountpoint,
            )
            sock.sendall(request)

            header, initial_payload = self._read_response_header(sock)
            self._validate_response(header)

            self._set_state(
                ntrip_connected=True,
                ntrip_mountpoint=mountpoint,
                ntrip_error=None,
            )

            self._inject_rtcm(initial_payload)

            while not self._stop_event.is_set():
                data = sock.recv(4096)

                if not data:
                    raise ConnectionError(
                        "NTRIP caster closed the correction stream."
                    )

                self._inject_rtcm(data)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            station = self._get_selected_station()

            if station is None:
                self._set_state(
                    ntrip_connected=False,
                    ntrip_error="Waiting for station selection",
                )
                self._stop_event.wait(1.0)
                continue

            host, port, mountpoint = station

            try:
                self._connect_and_stream(
                    host,
                    port,
                    mountpoint,
                )

            except Exception as exc:
                self._set_state(
                    ntrip_connected=False,
                    ntrip_error=str(exc),
                )

                self._stop_event.wait(self.reconnect_delay)

        self._set_state(ntrip_connected=False)
