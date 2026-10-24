#!/usr/bin/env python3
"""Serve the robot's diagnostics page on localhost over USB serial.

Every browser request goes down the USB cable to the robot, which replays it against its own web
server (src/usb_bridge.cpp) and streams the raw response back. The robot's log lines print here,
and to --log when given. Nothing on the robot needs WiFi.

    venv/bin/python firmware/mr_stabs_mk2/scripts/usb_dashboard.py
    # then open http://localhost:8080
"""

import argparse
import base64
import binascii
import glob
import itertools
import queue
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import BinaryIO

import serial

FRAME_START = b"\x1e"
RESPONSE_TIMEOUT_S = 15.0
# A GET waits this long before it is sent again. The robot replays requests over loopback TCP, which
# now and then stalls a response for 10 s (a dropped segment waiting out its retransmit timer).
# Answers are held for at most about 3 s while an ESC exchange runs.
GET_TIMEOUT_S = 4.5
GET_TRIES = 3
SLOW_S = 2.0  # requests slower than this are noted in the log


class DamagedFrameError(Exception):
    """A frame of this stream failed its CRC or did not parse."""


Item = bytes | DamagedFrameError | None  # response bytes, damage, or the end of the stream


class Bridge:
    def __init__(self, port: str, log: BinaryIO | None) -> None:
        # The kernel raises DTR and RTS on open. pyserial then drops DTR before RTS, and DTR low
        # with RTS high resets the chip through the USB-Serial/JTAG unit, so leave DTR up.
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = 115200
        self._serial.rts = False
        self._serial.timeout = 0.05
        self._serial.open()
        self._log = log
        self._write_lock = threading.Lock()
        self._streams: dict[int, queue.Queue[Item]] = {}
        self._streams_lock = threading.Lock()
        self._ids = itertools.count(1)
        threading.Thread(target=self._read_loop, daemon=True).start()

    def open(self, request: bytes) -> tuple[int, "queue.Queue[Item]"]:
        stream_id = next(self._ids)
        stream: queue.Queue[Item] = queue.Queue()
        with self._streams_lock:
            self._streams[stream_id] = stream
        self._send(b"Q %d " % stream_id + base64.b64encode(request))
        return stream_id, stream

    def close(self, stream_id: int) -> None:
        with self._streams_lock:
            self._streams.pop(stream_id, None)
        self._send(b"C %d" % stream_id)

    def _send(self, body: bytes) -> None:
        with self._write_lock:
            self._serial.write(FRAME_START + body + b"\n")

    def _read_loop(self) -> None:
        pending = b""
        while True:
            pending += self._serial.read(4096)
            *lines, pending = pending.split(b"\n")
            for line in lines:
                self._handle_line(line)

    def _handle_line(self, line: bytes) -> None:
        start = line.find(FRAME_START)
        if start < 0:
            self._print_log(line)
            return
        if start > 0:
            self._print_log(line[:start])
        parts = line[start + 1 :].split(b" ")
        try:
            kind, stream_id = parts[0], int(parts[1])
        except (IndexError, ValueError):
            # Which stream lost the frame is unknown, so all of them are suspect.
            self._print_log(b"usb_dashboard: unreadable frame: " + line[:80])
            for other in self._all_streams():
                other.put(DamagedFrameError())
            return
        with self._streams_lock:
            stream = self._streams.get(stream_id)
        if stream is None:
            return
        if kind == b"E":
            stream.put(None)
            return
        try:
            data = base64.b64decode(parts[2], validate=True)
            if binascii.crc32(data) != int(parts[3], 16):
                raise ValueError("CRC mismatch")
        except (IndexError, ValueError) as e:
            self._print_log(b"usb_dashboard: damaged frame (%s): %s" % (str(e).encode(), line[:60]))
            stream.put(DamagedFrameError())
            return
        stream.put(data)

    def note(self, text: str) -> None:
        self._print_log(b"usb_dashboard: " + text.encode())

    def _all_streams(self) -> list["queue.Queue[Item]"]:
        with self._streams_lock:
            return list(self._streams.values())

    def _print_log(self, line: bytes) -> None:
        text = line.rstrip(b"\r")
        if not text:
            return
        stamped = time.strftime("%H:%M:%S ").encode() + text + b"\n"
        sys.stdout.buffer.write(stamped)
        sys.stdout.flush()
        if self._log is not None:
            self._log.write(stamped)
            self._log.flush()


def response_size(received: bytes) -> int | None:
    """Total bytes of a response with a Content-Length, once its head has arrived, else None.

    The robot closes each connection after the response, but its side notices the close late, so
    the bridge closes it as soon as the body is complete instead.
    """
    end = received.find(b"\r\n\r\n")
    if end < 0:
        return None
    for header in received[:end].split(b"\r\n")[1:]:
        name, _, value = header.partition(b":")
        if name.strip().lower() == b"content-length":
            return end + 4 + int(value)
    return None


def build_request(handler: BaseHTTPRequestHandler) -> bytes:
    """The browser's request, rewritten for the robot's own server."""
    length = int(handler.headers.get("Content-Length", 0))
    body = handler.rfile.read(length) if length else b""
    lines = [f"{handler.command} {handler.path} HTTP/1.1", "Host: 127.0.0.1", "Connection: close"]
    for name in ("Content-Type", "Accept", "Cache-Control"):
        if name in handler.headers:
            lines.append(f"{name}: {handler.headers[name]}")
    if body:
        lines.append(f"Content-Length: {len(body)}")
    return ("\r\n".join(lines) + "\r\n\r\n").encode() + body


def fetch(bridge: Bridge, request: bytes, what: str, timeout_s: float) -> bytes | None:
    """One request's whole response, or None when it came back damaged, short, or not at all."""
    stream_id, stream = bridge.open(request)
    started = time.monotonic()
    response = b""
    expected: int | None = None
    try:
        while expected is None or len(response) < expected:
            item = stream.get(timeout=max(0.0, started + timeout_s - time.monotonic()))
            if isinstance(item, DamagedFrameError):
                return None
            if item is None:
                break
            response += item
            if expected is None:
                expected = response_size(response)
    except queue.Empty:
        bridge.note(f"{stream_id} {what}: no reply in {timeout_s} s")
        return None
    finally:
        bridge.close(stream_id)
    elapsed = time.monotonic() - started
    if elapsed > SLOW_S:
        bridge.note(f"{stream_id} {what}: took {elapsed:.1f} s")
    if not response:
        bridge.note(f"{stream_id} {what}: empty response")
        return None
    if expected is not None and len(response) < expected:
        bridge.note(f"{stream_id} {what}: cut short at {len(response)} of {expected} bytes")
        return None
    return response


def stream_events(bridge: Bridge, handler: BaseHTTPRequestHandler, request: bytes) -> None:
    """Passes the /events stream through until the browser leaves or a frame is lost."""
    stream_id, stream = bridge.open(request)
    try:
        while True:
            item = stream.get()
            if item is None or isinstance(item, DamagedFrameError):
                return
            handler.wfile.write(item)
            handler.wfile.flush()
    except (BrokenPipeError, ConnectionResetError, socket.timeout):
        pass
    finally:
        bridge.close(stream_id)


def relay(bridge: Bridge, handler: BaseHTTPRequestHandler) -> None:
    """Sends one browser request to the robot and writes back its response.

    A GET is retried when its response is damaged in transit. A POST is not, since the robot may
    already have acted on it; it gets a 502 instead.
    """
    request = build_request(handler)
    if handler.path == "/events":
        stream_events(bridge, handler, request)
        return
    what = f"{handler.command} {handler.path}"
    is_get = handler.command == "GET"
    tries = GET_TRIES if is_get else 1
    timeout_s = GET_TIMEOUT_S if is_get else RESPONSE_TIMEOUT_S
    for _ in range(tries):
        response = fetch(bridge, request, what, timeout_s)
        if response is not None:
            break
    try:
        if response is None:
            handler.send_error(502, "usb bridge: the response was damaged in transit")
        else:
            handler.wfile.write(response)
    except (BrokenPipeError, ConnectionResetError, socket.timeout):
        pass


def make_handler(bridge: Bridge) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, format: str, *args: object) -> None:  # noqa: A002
            pass

        def do_GET(self) -> None:  # noqa: N802
            relay(bridge, self)

        def do_POST(self) -> None:  # noqa: N802
            relay(bridge, self)

    return Handler


def find_port() -> str:
    ports = sorted(glob.glob("/dev/serial/by-id/*Espressif*")) or sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("no robot on USB: plug it in or pass --port")
    return ports[0]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port (default: the first Espressif device)")
    parser.add_argument("--listen", type=int, default=8080, help="localhost port (default 8080)")
    parser.add_argument("--log", help="also append the robot's log lines to this file")
    args = parser.parse_args()

    log = open(args.log, "ab") if args.log else None  # noqa: SIM115
    bridge = Bridge(args.port or find_port(), log)
    server = ThreadingHTTPServer(("127.0.0.1", args.listen), make_handler(bridge))
    server.daemon_threads = True
    print(f"robot dashboard on http://localhost:{args.listen}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
