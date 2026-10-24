"""4-way transport over the Mk2 robot's ESC tuning mode (firmware/mr_stabs_mk2, /esc/ routes).

HttpPort satisfies fourway.SerialPort, so esc.Session runs over WiFi unchanged: write() posts one
4-way request frame and waits for the robot to relay it, and read() serves the response bytes.
Pass msp=False to Session; the relay speaks 4-way from the start.

    POST /esc/tuning  enter=1|0      enter or leave tuning mode (409 while armed)
    GET  /esc/status                 {"state": "combat"|"tuning"|"exiting", "error": ...}
    POST /esc/4way    frame=<hex>    queue one request frame -> {"id": n}
    GET  /esc/4way                   {"id": n, "state": "none"|"pending"|"done", "frame": <hex>}
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any


class RelayError(RuntimeError):
    pass


class HttpPort:
    def __init__(self, url: str, timeout_s: float = 15.0, poll_s: float = 0.03) -> None:
        self._base = url.rstrip("/")
        self._timeout_s = timeout_s
        self._poll_s = poll_s
        self._tx = bytearray()
        self._rx = bytearray()

    def _call(self, path: str, fields: dict[str, str] | None = None) -> str:
        data = urllib.parse.urlencode(fields).encode() if fields is not None else None
        req = urllib.request.Request(self._base + path, data=data)
        try:
            with urllib.request.urlopen(req, timeout=10.0) as resp:
                body: bytes = resp.read()
        except urllib.error.HTTPError as e:
            raise RelayError(f"{path}: HTTP {e.code}: {e.read().decode(errors='replace')}") from e
        except urllib.error.URLError as e:
            raise RelayError(f"{path}: {e.reason}") from e
        return body.decode()

    def status(self) -> dict[str, Any]:
        result: dict[str, Any] = json.loads(self._call("/esc/status"))
        return result

    def _wait_for(self, state: str, timeout_s: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while True:
            status = self.status()
            if status["state"] == state:
                return status
            if time.monotonic() > deadline:
                reason = f": {status['error']}" if status.get("error") else ""
                raise RelayError(f"robot stayed in {status['state']}, not {state}{reason}")
            time.sleep(0.1)

    def enter_tuning(self) -> bool:
        """Enter tuning mode. False if the robot was already in it."""
        if self.status()["state"] == "tuning":
            return False
        self._call("/esc/tuning", {"enter": "1"})
        # The robot holds entry for up to 3 s after leaving tuning mode, while the ESCs restart.
        self._wait_for("tuning", 8.0)
        return True

    def exit_tuning(self) -> None:
        """Leave tuning mode and wait out the 2 s ESC wake-up."""
        self._call("/esc/tuning", {"enter": "0"})
        self._wait_for("combat", 10.0)

    def _relay(self, frame: bytes) -> bytes:
        frame_id = json.loads(self._call("/esc/4way", {"frame": frame.hex()}))["id"]
        deadline = time.monotonic() + self._timeout_s
        while time.monotonic() < deadline:
            time.sleep(self._poll_s)
            poll = json.loads(self._call("/esc/4way"))
            if poll["id"] != frame_id:
                raise RelayError(f"frame {frame_id} was replaced by {poll['id']}")
            if poll["state"] == "done":
                return bytes.fromhex(poll["frame"])
            if poll["state"] == "none":
                raise RelayError(f"frame {frame_id} was dropped (tuning mode left?)")
        raise RelayError(f"frame {frame_id}: no response in {self._timeout_s:.0f} s")

    def write(self, data: bytes) -> int:
        self._tx += data
        while len(self._tx) >= 5:
            size = 5 + (self._tx[4] or 256) + 2
            if len(self._tx) < size:
                break
            frame, self._tx = bytes(self._tx[:size]), self._tx[size:]
            self._rx += self._relay(frame)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        out, self._rx = bytes(self._rx[:size]), self._rx[size:]
        return out

    def reset_input_buffer(self) -> None:
        self._rx.clear()
