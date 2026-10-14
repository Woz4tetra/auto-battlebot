"""The sysid monitor's page: one static HTML file and a websocket of JSON snapshots.

Served by the Python process itself, on the laptop, because the page shows that process's
state, not the app's. ``GET /`` returns ``static/index.html``; ``/ws`` sends a ``hello`` with
the parameter list, then every snapshot ``publish`` hands it.
"""

from __future__ import annotations

import json
import threading
from http import HTTPStatus
from pathlib import Path
from typing import Any

from websockets.datastructures import Headers
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response
from websockets.sync.server import Server, ServerConnection, serve

STATIC = Path(__file__).resolve().parent / "static"


class PageServer:
    def __init__(self, host: str, port: int, hello: dict[str, Any]) -> None:
        self.hello = json.dumps({"type": "hello", **hello})
        self._cond = threading.Condition()
        self._snapshot: str | None = None
        self._version = 0
        self._server: Server = serve(
            self._handler, host, port, process_request=self._http, max_size=None
        )
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def publish(self, snapshot: dict[str, Any]) -> None:
        text = json.dumps({"type": "snapshot", "snapshot": snapshot}, allow_nan=False)
        with self._cond:
            self._snapshot = text
            self._version += 1
            self._cond.notify_all()

    def close(self) -> None:
        self._server.shutdown()

    def _http(self, connection: ServerConnection, request: Request) -> Response | None:
        if request.path == "/ws":
            return None
        if request.path in ("/", "/index.html"):
            body = (STATIC / "index.html").read_bytes()
            headers = Headers(
                [("Content-Type", "text/html; charset=utf-8"), ("Cache-Control", "no-store")]
            )
            return Response(HTTPStatus.OK.value, "OK", headers, body)
        return connection.respond(HTTPStatus.NOT_FOUND, "not found\n")

    def _handler(self, ws: ServerConnection) -> None:
        seen = -1
        try:
            ws.send(self.hello)
            while True:
                with self._cond:
                    self._cond.wait_for(lambda: self._version != seen, timeout=1.0)
                    text, seen = self._snapshot, self._version
                if text is not None:
                    ws.send(text)
        except ConnectionClosed:
            return
