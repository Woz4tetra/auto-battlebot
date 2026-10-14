"""Subscribe-only client for the Foxglove WebSocket protocol, as ``viz_relay`` serves it.

Yields the same ``(topic, log_time_ns, MessageBytes)`` tuples as ``mcap_io.iter_messages``, so
one consumer serves a live relay and a recording. The relay advertises channels with their
schemas; protobuf schemas are registered with ``mcap_io`` so ``decode_tf_message`` and friends
work on live payloads unchanged.

There is no publish path: the client never advertises a client channel and never sends a
message frame. It sends ``subscribe`` and ``unsubscribe`` and nothing else.

Wire format (foxglove.websocket.v1, which the SDK's ``foxglove.sdk.v1`` shares):

- server text frames: JSON ``{"op": "serverInfo" | "advertise" | "unadvertise" | ...}``
- client text frames: ``{"op": "subscribe", "subscriptions": [{"id", "channelId"}]}``
- server binary frames, opcode 0x01: ``u8 opcode, u32 subscription id, u64 log time ns``
  (little endian), then the payload.
"""

from __future__ import annotations

import base64
import json
import logging
import struct
import time
from dataclasses import dataclass
from typing import Any, Iterator

from mcap.records import Schema
from websockets.exceptions import ConnectionClosed
from websockets.sync.client import connect

from auto_battlebot.recording.mcap_io import MessageBytes, _register_proto_schema

SUBPROTOCOLS = ["foxglove.sdk.v1", "foxglove.websocket.v1"]
OP_MESSAGE_DATA = 0x01
_HEADER = struct.Struct("<BIQ")

log = logging.getLogger(__name__)


@dataclass
class _Channel:
    id: int
    topic: str
    encoding: str
    schema_name: str


class FoxgloveSubscriber:
    """Connects, subscribes to ``topics`` as they are advertised, and yields their messages.

    ``messages`` reconnects forever with a backoff; stop it by breaking out of the loop or
    calling ``close``.
    """

    def __init__(self, url: str, topics: list[str], open_timeout_s: float = 3.0) -> None:
        self.url = url
        self.topics = set(topics)
        self.open_timeout_s = open_timeout_s
        self.connected = False
        self._closed = False
        self._ws: Any = None

    def stopped(self) -> bool:
        return self._closed

    def close(self) -> None:
        self._closed = True
        if self._ws is not None:
            self._ws.close()

    def messages(self) -> Iterator[tuple[str, int, MessageBytes]]:
        backoff = 0.5
        while not self.stopped():
            try:
                with connect(
                    self.url,
                    subprotocols=SUBPROTOCOLS,  # type: ignore[arg-type]
                    max_size=None,
                    open_timeout=self.open_timeout_s,
                ) as ws:
                    self._ws = ws
                    self.connected = True
                    backoff = 0.5
                    yield from self._session(ws)
            except (OSError, ConnectionClosed, TimeoutError) as exc:
                log.warning("relay %s: %s", self.url, exc)
            finally:
                self.connected = False
                self._ws = None
            if self.stopped():
                return
            time.sleep(backoff)
            backoff = min(backoff * 2, 5.0)

    def _session(self, ws: Any) -> Iterator[tuple[str, int, MessageBytes]]:
        subs: dict[int, _Channel] = {}  # subscription id -> channel
        for frame in ws:
            if isinstance(frame, str):
                self._control(ws, json.loads(frame), subs)
                continue
            data = bytes(frame)
            if not data or data[0] != OP_MESSAGE_DATA or len(data) < _HEADER.size:
                continue
            _op, sub_id, log_time_ns = _HEADER.unpack_from(data)
            chan = subs.get(sub_id)
            if chan is None:
                continue
            payload = MessageBytes(
                data[_HEADER.size :], chan.encoding, chan.schema_name, chan.topic
            )
            yield chan.topic, int(log_time_ns), payload

    def _control(self, ws: Any, msg: dict[str, Any], subs: dict[int, _Channel]) -> None:
        """Subscribe to wanted channels as they are advertised; forget withdrawn ones."""
        op = msg.get("op")
        if op == "advertise":
            have = {c.id for c in subs.values()}
            wanted = []
            for ch in msg.get("channels", []):
                chan = self._advertised(ch)
                if chan.topic in self.topics and chan.id not in have:
                    sub_id = max(subs, default=0) + 1
                    subs[sub_id] = chan
                    have.add(chan.id)
                    wanted.append({"id": sub_id, "channelId": chan.id})
            if wanted:
                ws.send(json.dumps({"op": "subscribe", "subscriptions": wanted}))
        elif op == "unadvertise":
            gone = set(msg.get("channelIds", []))
            for sub_id in [s for s, c in subs.items() if c.id in gone]:
                del subs[sub_id]

    @staticmethod
    def _advertised(ch: dict[str, Any]) -> _Channel:
        encoding = str(ch.get("encoding", ""))
        schema_name = str(ch.get("schemaName", ""))
        if encoding == "protobuf" and ch.get("schema"):
            _register_proto_schema(
                Schema(
                    id=int(ch["id"]),
                    name=schema_name,
                    encoding="protobuf",
                    data=base64.b64decode(ch["schema"]),
                )
            )
        return _Channel(int(ch["id"]), str(ch["topic"]), encoding, schema_name)
