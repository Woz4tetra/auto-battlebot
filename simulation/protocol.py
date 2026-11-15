"""Binary protocol shared between the Genesis server and the C++ client.

Wire format must stay in sync with include/simulation/sim_connection.hpp.
"""

from __future__ import annotations

import socket
import struct
from typing import NamedTuple


class Request(NamedTuple):
    linear_x: float
    linear_y: float
    angular_z: float


class ResponseHeader(NamedTuple):
    width: int
    height: int
    tf_matrix: tuple[float, ...]
    fx: float
    fy: float
    cx: float
    cy: float
    sim_time: float


class MeasPose(NamedTuple):
    measured: float
    x: float
    y: float
    yaw: float


REQUEST_FMT: str = "<ddd"
REQUEST_SIZE: int = struct.calcsize(REQUEST_FMT)

RESPONSE_HEADER_FMT: str = "<II16d4dd"
RESPONSE_HEADER_SIZE: int = struct.calcsize(RESPONSE_HEADER_FMT)

GT_POSE_FMT: str = "<ddd"
GT_POSE_SIZE: int = struct.calcsize(GT_POSE_FMT)
GT_COUNT_FMT: str = "<I"
GT_COUNT_SIZE: int = struct.calcsize(GT_COUNT_FMT)

MEAS_HEADER_FMT: str = "<dd"
MEAS_HEADER_SIZE: int = struct.calcsize(MEAS_HEADER_FMT)
MEAS_POSE_FMT: str = "<dddd"
MEAS_POSE_SIZE: int = struct.calcsize(MEAS_POSE_FMT)

_SOCK_BUF_SIZE: int = 4 * 1024 * 1024


def configure_socket(sock: socket.socket) -> None:
    """Apply low-latency socket options (TCP_NODELAY, large buffers)."""
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, _SOCK_BUF_SIZE)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, _SOCK_BUF_SIZE)


def recv_all(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("client disconnected")
        buf.extend(chunk)
    return bytes(buf)


def send_all(sock: socket.socket, data: bytes | bytearray | memoryview) -> None:
    sock.sendall(data)
