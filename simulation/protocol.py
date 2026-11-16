"""Binary protocol shared between the Genesis server and the C++ client."""

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


def unpack_request(data: bytes) -> Request:
    return Request._make(struct.unpack(REQUEST_FMT, data))


def pack_response_header(header: ResponseHeader) -> bytes:
    return struct.pack(
        RESPONSE_HEADER_FMT,
        header.width,
        header.height,
        *header.tf_matrix,
        header.fx,
        header.fy,
        header.cx,
        header.cy,
        header.sim_time,
    )


def pack_meas_pose(pose: MeasPose) -> bytes:
    return struct.pack(MEAS_POSE_FMT, *pose)


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
