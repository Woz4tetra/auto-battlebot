"""Binary protocol shared between the Genesis server and the C++ client.

Wire format must stay in sync with include/simulation/sim_connection.hpp.
"""

from __future__ import annotations

import socket
import struct

REQUEST_FMT: str = "<ddd"  # linear_x, linear_y, angular_z (3 x float64 = 24 bytes)
REQUEST_SIZE: int = struct.calcsize(REQUEST_FMT)

RESPONSE_HEADER_FMT: str = "<II16d4d"  # width, height, tf_matrix[16], fx/fy/cx/cy
RESPONSE_HEADER_SIZE: int = struct.calcsize(RESPONSE_HEADER_FMT)

# Ground truth poses appended after depth buffer:
#   uint32 num_robots, then num_robots * (x, y, yaw) as float64.
#   First entry = our robot, rest = opponents in config order.
GT_POSE_FMT: str = "<ddd"  # x, y, yaw per robot (3 x float64 = 24 bytes)
GT_POSE_SIZE: int = struct.calcsize(GT_POSE_FMT)
GT_COUNT_FMT: str = "<I"  # uint32 num_robots
GT_COUNT_SIZE: int = struct.calcsize(GT_COUNT_FMT)

_SOCK_BUF_SIZE: int = 4 * 1024 * 1024  # 4 MiB


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
