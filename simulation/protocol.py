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


def recv_all(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("client disconnected")
        buf.extend(chunk)
    return bytes(buf)


def send_all(sock: socket.socket, data: bytes) -> None:
    sock.sendall(data)
