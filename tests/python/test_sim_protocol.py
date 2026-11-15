import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation"))

import protocol  # noqa: E402


def test_request_round_trip() -> None:
    request = protocol.unpack_request(struct.pack("<ddd", 0.5, 0.1, -0.25))
    assert (request.linear_x, request.linear_y, request.angular_z) == (0.5, 0.1, -0.25)


def test_response_header_matches_wire_size() -> None:
    header = protocol.ResponseHeader(640, 480, tuple(float(i) for i in range(16)), 1, 2, 3, 4, 5.5)
    packed = protocol.pack_response_header(header)
    assert len(packed) == protocol.RESPONSE_HEADER_SIZE
    assert struct.unpack(protocol.RESPONSE_HEADER_FMT, packed)[-1] == 5.5


def test_meas_pose_matches_wire_size() -> None:
    packed = protocol.pack_meas_pose(protocol.MeasPose(1.0, 0.2, 0.3, 0.4))
    assert len(packed) == protocol.MEAS_POSE_SIZE
