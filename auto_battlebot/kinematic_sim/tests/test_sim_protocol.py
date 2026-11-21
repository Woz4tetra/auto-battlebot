import re
import struct

from auto_battlebot.kinematic_sim import protocol
from auto_battlebot.paths import REPO_ROOT


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


def _cpp_struct_doubles(source: str, name: str) -> int:
    body = re.search(rf"struct {name} \{{(.*?)\}};", source, re.S).group(1)
    total = 0
    for line in body.split(";"):
        line = line.split("//")[0].strip()
        match = re.match(r"(uint32_t|double)\s+(.*)", line)
        if not match:
            continue
        for declarator in match.group(2).split(","):
            count = re.search(r"\[(\d+)\]", declarator)
            size = 4 if match.group(1) == "uint32_t" else 8
            total += size * (int(count.group(1)) if count else 1)
    return total


def test_formats_match_cpp_header() -> None:
    source = (REPO_ROOT / "src" / "simulation" / "sim_connection.cpp").read_text()
    assert protocol.REQUEST_SIZE == _cpp_struct_doubles(source, "SimRequest")
    assert protocol.RESPONSE_HEADER_SIZE == _cpp_struct_doubles(source, "SimResponseHeader")
    assert protocol.GT_POSE_SIZE == 3 * 8
    assert protocol.MEAS_HEADER_SIZE == 2 * 8
    assert protocol.MEAS_POSE_SIZE == 4 * 8
    assert "double x, y, yaw;" in source
    assert "double delivered, capture_time;" in source
    assert "double measured, x, y, yaw;" in source
