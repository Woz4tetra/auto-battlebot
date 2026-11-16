"""SmallRig 2948 mini ball head reference model."""

from dataclasses import dataclass

from build123d import (
    Align,
    Box,
    Cone,
    Cylinder,
    Location,
    Part,
    Pos,
    RegularPolygon,
    Rot,
    Sphere,
    extrude,
)

PROCESS = "none"
MATERIAL = "al6061"
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class Params:
    body_h: float = 27.5
    body_d_top: float = 19.0
    body_d_bot: float = 23.0
    ball_z: float = 18.5
    ball_d: float = 14.0
    neck_d: float = 7.0
    platform_gap: float = 13.5
    platform_d: float = 26.0
    platform_t: float = 5.0
    stud_len: float = 5.5
    stud_d: float = 6.35
    tap_d: float = 5.1
    tap_depth: float = 8.0
    notch_w: float = 7.6
    knob_z: float = 16.0
    knob_reach: float = 21.5
    pan: float = 0.0
    tilt: float = 0.0
    tilt_dir: float = 0.0
    platform_spin_deg: float = 0.0


def _check_pose(p: Params) -> None:
    into_notch = abs(((p.tilt_dir + 180) % 360) - 180) < 1e-6
    limit = 90.0 if into_notch else 20.0
    assert 0 <= p.tilt <= limit, f"tilt {p.tilt}° toward {p.tilt_dir}° exceeds {limit}°"


def _moving(p: Params) -> Part:
    """Ball, neck, platform and stud, upright, in body coordinates."""
    top = Pos(0, 0, p.ball_z) * Sphere(p.ball_d / 2)
    top += Pos(0, 0, p.ball_z) * Cylinder(p.neck_d / 2, p.platform_gap, align=C_MIN)
    plat_z = p.ball_z + p.platform_gap
    top += Pos(0, 0, plat_z) * extrude(RegularPolygon(p.platform_d / 2, 8), p.platform_t)
    top += Pos(0, 0, plat_z + p.platform_t) * Cylinder(p.stud_d / 2, p.stud_len, align=C_MIN)
    return top


def _pose(p: Params) -> Location:
    """Body frame -> tilted top frame: spin about the stud, tilt about the ball, then pan."""
    c = p.ball_z
    return (
        Rot(0, 0, p.pan)
        * Pos(0, 0, c)
        * Rot(0, 0, p.tilt_dir)
        * Rot(0, p.tilt, 0)
        * Rot(0, 0, -p.tilt_dir)
        * Pos(0, 0, -c)
        * Rot(0, 0, p.platform_spin_deg)
    )


def top_location(p: Params) -> Location:
    _check_pose(p)
    return _pose(p) * Pos(0, 0, p.ball_z + p.platform_gap + p.platform_t)


def build(p: Params) -> Part:
    _check_pose(p)
    body = Cone(p.body_d_bot / 2, p.body_d_top / 2, p.body_h, align=C_MIN)
    body -= Pos(0, 0, p.ball_z) * Cylinder(p.neck_d / 2 + 1.5, p.body_h, align=C_MIN)
    notch_floor = p.ball_z - p.notch_w / 2
    body -= Pos(p.body_d_bot / 2, 0, notch_floor) * Box(
        p.body_d_bot, p.notch_w, p.body_h, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    body -= Pos(0, 0, p.ball_z) * Sphere(p.ball_d / 2)
    body -= Cylinder(p.tap_d / 2, p.tap_depth, align=C_MIN)
    hub_len = p.knob_reach - 3.0 - p.body_d_bot / 2 + 2.0
    body += (
        Pos(-(p.body_d_bot / 2 - 2.0), 0, p.knob_z)
        * Rot(0, -90, 0)
        * Cylinder(3.5, hub_len, align=C_MIN)
    )
    body += Pos(-p.knob_reach, 0, p.knob_z) * Box(
        3.0, 12.0, 16.0, align=(Align.MIN, Align.CENTER, Align.CENTER)
    )
    head = Rot(0, 0, p.pan) * body
    return head + _pose(p) * _moving(p)


def sections(p: Params) -> list[str]:
    return ["y=0"]


def expect(p: Params) -> dict:
    return {"bbox": (None, None, 42.5)}


if __name__ == "__main__":
    print(build(Params()).bounding_box())
    print(top_location(Params(tilt=90)))
