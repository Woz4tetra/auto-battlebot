"""SmallRig 2948 mini ball head (Amazon B09G9PFQCN, sold as the 2948B two-pack), cold shoe removed.

Reference model for the camera bracket assembly, not a part to make. SmallRig publishes no
drawing or CAD for it. What is known:

- Listing (smallrigreseller.com, 2948B): 49 x 33 x 25 mm overall with the cold shoe fitted,
  47 g, 1.5 kg load, 360° pan, 135° tilt.
- Top: male 1/4-20 stud on a round platform with a rubber pad. Bottom: female 1/4-20 once
  the cold shoe adapter is unscrewed, which is how it mounts here.
- Everything else is scaled from SmallRig's side-view product photo (2948b_4_.jpg, about
  24 px/mm, using the listed 25 mm base width): body Ø19 at the top flaring to Ø23 at the
  bottom, 27.5 mm tall; a drop notch on one side lets the neck fall to 90°; the wing knob is
  on the opposite side; the platform is about Ø26 x 5.

Coordinates: origin at the center of the bottom face, which seats on the mounting surface;
+Z up the body axis. The drop notch is on +X, the knob on -X.

Pose: `pan` turns the whole head about Z (it is free to spin on its stud). `tilt` swings the
neck about the ball center toward `tilt_dir` (degrees in the body frame, 0 = into the notch).
Into the notch it reaches 90°; any other direction stops at about 20°.
`top_location(p)` gives the platform's top-face center with +Z along the stud, which is
where a mounted part's bottom face sits.

Open questions
- Every dimension except the overall 49 x 33 x 25 is photo-scaled, about ±1 mm.
- Bottom thread depth is not published; 8 mm assumed. Measure it before choosing a stud
  length: the 1/2" set screw here puts 6.35 mm into it.
"""

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
    body_h: float = 27.5  # photo scale
    body_d_top: float = 19.0  # photo scale
    body_d_bot: float = 23.0  # photo scale, listing gives 25 across the cold shoe
    ball_z: float = 18.5  # ball center above the bottom face, photo scale
    ball_d: float = 14.0  # photo scale
    neck_d: float = 7.0  # photo scale
    platform_gap: float = 13.5  # ball center to platform underside, photo scale
    platform_d: float = 26.0  # photo scale, octagon across corners
    platform_t: float = 5.0  # photo scale, metal plus rubber pad
    stud_len: float = 5.5  # photo scale; 1/4-20 male
    stud_d: float = 6.35  # 1/4-20 major diameter
    tap_d: float = 5.1  # 1/4-20 tap drill, fasteners.md
    tap_depth: float = 8.0  # assumed
    notch_w: float = 7.6  # neck + clearance
    knob_z: float = 16.0  # photo scale
    knob_reach: float = 21.5  # from axis; 33 mm listed depth minus half the 23 mm body
    pan: float = 0.0
    tilt: float = 0.0
    tilt_dir: float = 0.0
    spin: float = 0.0  # platform about the stud


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
        * Rot(0, 0, p.spin)
    )


def top_location(p: Params) -> Location:
    _check_pose(p)
    return _pose(p) * Pos(0, 0, p.ball_z + p.platform_gap + p.platform_t)


def build(p: Params) -> Part:
    _check_pose(p)
    body = Cone(p.body_d_bot / 2, p.body_d_top / 2, p.body_h, align=C_MIN)
    # Socket opening and the drop notch toward +X, down to the neck's lowest position.
    body -= Pos(0, 0, p.ball_z) * Cylinder(p.neck_d / 2 + 1.5, p.body_h, align=C_MIN)
    notch_floor = p.ball_z - p.notch_w / 2
    body -= Pos(p.body_d_bot / 2, 0, notch_floor) * Box(
        p.body_d_bot, p.notch_w, p.body_h, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    body -= Pos(0, 0, p.ball_z) * Sphere(p.ball_d / 2)
    body -= Cylinder(p.tap_d / 2, p.tap_depth, align=C_MIN)
    # Wing knob on -X: hub along -X, then a flat wing.
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
    # Listing: 49 mm tall with the ~12 mm cold shoe, 33 mm deep across the knob.
    return {"bbox": (None, None, 42.5)}


if __name__ == "__main__":
    print(build(Params()).bounding_box())
    print(top_location(Params(tilt=90)))
