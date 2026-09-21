"""Camera mounts on the NHRL cage: where a camera can plausibly sit, and the pose it sees.

Pure module (no Blender). `cage_scene.py` turns the poses returned here into BlenderProc
camera poses; tests check the geometry without a renderer.

A mount is five numbers and a wall, not a free pose: the camera is clamped high on one of the
four walls looking down and across the mat. `playground/cage_scene/fit_cage_camera.py` fits
NHRL's own `Cage-2-Overhead-High` phone that way across eleven clips and lands at 1.16 m above
the mat, 0.11 m inside the wall plane, 32 deg off straight down, within 1.7 deg of square to
the wall. The default ranges below bracket those fits, so a sampled mount is the same kind of
mount with mounting slop instead of one fitted pose.

Poses come out as the cam2world matrix `bproc.camera.add_camera_pose` wants, in the W frame
`auto_battlebot/perception/cage_calibration.py` documents: mat centre at the origin, z up, the
mat top surface at z = 0. The Blender camera looks down its own -z with +y up, which is the
one convention this module encodes by hand; `tests/test_cage_mount.py` pins it against
`cage_calibration`'s own helpers so the two cannot drift.
"""

from __future__ import annotations

import math
import random
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

# The wall a camera is mounted on, named from the fitted camera's point of view: it sits on
# the near wall, so +y in the W frame runs away from it.
WALLS: tuple[str, ...] = ("near", "far", "left", "right")

_UP = np.array([0.0, 0.0, 1.0])
_DOWN = np.array([0.0, 0.0, -1.0])
# Above this |cos| between the optical axis and world z, the axis is too close to straight
# down for world up to orient the image, and the inward heading is used instead.
_DEGENERATE_COS = 0.999


@dataclass(frozen=True)
class CageMount:
    """One camera mount: a wall, where on it, and where it points.

    `along_m` runs along the wall from its centre, positive toward the wall's own +along axis
    (see `wall_axes`). `inset_m` is how far inside the wall plane the lens sits, so positive
    is toward the mat centre. `tilt_deg` is the optical axis off straight down, `yaw_deg` its
    heading away from square-to-the-wall, and `roll_deg` the camera's roll about that axis.
    """

    wall: str
    along_m: float
    height_m: float
    inset_m: float
    tilt_deg: float
    yaw_deg: float
    roll_deg: float


@dataclass(frozen=True)
class CageMountRanges:
    """Uniform sampling ranges for a `CageMount`, bracketing the fitted NHRL mounts."""

    walls: tuple[str, ...] = WALLS
    along_m: tuple[float, float] = (-0.45, 0.45)
    height_m: tuple[float, float] = (1.00, 1.45)
    # Distance inside the wall plane. Negative puts the mount outside the cage, which is
    # allowed: a pane the camera looks through from outside is hidden for that frame, so it
    # costs no labels (see `panels_outside_camera`).
    inset_m: tuple[float, float] = (0.02, 0.25)
    # "fixed" samples `tilt_deg` directly. "centre" derives tilt so the optical axis meets the
    # field centre and samples `tilt_offset_deg` on top, which keeps tilt tied to how far out
    # and how high the mount landed instead of drawing the three independently.
    aim: str = "fixed"
    tilt_deg: tuple[float, float] = (26.0, 42.0)
    tilt_offset_deg: tuple[float, float] = (0.0, 0.0)
    yaw_deg: tuple[float, float] = (-9.0, 9.0)
    roll_deg: tuple[float, float] = (-3.0, 3.0)


def wall_axes(wall: str) -> tuple[np.ndarray, np.ndarray]:
    """(inward normal, along axis) of *wall* in the W frame, both unit vectors.

    The along axis is the inward normal turned -90 deg about z, so for the near wall (normal
    +y) it is +x: positive `along_m` moves right as that camera sees it.

    Raises:
        ValueError: When *wall* is not one of `WALLS`.
    """
    if wall not in WALLS:
        raise ValueError(f"unknown cage wall {wall!r}; valid walls are {list(WALLS)}")
    normal = {
        "near": np.array([0.0, 1.0, 0.0]),
        "far": np.array([0.0, -1.0, 0.0]),
        "left": np.array([1.0, 0.0, 0.0]),
        "right": np.array([-1.0, 0.0, 0.0]),
    }[wall]
    along = np.array([normal[1], -normal[0], 0.0])
    return normal, along


def aim_tilt_deg(height_m: float, distance_m: float) -> float:
    """Tilt off straight down whose optical axis meets the field centre."""
    return math.degrees(math.atan2(distance_m, height_m))


def sample_cage_mount(ranges: CageMountRanges, wall_half_m: float) -> CageMount:
    """Draw one mount, uniform in every range and in the wall choice.

    `wall_half_m` is half the cage interior, needed only when `aim` is "centre": the tilt then
    follows from how far the drawn mount ended up from the field centre.
    """
    if ranges.aim not in ("fixed", "centre"):
        raise ValueError(f"mount aim is {ranges.aim!r}; use 'fixed' or 'centre'")
    along = random.uniform(*ranges.along_m)
    height = random.uniform(*ranges.height_m)
    inset = random.uniform(*ranges.inset_m)
    if ranges.aim == "centre":
        distance = math.hypot(wall_half_m - inset, along)
        tilt = aim_tilt_deg(height, distance) + random.uniform(*ranges.tilt_offset_deg)
    else:
        tilt = random.uniform(*ranges.tilt_deg)
    return CageMount(
        wall=random.choice(list(ranges.walls)),
        along_m=along,
        height_m=height,
        inset_m=inset,
        tilt_deg=tilt,
        yaw_deg=random.uniform(*ranges.yaw_deg),
        roll_deg=random.uniform(*ranges.roll_deg),
    )


def mount_position(mount: CageMount, wall_half_m: float) -> np.ndarray:
    """Lens position in the W frame. *wall_half_m* is half the cage interior span."""
    normal, along = wall_axes(mount.wall)
    horizontal = -normal * (wall_half_m - mount.inset_m) + along * mount.along_m
    return np.asarray(horizontal + _UP * mount.height_m)


def mount_corners(ranges: CageMountRanges, wall_half_m: float) -> list[tuple[str, np.ndarray]]:
    """Horizontal extremes of every mount *ranges* can draw, as (wall, xy) pairs.

    A mount's position is linear in `along_m` and `inset_m`, so the four corners of that
    rectangle bound every draw on a wall.
    """
    corners = []
    for wall in ranges.walls:
        for along in ranges.along_m:
            for inset in ranges.inset_m:
                mount = CageMount(wall, along, 0.0, inset, 0.0, 0.0, 0.0)
                corners.append((wall, mount_position(mount, wall_half_m)[:2]))
    return corners


def mounts_behind_walls(
    ranges: CageMountRanges,
    wall_half_m: float,
    venue_walls: Sequence[tuple[str, tuple[float, float], tuple[float, float]]],
    margin_m: float = 0.10,
) -> list[str]:
    """One message per mount corner that a venue wall hides the mat from, empty when none does.

    *venue_walls* are (name, start, end) segments in W, as `[[walls]]` in a cage spec gives
    them. A wall's box side is the side the mat centre is on. A corner is blocked when it is
    within *margin_m* of the wall's plane or past it, and abreast of the segment: a camera
    beyond a wall's end sees round it and is left alone.
    """
    blocked = []
    for name, start, end in venue_walls:
        a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        length = float(np.linalg.norm(b - a))
        direction = (b - a) / length
        normal = np.array([-direction[1], direction[0]])
        if float(np.dot(-a, normal)) < 0.0:
            normal = -normal
        for wall, corner in mount_corners(ranges, wall_half_m):
            clearance = float(np.dot(corner - a, normal))
            abreast = -margin_m <= float(np.dot(corner - a, direction)) <= length + margin_m
            if abreast and clearance < margin_m:
                blocked.append(
                    f"a {wall!r} mount at ({corner[0]:.2f}, {corner[1]:.2f}) m is {clearance:.2f} m"
                    f" in front of venue wall {name!r} (need {margin_m:.2f} m)"
                )
    return blocked


def mount_heading(mount: CageMount) -> np.ndarray:
    """Unit horizontal direction the mount faces: the wall's inward normal turned by its yaw."""
    normal, along = wall_axes(mount.wall)
    yaw = math.radians(mount.yaw_deg)
    return np.asarray(normal * math.cos(yaw) + along * math.sin(yaw))


def mount_forward(mount: CageMount) -> np.ndarray:
    """Unit optical axis in the W frame: tilted off straight down, yawed off the wall normal."""
    tilt = math.radians(mount.tilt_deg)
    return np.asarray(_unit(mount_heading(mount) * math.sin(tilt) + _DOWN * math.cos(tilt)))


def camera_basis_cam2world(
    position: np.ndarray, forward: np.ndarray, heading: np.ndarray, roll_rad: float
) -> np.ndarray:
    """A 4x4 cam2world for a Blender camera (-z forward, +y up) at *position* facing *forward*.

    *heading* is the horizontal direction the image's up axis falls back to when the optical axis
    is within `_DEGENERATE_COS` of vertical, where world up cannot define the image axes. A cage
    camera looking near-straight-down hits that case, so the fallback is not academic.

    `synthgen.freefly` calls this too, which is what keeps a flown pose and a `CageMount` pose on
    one convention.
    """
    reference = heading if abs(float(np.dot(_UP, forward))) > _DEGENERATE_COS else _UP
    up = _unit(reference - float(np.dot(reference, forward)) * forward)
    right = _unit(np.cross(up, -forward))
    right, up = _roll(right, up, forward, roll_rad)

    cam2world = np.eye(4)
    cam2world[:3, 0] = right
    cam2world[:3, 1] = up
    cam2world[:3, 2] = -forward
    cam2world[:3, 3] = position
    return cam2world


def mount_cam2world(mount: CageMount, wall_half_m: float) -> np.ndarray:
    """The 4x4 cam2world matrix for *mount* (Blender camera: -z forward, +y up)."""
    return camera_basis_cam2world(
        mount_position(mount, wall_half_m),
        mount_forward(mount),
        mount_heading(mount),
        math.radians(mount.roll_deg),
    )


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("cannot normalize a zero-length vector")
    return np.asarray(vector / norm)


def _roll(
    right: np.ndarray, up: np.ndarray, axis: np.ndarray, angle: float
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate the image axes about the optical axis. Both inputs are perpendicular to it."""
    if angle == 0.0:
        return right, up
    cos, sin = math.cos(angle), math.sin(angle)
    return (
        _unit(right * cos + np.cross(axis, right) * sin),
        _unit(up * cos + np.cross(axis, up) * sin),
    )
