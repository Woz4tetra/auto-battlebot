"""Free-flight camera poses, and their projection back onto a `CageMount`.

Pure module (no Blender). `pose_camera_server.py` flies a camera around the cage with WASD and a
mouse; `synthgen.cage_mount` describes the same camera as a wall, a place on it and an aim. This
module is the bridge: `freefly_cam2world` builds the flown pose and `mount_from_cam2world` reads
the equivalent `CageMount` straight back out.

That read-back is exact, not a projection. `(along, height, inset)` spans position and
`(tilt, yaw, roll)` spans orientation, so every pose a camera can fly to is some mount, including
ones outside the cage. What a flown pose *can* fall outside is the sampling ranges the batch render
will draw from, so `clamp_mount_to_ranges` is what pulls it back and `mount_residual` measures how
far it had to move.

Both directions go through `cage_mount.camera_basis_cam2world`, so a flown pose and a sampled mount
can never drift onto different camera conventions.

Angles follow `auto_battlebot.perception.cage_calibration.pose_summary`: `yaw_deg` is the heading of
the optical axis projected on the floor, 0 along +y and positive toward +x. `pitch_deg` is its
elevation above horizontal, so -90 looks straight down and a cage camera flies at roughly -58.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from synthgen.cage_mount import (
    WALLS,
    CageMount,
    CageMountRanges,
    camera_basis_cam2world,
    mount_cam2world,
    wall_axes,
)

# Straight up or straight down leaves yaw and roll expressing the same rotation, so the client
# never gets to send either.
PITCH_LIMIT_DEG = 89.9
# Matches `cage_mount._DEGENERATE_COS`: within this of vertical, heading no longer follows from the
# optical axis and has to come from the camera's own up axis instead.
_DEGENERATE_COS = 0.999


@dataclass(frozen=True)
class FreeflyPose:
    """A camera anywhere in the W frame: mat center at the origin, z up, mat surface at z = 0."""

    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float
    pitch_deg: float
    roll_deg: float

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x_m, self.y_m, self.z_m])


@dataclass(frozen=True)
class MountResidual:
    """How far a flown pose sits from some other mount's pose."""

    position_m: float
    angle_deg: float


def clamp_pose(pose: FreeflyPose) -> FreeflyPose:
    """Clamp pitch off both poles and fold yaw and roll into (-180, 180]."""
    return FreeflyPose(
        x_m=float(pose.x_m),
        y_m=float(pose.y_m),
        z_m=float(pose.z_m),
        yaw_deg=_wrap_deg(pose.yaw_deg),
        pitch_deg=float(min(PITCH_LIMIT_DEG, max(-PITCH_LIMIT_DEG, pose.pitch_deg))),
        roll_deg=_wrap_deg(pose.roll_deg),
    )


def freefly_heading(yaw_deg: float) -> np.ndarray:
    """Unit horizontal direction for *yaw_deg*: 0 is +y, positive turns toward +x."""
    yaw = math.radians(yaw_deg)
    return np.array([math.sin(yaw), math.cos(yaw), 0.0])


def freefly_forward(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    """Unit optical axis for a heading and an elevation above horizontal."""
    pitch = math.radians(pitch_deg)
    heading = freefly_heading(yaw_deg)
    return np.asarray(heading * math.cos(pitch) + np.array([0.0, 0.0, math.sin(pitch)]))


def freefly_cam2world(pose: FreeflyPose) -> np.ndarray:
    """The 4x4 cam2world matrix for *pose* (Blender camera: -z forward, +y up)."""
    return camera_basis_cam2world(
        pose.position,
        freefly_forward(pose.yaw_deg, pose.pitch_deg),
        freefly_heading(pose.yaw_deg),
        math.radians(pose.roll_deg),
    )


def freefly_from_cam2world(cam2world: np.ndarray) -> FreeflyPose:
    """Recover a `FreeflyPose` from a cam2world matrix. Exact inverse of `freefly_cam2world`."""
    matrix = np.asarray(cam2world, dtype=np.float64)
    position = matrix[:3, 3]
    forward = -matrix[:3, 2]
    up = matrix[:3, 1]

    if abs(float(forward[2])) > _DEGENERATE_COS:
        # Looking near-straight-down: the optical axis carries no heading, so take it from the
        # image's own up axis and call the remaining rotation yaw rather than roll.
        heading = _unit_horizontal(up)
        yaw_deg = math.degrees(math.atan2(heading[0], heading[1]))
    else:
        yaw_deg = math.degrees(math.atan2(float(forward[0]), float(forward[1])))
    pitch_deg = math.degrees(math.asin(float(np.clip(forward[2], -1.0, 1.0))))

    zero_roll = camera_basis_cam2world(position, forward, freefly_heading(yaw_deg), 0.0)
    roll_deg = _roll_between(zero_roll[:3, 1], up, forward)
    return FreeflyPose(
        x_m=float(position[0]),
        y_m=float(position[1]),
        z_m=float(position[2]),
        yaw_deg=yaw_deg,
        pitch_deg=pitch_deg,
        roll_deg=roll_deg,
    )


def mount_from_cam2world(
    cam2world: np.ndarray, wall_half_m: float, walls: tuple[str, ...] = WALLS
) -> CageMount:
    """The `CageMount` that describes *cam2world* exactly, picking among *walls*.

    Every pose is some mount, so nothing is lost here. The wall choice only decides which numbers
    say so: pick a different wall and `along_m` and `inset_m` change to match.
    """
    matrix = np.asarray(cam2world, dtype=np.float64)
    position = matrix[:3, 3]
    forward = -matrix[:3, 2]
    horizontal = np.array([position[0], position[1], 0.0])

    # The camera's own wall is the one whose inward normal points away from the lens, so the
    # smallest normal-dot-position wins. Getting this sign backwards puts every mount on the
    # opposite wall, and it costs a whole render run to notice.
    wall = min(walls, key=lambda name: float(np.dot(wall_axes(name)[0], horizontal)))
    normal, along = wall_axes(wall)

    tilt_deg = math.degrees(math.acos(float(np.clip(-forward[2], -1.0, 1.0))))
    if abs(float(forward[2])) > _DEGENERATE_COS:
        heading = _unit_horizontal(matrix[:3, 1])
    else:
        heading = _unit_horizontal(forward)
    yaw_deg = math.degrees(
        math.atan2(float(np.dot(along, heading)), float(np.dot(normal, heading)))
    )

    mount = CageMount(
        wall=wall,
        along_m=float(np.dot(along, horizontal)),
        height_m=float(position[2]),
        inset_m=float(wall_half_m + np.dot(normal, horizontal)),
        tilt_deg=tilt_deg,
        yaw_deg=yaw_deg,
        roll_deg=0.0,
    )
    zero_roll = mount_cam2world(mount, wall_half_m)
    return replace(mount, roll_deg=_roll_between(zero_roll[:3, 1], matrix[:3, 1], forward))


def mount_residual(cam2world: np.ndarray, mount: CageMount, wall_half_m: float) -> MountResidual:
    """Meters and degrees between a flown pose and *mount*'s pose.

    Against `mount_from_cam2world`'s own answer this reads zero and is a live self-check that the
    two directions have not drifted. Against a clamped mount it is the real number: how far outside
    the sampling ranges the camera has been flown.
    """
    flown = np.asarray(cam2world, dtype=np.float64)
    projected = mount_cam2world(mount, wall_half_m)
    position_m = float(np.linalg.norm(flown[:3, 3] - projected[:3, 3]))
    # Angle of the rotation that takes one camera basis to the other.
    relative = projected[:3, :3].T @ flown[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return MountResidual(position_m=position_m, angle_deg=math.degrees(math.acos(cosine)))


def clamp_mount_to_ranges(mount: CageMount, ranges: CageMountRanges) -> CageMount:
    """The nearest mount the batch sampler could actually draw from *ranges*.

    Every scalar is clamped into its range. The wall is kept if `ranges.walls` allows it; otherwise
    the caller should re-read the mount with `walls=ranges.walls` first, since moving a camera to
    another wall renames `along_m` and `inset_m` rather than clamping them.
    """
    return CageMount(
        wall=mount.wall if mount.wall in ranges.walls else ranges.walls[0],
        along_m=_clamp(mount.along_m, ranges.along_m),
        height_m=_clamp(mount.height_m, ranges.height_m),
        inset_m=_clamp(mount.inset_m, ranges.inset_m),
        tilt_deg=_clamp(mount.tilt_deg, ranges.tilt_deg),
        yaw_deg=_clamp(mount.yaw_deg, ranges.yaw_deg),
        roll_deg=_clamp(mount.roll_deg, ranges.roll_deg),
    )


def ranges_covering(
    mounts: list[CageMount], pad_m: float = 0.0, pad_deg: float = 0.0
) -> CageMountRanges:
    """The smallest `CageMountRanges` containing every mount in *mounts*, widened by the pads.

    Lengths widen by *pad_m* on each side and angles by *pad_deg*, so one number cannot be applied
    to two different units by accident.
    """
    if not mounts:
        raise ValueError("no mounts marked; fly to a pose and mark it before saving ranges")
    walls = tuple(wall for wall in WALLS if any(mount.wall == wall for mount in mounts))
    return CageMountRanges(
        walls=walls,
        aim="fixed",
        along_m=_span([m.along_m for m in mounts], pad_m),
        height_m=_span([m.height_m for m in mounts], pad_m),
        inset_m=_span([m.inset_m for m in mounts], pad_m),
        tilt_deg=_span([m.tilt_deg for m in mounts], pad_deg),
        tilt_offset_deg=(0.0, 0.0),
        yaw_deg=_span([m.yaw_deg for m in mounts], pad_deg),
        roll_deg=_span([m.roll_deg for m in mounts], pad_deg),
    )


def format_mount_ranges_toml(ranges: CageMountRanges, header: str = "") -> str:
    """The `[cages.mount]` block, ready to paste under a `[[cages]]` entry in `config.toml`."""
    lines = [f"# {line}" if line else "#" for line in header.splitlines()]
    lines.append("[cages.mount]")
    lines.append("walls = [" + ", ".join(f'"{wall}"' for wall in ranges.walls) + "]")
    lines.append(f'aim = "{ranges.aim}"')
    for field in ("along_m", "height_m", "inset_m", "tilt_deg", "yaw_deg", "roll_deg"):
        low, high = getattr(ranges, field)
        lines.append(f"{field} = [{low:.4f}, {high:.4f}]")
    return "\n".join(lines) + "\n"


def _clamp(value: float, bounds: tuple[float, float]) -> float:
    low, high = min(bounds), max(bounds)
    return float(min(high, max(low, value)))


def _span(values: list[float], pad: float) -> tuple[float, float]:
    return (float(min(values)) - pad, float(max(values)) + pad)


def _unit_horizontal(vector: np.ndarray) -> np.ndarray:
    flat = np.array([float(vector[0]), float(vector[1]), 0.0])
    norm = float(np.linalg.norm(flat))
    if norm < 1e-9:
        raise ValueError("vector has no horizontal component to take a heading from")
    return flat / norm


def _roll_between(zero_roll_up: np.ndarray, up: np.ndarray, forward: np.ndarray) -> float:
    """Signed angle in degrees from the zero-roll image up axis to *up*, about the optical axis."""
    sine = float(np.dot(np.cross(zero_roll_up, up), forward))
    cosine = float(np.dot(zero_roll_up, up))
    return math.degrees(math.atan2(sine, cosine))


def _wrap_deg(angle_deg: float) -> float:
    """Fold an angle into (-180, 180]."""
    wrapped = math.fmod(float(angle_deg) + 180.0, 360.0)
    if wrapped <= 0.0:
        wrapped += 360.0
    return wrapped - 180.0
