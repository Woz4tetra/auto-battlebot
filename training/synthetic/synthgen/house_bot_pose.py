"""Where the house bot stands in one scene (pure module, no Blender).

The NHRL house bot drives around the cage during a fight, so a fixed pose teaches the detector
that it only ever sits in one corner facing one way. Each cage scene draws a new one: anywhere on
the mat with its whole footprint on it, any heading, and clear of the robots already placed so
the two never interpenetrate. `cage_scene.CageStage.place_house_bot` applies it.
"""

from __future__ import annotations

import math
import random
from collections.abc import Sequence
from dataclasses import dataclass

# Draws before giving up on full clearance and keeping the roomiest draw seen.
PLACEMENT_TRIES = 200


@dataclass(frozen=True)
class HouseBotPose:
    x_m: float
    y_m: float
    yaw_deg: float
    # Smallest centre distance to a robot minus the required one; negative when no draw fit.
    clearance_margin_m: float


def footprint_radius(size_xy: tuple[float, float]) -> float:
    """Radius of the circle around the house bot's footprint, whatever its heading."""
    return math.hypot(size_xy[0], size_xy[1]) / 2


def sample_house_bot_pose(
    mat_half: float,
    radius: float,
    robot_xy: Sequence[Sequence[float]],
    robot_radius: float,
    clearance: float,
    rng: random.Random | None = None,
) -> HouseBotPose:
    """Draw a pose with the footprint on the mat and at least `clearance` from every robot.

    Positions are uniform over the square that keeps the footprint circle on the mat; the
    heading is uniform over the full turn. A draw is accepted when every robot centre is at
    least `radius + robot_radius + clearance` away. After `PLACEMENT_TRIES` rejections the
    draw with the most room is kept, and its negative margin says by how much it fell short.
    """
    rng = rng or random.Random()
    reach = mat_half - radius
    if reach <= 0:
        raise ValueError(f"house bot radius {radius:.3f} m does not fit a {2 * mat_half} m mat")
    needed = radius + robot_radius + clearance
    best: HouseBotPose | None = None
    for _ in range(PLACEMENT_TRIES):
        x, y = rng.uniform(-reach, reach), rng.uniform(-reach, reach)
        nearest = min((math.hypot(x - p[0], y - p[1]) for p in robot_xy), default=math.inf)
        pose = HouseBotPose(x, y, rng.uniform(0.0, 360.0), nearest - needed)
        if pose.clearance_margin_m >= 0:
            return pose
        if best is None or pose.clearance_margin_m > best.clearance_margin_m:
            best = pose
    assert best is not None
    return best
