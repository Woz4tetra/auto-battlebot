"""Robot instances are compared by identity: they wrap live Blender handles."""

from types import SimpleNamespace
from typing import Any

import numpy as np


def make_robot(robots: Any) -> Any:
    return robots.RobotInstance(
        name="bot",
        instance_id=1,
        meshes=[],
        parent="parent",
        bbox=[],
        size=1.0,
        kp_front=np.zeros(3),
        kp_back=np.ones(3),
        class_id=0,
        config=None,
    )


def test_robot_instance_identity_semantics(blender_stubs: SimpleNamespace) -> None:
    from synthgen import robots

    first = make_robot(robots)
    twin = make_robot(robots)
    assert first != twin
    scene_robots = [first, twin]
    assert scene_robots.index(twin) == 1
    scene_robots.remove(twin)
    assert scene_robots == [first]
    assert scene_robots[0] is first
