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


class FakeWorld:
    def __init__(self, tag: str) -> None:
        self.tag = tag

    def copy(self) -> "FakeWorld":
        return FakeWorld(self.tag)

    def __matmul__(self, point: Any) -> Any:
        return point


class FakeMesh:
    def __init__(self, name: str, events: list[str], stale: bool) -> None:
        self.name = name
        self.type = "MESH"
        self.bound_box = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)]
        self.events = events
        self._world = FakeWorld("stale" if stale else name)
        self.world_after_update = FakeWorld(name)

    @property
    def matrix_world(self) -> FakeWorld:
        self.events.append(f"read_world:{self._world.tag}")
        return self._world

    @matrix_world.setter
    def matrix_world(self, value: Any) -> None:
        self.events.append("write_world")

    @property
    def matrix_local(self) -> Any:
        return None

    @matrix_local.setter
    def matrix_local(self, value: FakeWorld) -> None:
        self.events.append(f"write_local:{value.tag}")


def import_with_fakes(blender_stubs: SimpleNamespace, tmp_path: Any) -> list[str]:
    from synthgen import robots

    events: list[str] = []
    mesh = FakeMesh("part", events, stale=True)

    def update() -> None:
        events.append("update")
        mesh._world = mesh.world_after_update

    blender_stubs.bpy.context.view_layer.update.side_effect = update
    blender_stubs.bpy.context.selected_objects = [mesh]
    blender_stubs.mathutils.Vector.side_effect = lambda c: SimpleNamespace(x=c[0], y=c[1], z=c[2])
    model = tmp_path / "bot.glb"
    model.write_bytes(b"")

    robots.import_gltf_as_robot(model, lambda path: path, scale=2.0)
    return events


def test_gltf_import_updates_the_view_layer_before_capturing_world_matrices(
    blender_stubs: SimpleNamespace, tmp_path: Any
) -> None:
    events = import_with_fakes(blender_stubs, tmp_path)

    assert events[0] == "update"
    assert events[1] == "read_world:part", "matrix_world is stale until the depsgraph updates"


def test_gltf_import_pins_parts_through_matrix_local_not_matrix_world(
    blender_stubs: SimpleNamespace, tmp_path: Any
) -> None:
    events = import_with_fakes(blender_stubs, tmp_path)

    assert "write_local:part" in events
    assert "write_world" not in events, "writing matrix_world cancels the parent's scale"
