"""Distractor instances are compared by identity: they wrap live Blender handles."""

from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock


def make_instance(distractors: Any, parent: Any) -> Any:
    return distractors.DistractorInstance(
        parent=parent, meshes=[], native_size=1.0, source_kind="cad"
    )


def test_instance_identity_semantics(blender_stubs: SimpleNamespace) -> None:
    from synthgen import distractors

    first = make_instance(distractors, "a")
    twin = make_instance(distractors, "a")
    assert first != twin
    assert [first, twin].index(twin) == 1
    pool = [first, twin]
    pool.remove(twin)
    assert pool == [first]
    assert pool[0] is first


def make_vertex_colored_object(color_attributes: list, tree: Any) -> Any:
    slot = SimpleNamespace(material=SimpleNamespace(use_nodes=True, node_tree=tree))
    return SimpleNamespace(
        name="blob",
        data=SimpleNamespace(color_attributes=color_attributes, vertex_colors=[]),
        material_slots=[slot],
    )


def test_vertex_colors_are_wired_into_the_base_color(blender_stubs: SimpleNamespace) -> None:
    from synthgen import distractors

    bsdf = SimpleNamespace(type="BSDF_PRINCIPLED", inputs={"Base Color": SimpleNamespace(links=[])})
    vcol = SimpleNamespace(outputs={"Color": "color_socket"}, layer_name=None)
    tree = SimpleNamespace(nodes=FakeNodes([bsdf], vcol), links=MagicMock())
    obj = make_vertex_colored_object([SimpleNamespace(name="COLOR_0")], tree)

    distractors._wire_vertex_colors(obj)

    assert vcol.layer_name == "COLOR_0"
    tree.links.new.assert_called_once_with("color_socket", bsdf.inputs["Base Color"])


def test_mesh_without_vertex_colors_keeps_its_material(blender_stubs: SimpleNamespace) -> None:
    from synthgen import distractors

    tree = SimpleNamespace(nodes=FakeNodes([], None), links=MagicMock())
    obj = make_vertex_colored_object([], tree)

    distractors._wire_vertex_colors(obj)

    tree.links.new.assert_not_called()


class FakeNodes(list):
    def __init__(self, nodes: list, created: Any) -> None:
        super().__init__(nodes)
        self.created = created

    def new(self, _kind: str) -> Any:
        return self.created
