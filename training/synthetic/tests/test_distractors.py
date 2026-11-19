"""Distractor instances are compared by identity: they wrap live Blender handles."""

from types import SimpleNamespace
from typing import Any


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
