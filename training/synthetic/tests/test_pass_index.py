"""Segmentation pass indices are unique and nonzero, set as each object is created."""

from __future__ import annotations

from types import SimpleNamespace


def make_object(pass_index: int = 0) -> SimpleNamespace:
    return SimpleNamespace(pass_index=pass_index)


def test_next_pass_index_skips_the_background_zero(blender_stubs: SimpleNamespace) -> None:
    from synthgen.pass_index import next_pass_index

    assert next_pass_index([]) == 1
    assert next_pass_index([0, 0]) == 1
    assert next_pass_index([3, 1, 7]) == 8


def test_new_objects_get_distinct_indices_above_existing_ones(
    blender_stubs: SimpleNamespace,
) -> None:
    from synthgen.pass_index import assign_pass_index

    existing = [make_object(1), make_object(2), make_object(0)]
    blender_stubs.bpy.data.objects = existing
    first, second = make_object(), make_object()
    assign_pass_index(first)
    existing.append(first)
    assign_pass_index(second)
    assert (first.pass_index, second.pass_index) == (3, 4)


def test_a_copy_that_inherited_an_index_gets_a_fresh_one(blender_stubs: SimpleNamespace) -> None:
    from synthgen.pass_index import assign_pass_index

    original = make_object(5)
    piece = make_object(5)
    blender_stubs.bpy.data.objects = [original, piece]
    assign_pass_index(piece)
    assert piece.pass_index == 6 and original.pass_index == 5


def test_blenderproc_wrappers_assign_to_the_wrapped_object(
    blender_stubs: SimpleNamespace,
) -> None:
    from synthgen.pass_index import assign_pass_index

    wrapped = make_object()
    wrapper = SimpleNamespace(blender_obj=wrapped)
    blender_stubs.bpy.data.objects = [wrapped]
    assert assign_pass_index(wrapper) is wrapper
    assert wrapped.pass_index == 1
