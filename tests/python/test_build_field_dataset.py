"""training/deeplab/build_field_dataset.py: video grouping, collision-free names, val split."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def builder(load_training):
    return load_training("training/deeplab/build_field_dataset.py")


def test_video_key_groups_slices_of_one_video(builder) -> None:
    first = builder.video_key(Path("cage3/clipA__0_segmask/frame_0001.jpg"))
    second = builder.video_key(Path("cage3/clipA__900_segmask/frame_0001.jpg"))
    other = builder.video_key(Path("cage3/clipB__0_segmask/frame_0001.jpg"))
    assert first == second
    assert first != other


def test_video_key_uses_leading_video_id_for_segmentflow_exports(builder) -> None:
    key_a = builder.video_key(Path("abcdefghijk_export/a/1.jpg"))
    key_b = builder.video_key(Path("abcdefghijk_other/b/2.jpg"))
    assert key_a == key_b == "abcdefghijk"


def test_video_key_falls_back_to_top_level_directory(builder) -> None:
    assert builder.video_key(Path("short/1.jpg")) == "short"


def test_scene_stem_is_collision_free(builder) -> None:
    stems = {
        builder.scene_stem(Path("exportA/frames/0001.jpg")),
        builder.scene_stem(Path("exportB/frames/0001.jpg")),
        builder.scene_stem(Path("exportA/other/0001.jpg")),
        builder.scene_stem(Path("0001.jpg")),
    }
    assert len(stems) == 4


def test_split_never_overshoots_val_frac(builder) -> None:
    sizes = [50, 30, 20, 5, 5]
    groups = {f"v{i}": [(Path("a"), Path("b"))] * size for i, size in enumerate(sizes)}
    total = sum(sizes)
    for seed in range(20):
        assignment = builder.assign_videos(groups, 0.2, seed)
        in_val = sum(len(groups[k]) for k, side in assignment.items() if side == "val")
        assert in_val <= total * 0.2
        assert set(assignment) == set(groups)
