from __future__ import annotations

import pytest

from auto_battlebot.segmentation import field_labels


@pytest.mark.parametrize(
    "stem",
    [
        "Cage-6-Red-2024-10-26-0001",
        "x_tournamentID-nhrl_oct24_3lb_Cage-6-Overhead-High_clip",
        "BZ-nhrl_feb26_3lb-cole-anubis-W-53-Cage-6-Overhead-High_clip",
    ],
)
def test_cage_token_in_bare_and_brettzone_names(stem: str) -> None:
    assert field_labels.parse_field(stem) == "cage6"


def test_cage_token_beats_camera_rig() -> None:
    assert field_labels.parse_field("SkyCam-A_x_Cage-3-Overhead-High_y") == "cage3"
    assert field_labels.parse_field("SkyCam-A_y") == "skycam"


def test_frame_tails_collapse_to_one_scene() -> None:
    first = field_labels.parse_scene("match_one_yolo_seg__frame_12")
    second = field_labels.parse_scene("match_one_yolo_seg__frame_900")
    assert first == second == "match_one"
    assert field_labels.parse_scene("clip-0001") == field_labels.parse_scene("clip-0420")


def test_roboflow_tail_stripped() -> None:
    assert field_labels.parse_scene("scene-0012_jpg.rf.0a1b2c") == "scene"
    assert field_labels.parse_scene("scene-0300_jpg.rf.ffee") == "scene"
    assert field_labels.parse_scene("scene_repaired_12_jpg.rf.0a1b2c") == "scene"


def test_augment_copies_share_source_and_scene() -> None:
    original = field_labels.parse_name("Cage-2-Red-0001")
    copy = field_labels.parse_name("floor_dataset__Cage-2-Red-0001_augment-3")
    assert copy.augment_index == 3
    assert copy.source == original.source
    assert copy.scene == original.scene
