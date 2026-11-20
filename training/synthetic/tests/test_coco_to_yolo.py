"""Tests for the COCO to YOLO label line."""

from coco_to_yolo import _annotation_to_label_line


def label_line(category: str) -> list[str]:
    annotation = {"bbox": [0, 0, 10, 10], "keypoints": [10, 20, 2, 30, 40, 2, 50, 60, 2]}
    line = _annotation_to_label_line(annotation, 100, 100, 0, [category], 3)
    return line.split()


def test_mr_stabs_mk2_swaps_keypoints_one_and_two():
    fields = label_line("mr_stabs_mk2")
    assert fields[5:8] == ["0.300000", "0.400000", "2"]
    assert fields[8:11] == ["0.100000", "0.200000", "2"]
    assert fields[11:14] == ["0.500000", "0.600000", "2"]


def test_other_categories_keep_export_keypoint_order():
    fields = label_line("other_robot")
    assert fields[5:8] == ["0.100000", "0.200000", "2"]
    assert fields[8:11] == ["0.300000", "0.400000", "2"]
