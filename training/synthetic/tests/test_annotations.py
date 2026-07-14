"""Tests for annotation math and the frozen YOLO writer formats."""

from pathlib import Path

import numpy as np
import pytest
from synthgen.annotations import (
    bbox_from_category_segmap,
    build_names_list,
    check_keypoint_visibility,
    normalize_annotation_mode,
    segmentation_annotations_from_segmap,
    write_data_yml,
    write_label_index,
    write_yolo_labels,
    write_yolo_seg_labels,
)


class TestNormalizeAnnotationMode:
    def test_valid_modes(self) -> None:
        assert normalize_annotation_mode(" Keypoints_BBOX ") == "keypoints_bbox"
        assert normalize_annotation_mode("segmentation_bbox") == "segmentation_bbox"

    def test_invalid_mode(self) -> None:
        with pytest.raises(ValueError, match="annotation_mode"):
            normalize_annotation_mode("boxes")


class TestBboxFromSegmap:
    def test_absent_id_returns_none(self) -> None:
        seg = np.zeros((10, 20), dtype=np.int32)
        assert bbox_from_category_segmap(seg, 1, 20, 10) is None

    def test_single_pixel(self) -> None:
        seg = np.zeros((10, 20), dtype=np.int32)
        seg[5, 10] = 1
        bbox = bbox_from_category_segmap(seg, 1, 20, 10)
        assert bbox is not None
        cx, cy, w, h = bbox
        assert cx == pytest.approx(10 / 20)
        assert cy == pytest.approx(5 / 10)
        assert w == pytest.approx(0.0)
        assert h == pytest.approx(0.0)

    def test_rectangle(self) -> None:
        seg = np.zeros((100, 200), dtype=np.int32)
        seg[20:41, 50:101] = 7
        bbox = bbox_from_category_segmap(seg, 7, 200, 100)
        assert bbox is not None
        cx, cy, w, h = bbox
        assert cx == pytest.approx((50 + 100) / 2 / 200)
        assert cy == pytest.approx((20 + 40) / 2 / 100)
        assert w == pytest.approx(50 / 200)
        assert h == pytest.approx(20 / 100)

    def test_extra_channel_dim_squeezed(self) -> None:
        seg = np.zeros((10, 20, 1), dtype=np.int32)
        seg[5, 10, 0] = 1
        assert bbox_from_category_segmap(seg, 1, 20, 10) is not None


class TestKeypointVisibility:
    DEPTH = np.full((10, 20), 2.0, dtype=np.float32)

    def test_out_of_frame(self) -> None:
        assert check_keypoint_visibility(-0.1, 0.5, 2.0, self.DEPTH, 20, 10) == 0
        assert check_keypoint_visibility(0.5, 1.2, 2.0, self.DEPTH, 20, 10) == 0

    def test_visible_within_tolerance(self) -> None:
        assert check_keypoint_visibility(0.5, 0.5, 2.02, self.DEPTH, 20, 10) == 2

    def test_occluded_beyond_tolerance(self) -> None:
        assert check_keypoint_visibility(0.5, 0.5, 3.0, self.DEPTH, 20, 10) == 1

    def test_tolerance_boundary_is_occluded(self) -> None:
        # abs(diff) < tolerance is strict: exactly at tolerance counts occluded.
        # 2.5 and 0.5 are exactly representable, so the comparison is exact.
        assert check_keypoint_visibility(0.5, 0.5, 2.5, self.DEPTH, 20, 10, tolerance=0.5) == 1

    def test_ignore_occlusion_in_frame(self) -> None:
        assert (
            check_keypoint_visibility(0.5, 0.5, 99.0, self.DEPTH, 20, 10, ignore_occlusion=True)
            == 2
        )

    def test_ignore_occlusion_still_flags_out_of_frame(self) -> None:
        assert (
            check_keypoint_visibility(1.5, 0.5, 2.0, self.DEPTH, 20, 10, ignore_occlusion=True) == 0
        )


class TestSegAnnotationsFromSegmap:
    def test_background_excluded(self) -> None:
        seg = np.zeros((20, 20), dtype=np.int32)
        assert segmentation_annotations_from_segmap(seg, 20, 20) == []

    def test_polygon_for_blob(self) -> None:
        seg = np.zeros((20, 20), dtype=np.int32)
        seg[5:15, 5:15] = 3
        annotations = segmentation_annotations_from_segmap(seg, 20, 20)
        assert len(annotations) == 1
        class_id, polygon = annotations[0]
        assert class_id == 3
        assert len(polygon) >= 3
        assert all(0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 for x, y in polygon)

    def test_tiny_blob_becomes_rectangle(self) -> None:
        seg = np.zeros((20, 20), dtype=np.int32)
        seg[10, 10] = 2  # single pixel -> contour with < 3 points
        annotations = segmentation_annotations_from_segmap(seg, 20, 20)
        assert len(annotations) == 1
        _, polygon = annotations[0]
        assert len(polygon) == 4

    def test_min_bbox_dim_filters(self) -> None:
        seg = np.zeros((20, 20), dtype=np.int32)
        seg[10, 10] = 2
        assert segmentation_annotations_from_segmap(seg, 20, 20, min_bbox_dim=2) == []

    def test_multiple_classes_sorted(self) -> None:
        seg = np.zeros((20, 40), dtype=np.int32)
        seg[2:8, 2:8] = 5
        seg[12:18, 30:38] = 3
        annotations = segmentation_annotations_from_segmap(seg, 40, 20)
        assert [class_id for class_id, _ in annotations] == [3, 5]


class TestWriterFormats:
    """Golden tests pinning the exact bytes of the frozen output contract."""

    def test_yolo_keypoint_labels(self, tmp_path: Path) -> None:
        path = tmp_path / "000001.txt"
        write_yolo_labels(
            path,
            [
                (0, (0.5, 0.25, 0.125, 0.0625), [(0.1, 0.2, 2), (0.3, 0.4, 1)]),
                (2, (0.5, 0.5, 0.5, 0.5), [(0.9, 0.9, 0), (0.111111, 0.222222, 2)]),
            ],
        )
        expected = (
            "0 0.500000 0.250000 0.125000 0.062500 0.100000 0.200000 2 0.300000 0.400000 1\n"
            "2 0.500000 0.500000 0.500000 0.500000 0.000000 0.000000 0 0.111111 0.222222 2\n"
        )
        assert path.read_text() == expected

    def test_yolo_seg_labels(self, tmp_path: Path) -> None:
        path = tmp_path / "000002.txt"
        write_yolo_seg_labels(path, [(3, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])])
        expected = "3 0.000000 0.000000 1.000000 0.000000 1.000000 1.000000\n"
        assert path.read_text() == expected

    def test_label_index(self, tmp_path: Path) -> None:
        path = tmp_path / "label_index.txt"
        write_label_index(path, {3: "robot", 0: "background", 1: "floor"})
        assert path.read_text() == "0:background\n1:floor\n3:robot\n"

    def test_data_yml_keypoint_mode(self, tmp_path: Path) -> None:
        path = tmp_path / "data.yml"
        write_data_yml(path, tmp_path, ["mr_stabs_mk2", "mrs_buff_mk3"], "keypoints_bbox")
        content = path.read_text()
        assert f"path: {tmp_path.resolve()}\n" in content
        assert "train: images\n" in content
        assert "val: images\n" in content
        assert "nc: 2\n" in content
        assert "names: ['mr_stabs_mk2', 'mrs_buff_mk3']\n" in content
        assert "kpt_shape: [2, 3]\n" in content
        assert "flip_idx: [0, 1]\n" in content

    def test_data_yml_seg_mode_has_no_kpt_shape(self, tmp_path: Path) -> None:
        path = tmp_path / "data.yml"
        write_data_yml(path, tmp_path, ["background", "floor"], "segmentation_bbox")
        content = path.read_text()
        assert "kpt_shape" not in content
        assert "flip_idx" not in content


class TestBuildNamesList:
    def test_empty(self) -> None:
        assert build_names_list({}) == []

    def test_gap_filling(self) -> None:
        assert build_names_list({0: "a", 3: "d"}) == ["a", "unknown_1", "unknown_2", "d"]
