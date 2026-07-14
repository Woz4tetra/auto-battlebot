"""Tests for asset discovery, VRAM audit loading, and class-id bookkeeping."""

import json
from pathlib import Path

import numpy as np
from synthgen.asset_index import (
    DistractorClassIdAssigner,
    SourceModelFiles,
    compute_source_budgets,
    discover_model_files,
    load_distractor_vram_audit,
    load_sidecar_keypoints,
    resolve_output_layout,
    resolve_start_index,
    setup_segmentation_labels,
)
from synthgen.configuration import DistractorSource, OutputConfig, RobotConfig
from synthgen.constants import (
    DISTRACTOR_CATEGORY_ID,
    NHRL_DISTRACTOR_INSTANCE_ID_BASE,
    SEG_OBJECT_CLASS_ID,
    SEG_ROBOT_CLASS_ID,
)


def _identity_resolve(path: Path) -> Path:
    return path


def _make_group(count: int, weight: float, kind: str = "cad") -> SourceModelFiles:
    return SourceModelFiles(
        files=[Path(f"m{i}.glb") for i in range(count)],
        weight=weight,
        source_dir=Path("."),
        kind=kind,
    )


class TestDiscoverModelFiles:
    def test_missing_directory_skipped(self, tmp_path: Path) -> None:
        sources = (DistractorSource(path=tmp_path / "nope"),)
        assert discover_model_files(sources, _identity_resolve) == []

    def test_finds_model_extensions_only(self, tmp_path: Path) -> None:
        d = tmp_path / "models"
        d.mkdir()
        for name in ("a.glb", "b.gltf", "c.obj", "d.ply", "ignored.txt", "e.json"):
            (d / name).touch()
        sources = (DistractorSource(path=d, weight=2.0, kind="cad"),)
        groups = discover_model_files(sources, _identity_resolve)
        assert len(groups) == 1
        assert sorted(f.name for f in groups[0].files) == ["a.glb", "b.gltf", "c.obj", "d.ply"]
        assert groups[0].weight == 2.0
        assert groups[0].kind == "cad"

    def test_kind_inferred_from_directory_name(self, tmp_path: Path) -> None:
        d = tmp_path / "objaverse_stuff"
        d.mkdir()
        (d / "a.glb").touch()
        sources = (DistractorSource(path=d),)
        groups = discover_model_files(sources, _identity_resolve)
        assert groups[0].kind == "objaverse"


class TestComputeSourceBudgets:
    def test_proportional_split(self) -> None:
        groups = [_make_group(10, 1.0), _make_group(10, 3.0)]
        budgets = compute_source_budgets(groups, 8)
        assert budgets == [2, 6]

    def test_minimum_one_slot(self) -> None:
        groups = [_make_group(10, 0.01), _make_group(10, 10.0)]
        budgets = compute_source_budgets(groups, 5)
        assert budgets[0] >= 1

    def test_remainder_goes_to_first(self) -> None:
        groups = [_make_group(10, 1.0), _make_group(10, 1.0), _make_group(10, 1.0)]
        budgets = compute_source_budgets(groups, 5)
        assert sum(budgets) >= 5


class TestVramAudit:
    def test_missing_csv(self, tmp_path: Path) -> None:
        assert load_distractor_vram_audit(tmp_path / "nope.csv", _identity_resolve) == {}

    def test_loads_rows(self, tmp_path: Path) -> None:
        csv_path = tmp_path / "audit.csv"
        csv_path.write_text("file,total_gpu_mb_est\n/models/a.glb,123.5\n/models/b.glb,7.25\n")
        estimates = load_distractor_vram_audit(csv_path, _identity_resolve)
        assert estimates[Path("/models/a.glb")] == 123.5
        assert estimates[Path("/models/b.glb")] == 7.25

    def test_malformed_rows_skipped(self, tmp_path: Path) -> None:
        csv_path = tmp_path / "audit.csv"
        csv_path.write_text(
            "file,total_gpu_mb_est\n/models/a.glb,not_a_number\n/models/b.glb,5.0\n,\n"
        )
        estimates = load_distractor_vram_audit(csv_path, _identity_resolve)
        assert estimates == {Path("/models/b.glb"): 5.0}


class TestSidecarKeypoints:
    def test_missing_sidecar(self, tmp_path: Path) -> None:
        assert load_sidecar_keypoints(tmp_path / "model.glb") is None

    def test_null_keypoints(self, tmp_path: Path) -> None:
        (tmp_path / "model.json").write_text(json.dumps({"keypoints": {"front": None}}))
        assert load_sidecar_keypoints(tmp_path / "model.glb") is None

    def test_invalid_json(self, tmp_path: Path) -> None:
        (tmp_path / "model.json").write_text("{not json")
        assert load_sidecar_keypoints(tmp_path / "model.glb") is None

    def test_valid_keypoints_converted_to_blender_axes(self, tmp_path: Path) -> None:
        (tmp_path / "model.json").write_text(
            json.dumps({"keypoints": {"front": [1.0, 2.0, 3.0], "back": [-1.0, 0.0, 0.5]}})
        )
        result = load_sidecar_keypoints(tmp_path / "model.glb")
        assert result is not None
        front, back = result
        np.testing.assert_allclose(front, [1.0, -3.0, 2.0])
        np.testing.assert_allclose(back, [-1.0, -0.5, 0.0])


def _robot(name: str, class_id: int) -> RobotConfig:
    from synthgen.configuration import KeypointPair

    return RobotConfig(
        name=name,
        model_path=Path(f"{name}.gltf"),
        keypoints=KeypointPair(front=np.zeros(3), back=np.zeros(3)),
        color_mapping=(),
        class_id=class_id,
    )


class TestSegmentationLabels:
    ROBOTS = (_robot("alpha", 0), _robot("beta", 1))

    def test_keypoint_mode_is_empty(self) -> None:
        ids, names, next_id = setup_segmentation_labels(self.ROBOTS, False)
        assert ids == {}
        assert names == {}
        assert next_id == SEG_ROBOT_CLASS_ID + 1

    def test_segmentation_mode_scheme(self) -> None:
        ids, names, next_id = setup_segmentation_labels(self.ROBOTS, True)
        assert names[0] == "background"
        assert names[1] == "floor"
        assert names[2] == "object"
        assert names[3] == "robot"
        assert ids == {1: 4, 2: 5}
        assert names[4] == "alpha"
        assert names[5] == "beta"
        assert next_id == 6


class TestDistractorClassIdAssigner:
    def test_keypoint_mode_constant_category(self) -> None:
        assigner = DistractorClassIdAssigner(False, SEG_ROBOT_CLASS_ID + 1)
        assert assigner(Path("x.glb"), "objaverse") == DISTRACTOR_CATEGORY_ID
        assert assigner(Path("y.glb"), "cad") == DISTRACTOR_CATEGORY_ID

    def test_segmentation_mode_by_kind(self) -> None:
        assigner = DistractorClassIdAssigner(True, 6)
        assert assigner(Path("x.glb"), "objaverse") == SEG_OBJECT_CLASS_ID
        assert assigner(Path("y.glb"), "cad") == SEG_ROBOT_CLASS_ID
        assert assigner(Path("z.glb"), "other") == 7
        assert assigner(Path("w.glb"), "other") == 8

    def test_instance_id_allocation(self) -> None:
        assigner = DistractorClassIdAssigner(False, 4)
        assert assigner.allocate_keypoint_instance_id() == NHRL_DISTRACTOR_INSTANCE_ID_BASE
        assert assigner.allocate_keypoint_instance_id() == NHRL_DISTRACTOR_INSTANCE_ID_BASE + 1

    def test_instance_id_zero_in_segmentation_mode(self) -> None:
        assigner = DistractorClassIdAssigner(True, 6)
        assert assigner.allocate_keypoint_instance_id() == 0


class TestOutputResolution:
    def test_layout_created(self, tmp_path: Path) -> None:
        output = OutputConfig(
            image_dir=tmp_path / "ds" / "images",
            label_dir=tmp_path / "ds" / "labels",
            num_images=1,
        )
        layout = resolve_output_layout(output, _identity_resolve)
        assert layout.image_dir.is_dir()
        assert layout.label_dir.is_dir()
        assert layout.dataset_root == tmp_path / "ds"
        assert layout.data_yml_path == tmp_path / "ds" / "data.yml"

    def test_start_index_explicit(self, tmp_path: Path) -> None:
        assert resolve_start_index(42, tmp_path) == 42

    def test_start_index_empty_dir(self, tmp_path: Path) -> None:
        assert resolve_start_index(None, tmp_path) == 0

    def test_start_index_auto_resume(self, tmp_path: Path) -> None:
        for name in ("000000.jpg", "000007.jpg", "_debug_frame0.jpg", "notes.jpg"):
            (tmp_path / name).touch()
        assert resolve_start_index(None, tmp_path) == 8
