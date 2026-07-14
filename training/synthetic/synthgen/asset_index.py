"""Filesystem discovery and class-id bookkeeping for assets (no Blender imports).

Covers distractor model discovery, VRAM audit loading, keypoint sidecar reading,
segmentation label maps, and output directory/start-index resolution.
"""

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from synthgen.configuration import DistractorSource, OutputConfig, RobotConfig
from synthgen.constants import (
    BACKGROUND_CATEGORY_ID,
    DISTRACTOR_CATEGORY_ID,
    MODEL_EXTENSIONS,
    NHRL_DISTRACTOR_INSTANCE_ID_BASE,
    SEG_FLOOR_CLASS_ID,
    SEG_OBJECT_CLASS_ID,
    SEG_ROBOT_CLASS_ID,
)
from synthgen.geometry import model_to_blender_local
from synthgen.logsetup import get_logger

logger = get_logger(__name__)

PathResolveFn = Callable[[Path], Path]


@dataclass(frozen=True)
class SourceModelFiles:
    """Model files found in one distractor source directory."""

    files: list[Path]
    weight: float
    source_dir: Path
    kind: str


@dataclass(frozen=True)
class OutputLayout:
    """Resolved output locations for images, labels, and dataset metadata."""

    image_dir: Path
    label_dir: Path
    dataset_root: Path
    data_yml_path: Path


def discover_model_files(
    sources: tuple[DistractorSource, ...], resolve: PathResolveFn
) -> list[SourceModelFiles]:
    """Find loadable 3D model files grouped by source directory.

    Args:
        sources: Configured distractor sources.
        resolve: Path resolver for relative source paths.

    Returns:
        One entry per source directory that exists and contains model files.
    """
    groups: list[SourceModelFiles] = []
    for src in sources:
        d = resolve(src.path)
        if not d.exists():
            logger.warning("Distractor source directory not found: %s", d)
            continue
        files: list[Path] = []
        for ext in MODEL_EXTENSIONS:
            files.extend(d.glob(f"*{ext}"))
        if files:
            groups.append(
                SourceModelFiles(
                    files=files, weight=src.weight, source_dir=d, kind=src.effective_kind()
                )
            )
    return groups


def compute_source_budgets(groups: list[SourceModelFiles], max_count: int) -> list[int]:
    """Split *max_count* slots across sources in proportion to their weights."""
    total_weight = sum(g.weight for g in groups)
    budgets = [max(1, round(max_count * g.weight / total_weight)) for g in groups]
    remaining = max_count - sum(budgets)
    if remaining > 0:
        budgets[0] += remaining
    return budgets


def load_distractor_vram_audit(csv_path: Path, resolve: PathResolveFn) -> dict[Path, float]:
    """Load precomputed distractor VRAM estimates keyed by absolute model path.

    Args:
        csv_path: Audit CSV location (possibly relative).
        resolve: Path resolver for the CSV and the model paths in its rows.

    Returns:
        Mapping of resolved model path to estimated VRAM megabytes; empty when
        the CSV is missing or unreadable.
    """
    resolved = resolve(csv_path)
    if not resolved.exists():
        logger.info("VRAM audit CSV not found: %s", resolved)
        return {}

    estimates: dict[Path, float] = {}
    try:
        with resolved.open(newline="") as f:
            reader = csv.DictReader(f)
            for row_num, row in enumerate(reader, start=2):
                file_str = row.get("file")
                mb_str = row.get("total_gpu_mb_est")
                if not file_str or not mb_str:
                    continue
                try:
                    estimates[resolve(Path(file_str))] = float(mb_str)
                except (ValueError, OSError) as e:
                    logger.warning("Skipping VRAM audit row %d (%s): %s", row_num, file_str, e)
    except (OSError, csv.Error) as e:
        logger.warning("Failed to parse VRAM audit CSV %s: %s", resolved, e)
        return {}

    logger.info("Loaded VRAM audit entries: %d from %s", len(estimates), resolved)
    return estimates


def load_sidecar_keypoints(model_path: Path) -> tuple[np.ndarray, np.ndarray] | None:
    """Load front/back keypoints from a distractor model's JSON sidecar.

    Returns the keypoints in Blender local axes (converted from the sidecar's
    GLTF-native frame via ``model_to_blender_local``), or ``None`` when the
    sidecar is missing or its keypoints are null (degenerate mesh).
    """
    sidecar = model_path.with_suffix(".json")
    if not sidecar.exists():
        return None
    try:
        with sidecar.open() as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError):
        return None
    kp = data.get("keypoints")
    if not kp or kp.get("front") is None or kp.get("back") is None:
        return None
    return model_to_blender_local(kp["front"]), model_to_blender_local(kp["back"])


def setup_segmentation_labels(
    robot_configs: tuple[RobotConfig, ...], is_segmentation_mode: bool
) -> tuple[dict[int, int], dict[int, str], int]:
    """Build segmentation class-id maps and return them with the next free class id.

    Args:
        robot_configs: Parsed ``[[robots]]`` entries (order defines instance ids).
        is_segmentation_mode: Whether segmentation labels are being produced.

    Returns:
        ``(robot_instance_id -> seg class id, seg class id -> name, next free id)``.
    """
    seg_robot_class_ids: dict[int, int] = {}
    seg_label_names: dict[int, str] = {}
    if not is_segmentation_mode:
        return seg_robot_class_ids, seg_label_names, SEG_ROBOT_CLASS_ID + 1

    seg_label_names[BACKGROUND_CATEGORY_ID] = "background"
    seg_label_names[SEG_FLOOR_CLASS_ID] = "floor"
    seg_label_names[SEG_OBJECT_CLASS_ID] = "object"
    seg_label_names[SEG_ROBOT_CLASS_ID] = "robot"
    next_seg_class_id = SEG_ROBOT_CLASS_ID + 1
    for ri, rcfg in enumerate(robot_configs, start=1):
        seg_robot_class_ids[ri] = next_seg_class_id
        seg_label_names[next_seg_class_id] = rcfg.name
        next_seg_class_id += 1
    return seg_robot_class_ids, seg_label_names, next_seg_class_id


class DistractorClassIdAssigner:
    """Stateful callable that assigns category ids to distractor models."""

    def __init__(self, is_segmentation_mode: bool, next_seg_class_id: int) -> None:
        self.is_segmentation_mode = is_segmentation_mode
        self.next_seg_class_id = next_seg_class_id
        self._next_instance_id = NHRL_DISTRACTOR_INSTANCE_ID_BASE

    def __call__(self, model_path: Path, source_kind: str) -> int:
        """Return the category id for one distractor model."""
        if not self.is_segmentation_mode:
            return DISTRACTOR_CATEGORY_ID

        if source_kind == "objaverse":
            return SEG_OBJECT_CLASS_ID
        elif source_kind == "cad":
            return SEG_ROBOT_CLASS_ID
        else:
            self.next_seg_class_id += 1
            return self.next_seg_class_id

    def allocate_keypoint_instance_id(self) -> int:
        """Unique robot_instance_id for a keypoint-annotated CAD distractor.

        Returns 0 in segmentation mode (instance ids are managed there by the
        segmentation pass, and distractor keypoints are not emitted).
        """
        if self.is_segmentation_mode:
            return 0
        instance_id = self._next_instance_id
        self._next_instance_id += 1
        return instance_id


def resolve_output_layout(output: OutputConfig, resolve: PathResolveFn) -> OutputLayout:
    """Resolve and create the output directories.

    Args:
        output: Parsed ``[output]`` config.
        resolve: Path resolver for the configured directories.

    Returns:
        Resolved layout with directories created.
    """
    image_dir = resolve(output.image_dir)
    label_dir = resolve(output.label_dir)
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    dataset_root = image_dir.parent if image_dir.parent == label_dir.parent else label_dir.parent
    return OutputLayout(
        image_dir=image_dir,
        label_dir=label_dir,
        dataset_root=dataset_root,
        data_yml_path=dataset_root / "data.yml",
    )


def resolve_start_index(start_index: int | None, output_image_dir: Path) -> int:
    """Return the explicit start index, or auto-detect the next from existing output."""
    if start_index is not None:
        return start_index
    existing = [int(p.stem) for p in output_image_dir.glob("*.jpg") if p.stem.isdigit()]
    resolved = max(existing) + 1 if existing else 0
    if existing:
        logger.info(
            "Auto-resuming from index %d (%d existing images found)", resolved, len(existing)
        )
    return resolved
