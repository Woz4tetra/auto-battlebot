#!/usr/bin/env python3
"""Shared utilities for the floor segmentation pipeline."""

from __future__ import annotations

import hashlib
import json
import logging
import random
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore


LOGGER = logging.getLogger("floor_pipeline")


def configure_logging(verbose: bool = False) -> None:
    """Configure process-wide logging."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


def read_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as f:
        return tomllib.load(f)


def mkdir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def json_dump(path: Path, payload: Any) -> None:
    mkdir(path.parent)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=False)


def json_load(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def append_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    mkdir(path.parent)
    with path.open("a", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, sort_keys=False) + "\n")


def run_cmd(command: list[str], check: bool = True) -> subprocess.CompletedProcess:
    LOGGER.debug("Running command: %s", " ".join(command))
    return subprocess.run(
        command,
        check=check,
        text=True,
        capture_output=True,
    )


def sha256_file(path: Path, block_size: int = 1024 * 1024) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            chunk = f.read(block_size)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def choose_deterministic(items: list[Any], count: int, seed: int) -> list[Any]:
    rng = random.Random(seed)
    if count >= len(items):
        return list(items)
    return rng.sample(items, count)


def now_ts() -> float:
    return time.time()


@dataclass
class PathsConfig:
    root: Path
    raw_videos: Path
    processed_videos: Path
    metadata_dir: Path
    masks_dir: Path
    filtered_masks_dir: Path
    export_dir: Path
    logs_dir: Path
    checkpoints_dir: Path


@dataclass
class ScrapeConfig:
    base_url: str
    index_url: str
    robot_fights_url: str
    max_fights: int
    random_seed: int
    min_cameras: int
    retries: int
    timeout_seconds: int
    allow_all_camera_angles: bool = True


@dataclass
class VideoConfig:
    target_width: int
    target_height: int
    target_fps: int
    crf: int
    preset: str
    chunk_seconds: int
    keep_audio: bool = False


@dataclass
class Sam3Config:
    adapter_module: str
    checkpoint: str
    device: str
    amp: bool
    text_threshold: float
    box_threshold: float
    model_cfg: str = ""
    prompt_tags: list[str] = field(default_factory=list)
    negative_tags: list[str] = field(default_factory=list)
    fallback_positive_samples: int = 12
    fallback_negative_samples: int = 12


@dataclass
class QualityConfig:
    min_floor_fraction: float
    max_floor_fraction: float
    max_area_delta: float
    min_iou_with_previous: float
    closing_kernel: int
    opening_kernel: int


@dataclass
class ExportConfig:
    stride: int
    min_polygon_points: int
    polygon_epsilon_ratio: float
    image_ext: str = ".jpg"


@dataclass
class ClusterConfig:
    gpu_ids: list[int]
    workers_per_gpu: int
    async_prefetch: int


@dataclass
class PipelineConfig:
    paths: PathsConfig
    scrape: ScrapeConfig
    video: VideoConfig
    sam3: Sam3Config
    quality: QualityConfig
    export: ExportConfig
    cluster: ClusterConfig


def load_pipeline_config(config_path: Path) -> PipelineConfig:
    raw = read_toml(config_path)

    paths_root = Path(raw["paths"]["root"]).resolve()
    paths = PathsConfig(
        root=paths_root,
        raw_videos=paths_root / raw["paths"]["raw_videos"],
        processed_videos=paths_root / raw["paths"]["processed_videos"],
        metadata_dir=paths_root / raw["paths"]["metadata_dir"],
        masks_dir=paths_root / raw["paths"]["masks_dir"],
        filtered_masks_dir=paths_root / raw["paths"]["filtered_masks_dir"],
        export_dir=paths_root / raw["paths"]["export_dir"],
        logs_dir=paths_root / raw["paths"]["logs_dir"],
        checkpoints_dir=paths_root / raw["paths"]["checkpoints_dir"],
    )

    scrape = ScrapeConfig(**raw["scrape"])
    video = VideoConfig(**raw["video"])
    sam3 = Sam3Config(**raw["sam3"])
    quality = QualityConfig(**raw["quality"])
    export = ExportConfig(**raw["export"])
    cluster = ClusterConfig(**raw["cluster"])

    return PipelineConfig(
        paths=paths,
        scrape=scrape,
        video=video,
        sam3=sam3,
        quality=quality,
        export=export,
        cluster=cluster,
    )
