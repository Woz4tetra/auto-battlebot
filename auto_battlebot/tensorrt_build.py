"""TensorRT builder scaffolding shared by the YOLO and DeepLab export CLIs."""

from __future__ import annotations

import ctypes
import platform
from pathlib import Path
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    import tensorrt as trt

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TIMING_CACHE = REPO_ROOT / ".cache" / "tensorrt" / "timing.cache"


def has_lean_runtime() -> bool:
    """Check if the TensorRT lean runtime is available (needed for VERSION_COMPATIBLE)."""
    try:
        ctypes.CDLL("libnvinfer_lean.so.10")
        return True
    except OSError:
        return False


def get_compute_capability() -> tuple[int, int]:
    """Query GPU compute capability of device 0."""
    if torch.cuda.is_available():
        return torch.cuda.get_device_capability(0)
    raise RuntimeError("CUDA not available — cannot determine GPU compute capability")


def engine_path_with_platform_tag(path: Path, precision_tag: str | None = None) -> Path:
    """Append the platform and GPU compute capability tag to an engine filename."""
    arch = platform.machine()
    major, minor = get_compute_capability()
    tag = f"{arch}_sm{major}{minor}"
    if precision_tag:
        tag = f"{precision_tag}_{tag}"
    suffix = path.suffix if path.suffix else ".engine"
    return path.parent / f"{path.stem}_{tag}{suffix}"


def load_timing_cache(config: trt.IBuilderConfig, path: Path | None) -> None:
    """Seed the builder with previously measured tactic timings, if any."""
    if path is None:
        return
    blob = path.read_bytes() if path.is_file() else b""
    cache = config.create_timing_cache(blob)
    if cache is not None:
        config.set_timing_cache(cache, ignore_mismatch=False)


def save_timing_cache(config: trt.IBuilderConfig, path: Path | None) -> None:
    """Persist tactic timings so the next build can skip the autotuning search."""
    if path is None:
        return
    cache = config.get_timing_cache()
    if cache is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(memoryview(cache.serialize()))
