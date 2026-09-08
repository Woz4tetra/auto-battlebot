"""TensorRT builder scaffolding shared by the YOLO and DeepLab export CLIs.

Both converters name engines the same way and reuse the same tactic timing cache, so the
platform tag and cache handling live here rather than being copied into each. The
conversion CLIs are `training/yolo/convert_to_tensorrt.py` and
`training/deeplab/convert_to_tensorrt.py`.
"""

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
    """Append platform + GPU compute capability tag so incompatible engines are distinct.

    Produces filenames like ``model_x86_64_sm89.engine`` — the ``sm`` tag
    prevents silently loading an engine built for a different GPU architecture.
    A precision tag goes ahead of it (``model_int8_x86_64_sm89.engine``) so the trailing
    ``_<arch>_sm<XX>.engine`` shape the config candidate lists match on is unchanged, and
    so FP16 filenames stay byte-identical to what ``config/_jetson.toml`` already names.
    """
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
    # Last writer wins. Concurrent builders may race here; the cache is advisory, and a lost
    # update only costs the next build some re-timing.
    path.write_bytes(memoryview(cache.serialize()))
