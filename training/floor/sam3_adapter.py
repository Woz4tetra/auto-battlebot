#!/usr/bin/env python3
"""SAM3 adapter abstraction.

This file defines the adapter contract used by the floor pipeline. It is designed
to keep the project strict-SAM3 while allowing you to swap in the exact SAM3
implementation/API available in your environment.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np


@dataclass
class SegmentationSeed:
    """Seed inputs for floor segmentation."""

    frame_bgr: np.ndarray
    prompt_tags: list[str]
    negative_tags: list[str]
    text_threshold: float
    box_threshold: float
    positive_points_xy: list[tuple[int, int]]
    negative_points_xy: list[tuple[int, int]]
    arena_box_xyxy: tuple[int, int, int, int] | None


class Sam3AdapterProtocol(Protocol):
    """Required adapter API for this pipeline."""

    def segment_seed(self, seed: SegmentationSeed) -> np.ndarray:
        """Return initial floor mask (uint8, 0/1) for the seed frame."""

    def propagate_video(
        self,
        video_path: Path,
        initial_mask: np.ndarray,
        start_frame_idx: int,
        end_frame_idx: int,
        async_prefetch: int,
        disable_cache: bool,
    ) -> dict[int, np.ndarray]:
        """Return frame_idx -> mask (uint8, 0/1) within inclusive frame range."""


class NativeSam3Adapter:
    """Adapter for a user-installed SAM3 package.

    The exact SAM3 API differs between distributions. This adapter deliberately
    checks for a known callable surface:
      - module must expose `build_floor_segmenter(checkpoint, device, amp)`
      - returned object must implement:
          - segment_seed(seed: SegmentationSeed) -> np.ndarray
          - propagate_video(...)
    """

    def __init__(self, checkpoint: str, device: str, amp: bool) -> None:
        try:
            sam3 = importlib.import_module("sam3")
        except ModuleNotFoundError as exc:  # pragma: no cover
            raise RuntimeError(
                "SAM3 package not found. Install SAM3 with "
                "`install/install_floor_sam3_environment.sh --sam3-path ...` "
                "or provide a custom adapter module in pipeline.toml."
            ) from exc

        builder = getattr(sam3, "build_floor_segmenter", None)
        if builder is None:
            raise RuntimeError(
                "Installed `sam3` module does not expose `build_floor_segmenter`. "
                "Provide a custom adapter module with build_adapter(checkpoint, device, amp)."
            )
        self._impl = builder(checkpoint=checkpoint, device=device, amp=amp)
        for required in ("segment_seed", "propagate_video"):
            if not hasattr(self._impl, required):
                raise RuntimeError(f"SAM3 segmenter missing required method: {required}")

    def segment_seed(self, seed: SegmentationSeed) -> np.ndarray:
        mask = self._impl.segment_seed(seed)
        return np.asarray(mask, dtype=np.uint8)

    def propagate_video(
        self,
        video_path: Path,
        initial_mask: np.ndarray,
        start_frame_idx: int,
        end_frame_idx: int,
        async_prefetch: int,
        disable_cache: bool,
    ) -> dict[int, np.ndarray]:
        outputs = self._impl.propagate_video(
            video_path=video_path,
            initial_mask=initial_mask,
            start_frame_idx=start_frame_idx,
            end_frame_idx=end_frame_idx,
            async_prefetch=async_prefetch,
            disable_cache=disable_cache,
        )
        return {int(k): np.asarray(v, dtype=np.uint8) for k, v in outputs.items()}


def build_adapter(adapter_module: str, checkpoint: str, device: str, amp: bool) -> Sam3AdapterProtocol:
    """Build adapter from module path.

    If adapter_module is this file, it uses `NativeSam3Adapter`.
    Otherwise it imports adapter_module and calls:
      build_adapter(checkpoint: str, device: str, amp: bool) -> Sam3AdapterProtocol
    """
    if adapter_module in {"training.floor.sam3_adapter", "sam3_adapter"}:
        return NativeSam3Adapter(checkpoint=checkpoint, device=device, amp=amp)

    mod = importlib.import_module(adapter_module)
    builder = getattr(mod, "build_adapter", None)
    if builder is None:
        raise RuntimeError(
            f"Custom adapter module `{adapter_module}` is missing `build_adapter`."
        )
    adapter = builder(checkpoint=checkpoint, device=device, amp=amp)
    for required in ("segment_seed", "propagate_video"):
        if not hasattr(adapter, required):
            raise RuntimeError(
                f"Custom adapter `{adapter_module}` missing required method `{required}`"
            )
    return adapter
