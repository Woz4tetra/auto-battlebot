#!/usr/bin/env python3
"""SAM3 adapter — thin wrapper around the known SAM3 base-predictor API.

Targets the concrete API exposed by the installed SAM3 fork at
training/floor/third_party/sam3 (Sam3BasePredictor):

    build_sam3_predictor(version, use_fa3, ...)
    predictor.start_session(resource_path, ...)  -> {"session_id": str}
    predictor.add_prompt(session_id, frame_idx, points, point_labels, ...)
    predictor.propagate_in_video(session_id, ...)  -> yields dicts
    predictor.close_session(session_id)
"""

from __future__ import annotations

import importlib
import logging
from pathlib import Path

import numpy as np

LOGGER = logging.getLogger("floor_pipeline")


def _gpu_supports_fa3() -> bool:
    """Flash Attention 3 requires Hopper (sm_90+). A6000 is Ampere (sm_86)."""
    try:
        import torch

        if torch.cuda.is_available():
            return torch.cuda.get_device_capability(0)[0] >= 9
    except Exception:  # noqa: BLE001
        pass
    return False


def _inference_ctx():
    """Return a torch.inference_mode() context (or nullcontext if unavailable)."""
    try:
        import torch

        return torch.inference_mode()
    except Exception:  # noqa: BLE001
        from contextlib import nullcontext

        return nullcontext()


def _to_numpy(x) -> np.ndarray:
    if isinstance(x, np.ndarray):
        return x
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x)


def _extract_mask(item: dict | None) -> np.ndarray | None:
    """Pull a binary uint8 mask from a SAM3 response dict.

    SAM3 returns: {"frame_index": int, "outputs": {"out_binary_masks": tensor}}
    """
    if item is None:
        return None
    outputs = item.get("outputs", item) if isinstance(item, dict) else None
    if outputs is None:
        return None
    masks = outputs.get("out_binary_masks")
    if masks is None:
        return None
    arr = _to_numpy(masks)
    if arr.size == 0:
        return None
    if arr.ndim == 3:
        arr = arr[0]
    return (arr > 0).astype(np.uint8)


class NativeSam3Adapter:
    """Concrete adapter for the installed SAM3 fork."""

    def __init__(
        self,
        checkpoint: str = "",
        device: str = "cuda",
        amp: bool = True,
        model_cfg: str = "",
    ) -> None:
        sam3 = importlib.import_module("sam3")
        builder = getattr(sam3, "build_sam3_predictor", None)
        if not callable(builder):
            raise RuntimeError(
                "sam3 module does not expose build_sam3_predictor. "
                "Check your SAM3 installation."
            )
        self._predictor = builder(
            checkpoint_path=checkpoint or None,
            version="sam3.1",
            use_fa3=_gpu_supports_fa3(),
        )

    def segment_frame(
        self,
        video_path: str | Path,
        frame_idx: int,
        points: np.ndarray,
        point_labels: np.ndarray,
        obj_id: int = 1,
    ) -> np.ndarray | None:
        """Get a mask for a single frame via add_prompt (+ 1-frame propagate fallback)."""
        sid = self._predictor.start_session(
            str(video_path), offload_video_to_cpu=True
        )["session_id"]
        try:
            with _inference_ctx():
                result = self._predictor.add_prompt(
                    session_id=sid,
                    frame_idx=frame_idx,
                    points=points.tolist(),
                    point_labels=point_labels.tolist(),
                    obj_id=obj_id,
                )
            mask = _extract_mask(result)
            if mask is not None:
                return mask

            LOGGER.debug("add_prompt returned empty mask; falling back to propagate")
            with _inference_ctx():
                for item in self._predictor.propagate_in_video(
                    session_id=sid,
                    start_frame_idx=frame_idx,
                    max_frame_num_to_track=1,
                ):
                    mask = _extract_mask(item)
                    if mask is not None:
                        return mask
        finally:
            self._predictor.close_session(sid)
            _free_gpu_cache()
        return None

    def propagate_video(
        self,
        video_path: str | Path,
        prompt_frame_idx: int,
        points: np.ndarray,
        point_labels: np.ndarray,
        start_frame_idx: int | None = None,
        max_frames: int | None = None,
        obj_id: int = 1,
    ) -> dict[int, np.ndarray]:
        """Add prompt on one frame, propagate to all frames, return {frame_idx: mask}."""
        sid = self._predictor.start_session(
            str(video_path), offload_video_to_cpu=True
        )["session_id"]
        try:
            with _inference_ctx():
                self._predictor.add_prompt(
                    session_id=sid,
                    frame_idx=prompt_frame_idx,
                    points=points.tolist(),
                    point_labels=point_labels.tolist(),
                    obj_id=obj_id,
                )
            outputs: dict[int, np.ndarray] = {}
            with _inference_ctx():
                for item in self._predictor.propagate_in_video(
                    session_id=sid,
                    start_frame_idx=start_frame_idx,
                    max_frame_num_to_track=max_frames,
                ):
                    fidx = item.get("frame_index")
                    if fidx is None:
                        continue
                    mask = _extract_mask(item)
                    if mask is not None:
                        outputs[int(fidx)] = mask
            return outputs
        finally:
            self._predictor.close_session(sid)
            _free_gpu_cache()


def _free_gpu_cache() -> None:
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:  # noqa: BLE001
        pass


def build_adapter(
    adapter_module: str = "sam3_adapter",
    checkpoint: str = "",
    device: str = "cuda",
    amp: bool = True,
    model_cfg: str = "",
) -> NativeSam3Adapter:
    if adapter_module in {"training.floor.sam3_adapter", "sam3_adapter"}:
        return NativeSam3Adapter(
            checkpoint=checkpoint, device=device, amp=amp, model_cfg=model_cfg
        )
    mod = importlib.import_module(adapter_module)
    builder = getattr(mod, "build_adapter", None)
    if builder is None:
        raise RuntimeError(
            f"Custom adapter module `{adapter_module}` missing `build_adapter`."
        )
    return builder(checkpoint=checkpoint, device=device, amp=amp, model_cfg=model_cfg)
