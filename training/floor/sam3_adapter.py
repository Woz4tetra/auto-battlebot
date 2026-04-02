#!/usr/bin/env python3
"""SAM3 adapter — uses the official handle_request / handle_stream_request API.

Reference: https://github.com/facebookresearch/sam3
See examples/sam3_video_predictor_example.ipynb for the canonical usage pattern.

Set TORCHDYNAMO_DISABLE=1 to skip torch.compile (avoids multi-minute warm-up).
"""

from __future__ import annotations

import importlib
import logging
import os
import time
from pathlib import Path

import numpy as np
import torch

LOGGER = logging.getLogger("floor_pipeline")

# OOM consistently around frame ~5000 on a single A6000 (48 GB).
MAX_PROPAGATION_FRAMES = 4000


def _disable_torch_compile_if_requested() -> None:
    if os.environ.get("TORCHDYNAMO_DISABLE", "") == "1":
        try:
            import torch._dynamo

            torch._dynamo.config.disable = True
            LOGGER.info("torch.compile disabled via TORCHDYNAMO_DISABLE=1")
        except Exception:  # noqa: BLE001
            pass


_disable_torch_compile_if_requested()


def _to_numpy(x) -> np.ndarray:
    if isinstance(x, np.ndarray):
        return x
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x)


_OUTPUT_LOGGED = False


def _extract_mask(outputs, obj_id: int = 1) -> np.ndarray | None:
    """Extract a binary uint8 mask from a SAM3 outputs dict.

    The official API returns outputs that go through prepare_masks_for_visualization.
    Depending on the code path, masks may appear under various keys or as
    per-object-id tensors.
    """
    global _OUTPUT_LOGGED  # noqa: PLW0603
    if outputs is None:
        return None

    if not _OUTPUT_LOGGED:
        _log_output_structure(outputs)
        _OUTPUT_LOGGED = True

    tensor = _find_mask_tensor(outputs, obj_id)
    if tensor is None:
        return None

    arr = _to_numpy(tensor)
    if arr.size == 0:
        return None
    while arr.ndim > 2:
        arr = arr[0]
    return (arr > 0).astype(np.uint8)


def _find_mask_tensor(outputs, obj_id: int = 1):
    """Search for a mask tensor in the outputs, trying multiple formats."""
    if not isinstance(outputs, dict):
        if hasattr(outputs, "shape"):
            return outputs
        return None

    for key in ("out_binary_masks", "pred_masks", "masks", "video_res_masks"):
        val = outputs.get(key)
        if val is not None and hasattr(val, "shape"):
            return val

    if obj_id in outputs:
        val = outputs[obj_id]
        if hasattr(val, "shape"):
            return val

    for val in outputs.values():
        if hasattr(val, "shape"):
            return val

    return None


def _log_output_structure(outputs) -> None:
    if isinstance(outputs, dict):
        summary = {
            k: (f"tensor{tuple(v.shape)}" if hasattr(v, "shape") else type(v).__name__)
            for k, v in outputs.items()
        }
        LOGGER.info("SAM3 output keys: %s", summary)
    elif hasattr(outputs, "shape"):
        LOGGER.info("SAM3 output: tensor%s", tuple(outputs.shape))
    else:
        LOGGER.info("SAM3 output type: %s", type(outputs).__name__)


def _free_gpu_cache() -> None:
    try:
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:  # noqa: BLE001
        pass


def _abs_to_rel(points: np.ndarray, w: int, h: int) -> torch.Tensor:
    """Convert absolute pixel coordinates to relative (0-1) as the official API requires."""
    rel = points.copy().astype(np.float32)
    rel[:, 0] /= w
    rel[:, 1] /= h
    return torch.tensor(rel, dtype=torch.float32)


class NativeSam3Adapter:
    """Adapter using the official SAM3 handle_request / handle_stream_request API."""

    def __init__(
        self,
        checkpoint: str = "",
        device: str = "cuda",
        amp: bool = True,
        model_cfg: str = "",
    ) -> None:
        model_builder = importlib.import_module("sam3.model_builder")
        builder = getattr(model_builder, "build_sam3_video_predictor", None)
        if not callable(builder):
            raise RuntimeError(
                "sam3.model_builder does not expose build_sam3_video_predictor."
            )

        gpu_id = 0
        if device.startswith("cuda:"):
            try:
                gpu_id = int(device.split(":")[1])
            except (IndexError, ValueError):
                pass

        LOGGER.info("Building SAM3 video predictor on GPU %d", gpu_id)
        self._predictor = builder(gpus_to_use=[gpu_id])
        self._frame_hw: tuple[int, int] | None = None

    def shutdown(self) -> None:
        if hasattr(self._predictor, "shutdown"):
            self._predictor.shutdown()

    def _start_session(self, video_path: str) -> str:
        response = self._predictor.handle_request(
            request=dict(type="start_session", resource_path=video_path)
        )
        return response["session_id"]

    def _close_session(self, session_id: str) -> None:
        self._predictor.handle_request(
            request=dict(type="close_session", session_id=session_id)
        )

    def _add_prompt(
        self,
        session_id: str,
        frame_idx: int,
        points: np.ndarray,
        point_labels: np.ndarray,
        w: int,
        h: int,
        obj_id: int = 1,
    ) -> dict:
        points_rel = _abs_to_rel(points, w, h)
        labels_tensor = torch.tensor(point_labels, dtype=torch.int32)
        response = self._predictor.handle_request(
            request=dict(
                type="add_prompt",
                session_id=session_id,
                frame_index=frame_idx,
                points=points_rel,
                point_labels=labels_tensor,
                obj_id=obj_id,
            )
        )
        return response

    def _get_video_dims(self, video_path: str | Path) -> tuple[int, int]:
        """Return (width, height) of the video."""
        import cv2

        cap = cv2.VideoCapture(str(video_path))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        return w, h

    def segment_frame(
        self,
        video_path: str | Path,
        frame_idx: int,
        points: np.ndarray,
        point_labels: np.ndarray,
        obj_id: int = 1,
    ) -> np.ndarray | None:
        """Get a mask for a single frame."""
        vname = Path(video_path).name
        w, h = self._get_video_dims(video_path)

        LOGGER.info("segment_frame: opening session for %s", vname)
        t0 = time.monotonic()
        sid = self._start_session(str(video_path))
        LOGGER.info("segment_frame: session started (%.1fs)", time.monotonic() - t0)

        try:
            LOGGER.info(
                "segment_frame: add_prompt on frame %d (%d pos, %d neg points)",
                frame_idx,
                int((point_labels == 1).sum()),
                int((point_labels == 0).sum()),
            )
            t1 = time.monotonic()
            response = self._add_prompt(sid, frame_idx, points, point_labels, w, h, obj_id)
            LOGGER.info("segment_frame: add_prompt returned (%.1fs)", time.monotonic() - t1)

            mask = _extract_mask(response.get("outputs"), obj_id)
            if mask is not None:
                pct = 100.0 * mask.sum() / max(mask.size, 1)
                LOGGER.info("segment_frame: got mask (%.1f%% coverage)", pct)
                return mask

            LOGGER.info("segment_frame: add_prompt returned no mask; trying propagation")
            t2 = time.monotonic()
            for resp in self._predictor.handle_stream_request(
                request=dict(
                    type="propagate_in_video",
                    session_id=sid,
                )
            ):
                fidx = resp.get("frame_index")
                if fidx is not None and fidx == frame_idx:
                    mask = _extract_mask(resp.get("outputs"), obj_id)
                    if mask is not None:
                        LOGGER.info(
                            "segment_frame: propagation produced mask (%.1fs)",
                            time.monotonic() - t2,
                        )
                        return mask
            LOGGER.warning("segment_frame: no mask produced for frame %d", frame_idx)
        finally:
            self._close_session(sid)
            _free_gpu_cache()
        return None

    def propagate_video(
        self,
        video_path: str | Path,
        prompt_frame_idx: int,
        points: np.ndarray,
        point_labels: np.ndarray,
        max_frames: int | None = None,
        obj_id: int = 1,
    ) -> dict[int, np.ndarray]:
        """Add prompt on one frame, propagate to all frames, return {frame_idx: mask}."""
        vname = Path(video_path).name
        w, h = self._get_video_dims(video_path)
        outputs: dict[int, np.ndarray] = {}
        t_total = time.monotonic()

        LOGGER.info("propagate_video: opening session for %s", vname)
        t0 = time.monotonic()
        sid = self._start_session(str(video_path))
        LOGGER.info("propagate_video: session started (%.1fs)", time.monotonic() - t0)

        try:
            LOGGER.info("propagate_video: add_prompt on frame %d", prompt_frame_idx)
            t1 = time.monotonic()
            self._add_prompt(sid, prompt_frame_idx, points, point_labels, w, h, obj_id)
            LOGGER.info("propagate_video: add_prompt done (%.1fs)", time.monotonic() - t1)

            cap = max_frames if max_frames is not None else MAX_PROPAGATION_FRAMES
            cap = min(cap, MAX_PROPAGATION_FRAMES)

            LOGGER.info(
                "propagate_video: propagating from frame %d (cap %d frames)",
                prompt_frame_idx, cap,
            )
            t2 = time.monotonic()
            n_yielded = 0
            try:
                for resp in self._predictor.handle_stream_request(
                    request=dict(
                        type="propagate_in_video",
                        session_id=sid,
                    )
                ):
                    fidx = resp.get("frame_index")
                    if fidx is None:
                        continue
                    mask = _extract_mask(resp.get("outputs"), obj_id)
                    if mask is not None:
                        outputs[int(fidx)] = mask
                    n_yielded += 1
                    if n_yielded >= cap:
                        LOGGER.info(
                            "propagate_video: reached frame cap (%d), stopping", cap,
                        )
                        break
            except RuntimeError as exc:
                if "out of memory" in str(exc).lower():
                    LOGGER.warning(
                        "propagate_video: OOM after %d masks — keeping partial for %s",
                        len(outputs), vname,
                    )
                else:
                    raise

            elapsed = time.monotonic() - t2
            LOGGER.info(
                "propagate_video: %d masks in %.1fs (%.1f fps) for %s",
                len(outputs), elapsed,
                len(outputs) / max(elapsed, 0.001), vname,
            )
        finally:
            self._close_session(sid)
            _free_gpu_cache()

        total_elapsed = time.monotonic() - t_total
        LOGGER.info("propagate_video: total %.1fs for %s", total_elapsed, vname)
        return outputs


def build_adapter(
    adapter_module: str = "sam3_adapter",
    checkpoint: str = "",
    device: str = "cuda",
    amp: bool = True,
    model_cfg: str = "",
) -> NativeSam3Adapter:
    if adapter_module in {"training.floor.sam3_adapter", "sam3_adapter"}:
        return NativeSam3Adapter(
            checkpoint=checkpoint, device=device, amp=amp, model_cfg=model_cfg,
        )
    mod = importlib.import_module(adapter_module)
    builder = getattr(mod, "build_adapter", None)
    if builder is None:
        raise RuntimeError(
            f"Custom adapter module `{adapter_module}` missing `build_adapter`."
        )
    return builder(checkpoint=checkpoint, device=device, amp=amp, model_cfg=model_cfg)
