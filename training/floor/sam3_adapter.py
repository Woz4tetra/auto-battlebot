#!/usr/bin/env python3
"""SAM3 adapter for floor segmentation pipeline.

Uses build_sam3_predictor + the Sam3BasePredictor direct API for point-prompt
segmentation and mask propagation.  For single-frame seeding, the target frame
is extracted to a temp directory so we don't load thousands of frames into VRAM.

The handle_request / handle_stream_request high-level API is designed around
text prompts; point-prompt workflows are better served by the base predictor.

Set TORCHDYNAMO_DISABLE=1 to skip torch.compile (avoids multi-minute warm-up).
"""

from __future__ import annotations

import importlib
import logging
import os
import shutil
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

LOGGER = logging.getLogger("floor_pipeline")

MAX_PROPAGATION_FRAMES = 4000
MAX_POINTS = 16


def _disable_torch_compile_if_requested() -> None:
    if os.environ.get("TORCHDYNAMO_DISABLE", "") == "1":
        try:
            import torch._dynamo

            torch._dynamo.config.disable = True
            LOGGER.info("torch.compile disabled via TORCHDYNAMO_DISABLE=1")
        except Exception:  # noqa: BLE001
            pass


_disable_torch_compile_if_requested()


def _gpu_supports_fa3() -> bool:
    try:
        import torch

        if torch.cuda.is_available():
            return torch.cuda.get_device_capability(0)[0] >= 9
    except Exception:  # noqa: BLE001
        pass
    return False


def _inference_ctx():
    try:
        import torch

        return torch.inference_mode()
    except Exception:  # noqa: BLE001
        from contextlib import nullcontext

        return nullcontext()


def _free_gpu_cache() -> None:
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:  # noqa: BLE001
        pass


def _to_numpy(x) -> np.ndarray:
    if isinstance(x, np.ndarray):
        return x
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x)


# ---------------------------------------------------------------------------
# Mask extraction — handles multiple SAM3 output formats
# ---------------------------------------------------------------------------


_EXTRACT_LOG_N = 0


def _extract_mask(item: dict | None, obj_id: int = 1) -> np.ndarray | None:
    """Pull a binary uint8 mask from a SAM3 response dict."""
    global _EXTRACT_LOG_N  # noqa: PLW0603
    if item is None:
        return None

    outputs = item.get("outputs", item) if isinstance(item, dict) else item
    if outputs is None:
        return None

    if _EXTRACT_LOG_N < 3:
        _log_output_structure(outputs)
        _EXTRACT_LOG_N += 1

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
    if not isinstance(outputs, dict):
        return outputs if hasattr(outputs, "shape") else None

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


# ---------------------------------------------------------------------------
# Session helpers
# ---------------------------------------------------------------------------


def _start_session(predictor, video_path: str) -> str:
    import inspect

    sig = inspect.signature(predictor.start_session)
    kwargs: dict = {"offload_video_to_cpu": True}
    if "offload_state_to_cpu" in sig.parameters:
        kwargs["offload_state_to_cpu"] = True

    result = predictor.start_session(video_path, **kwargs)
    sid = result["session_id"]

    if "offload_state_to_cpu" not in kwargs:
        _patch_offload_state(predictor, sid)

    return sid


def _patch_offload_state(predictor, session_id: str) -> None:
    states = getattr(predictor, "_all_inference_states", None)
    if states is None:
        return
    inf_state = states.get(session_id)
    if inf_state is None:
        return
    if isinstance(inf_state, dict) and "offload_state_to_cpu" in inf_state:
        inf_state["offload_state_to_cpu"] = True
        LOGGER.debug("Patched offload_state_to_cpu=True")


def _invoke_add_prompt_text(
    predictor,
    session_id: str,
    frame_inner_idx: int,
    text: str,
    obj_id: int,
):
    """Call predictor with a text-only prompt (add_prompt or handle_request)."""
    import inspect

    sig = inspect.signature(predictor.add_prompt)
    if "text" in sig.parameters:
        kwargs: dict = {"session_id": session_id, "obj_id": obj_id, "text": text}
        if "frame_idx" in sig.parameters:
            kwargs["frame_idx"] = frame_inner_idx
        elif "frame_index" in sig.parameters:
            kwargs["frame_index"] = frame_inner_idx
        else:
            kwargs["frame_idx"] = frame_inner_idx
        return predictor.add_prompt(**kwargs)

    handle_request = getattr(predictor, "handle_request", None)
    if callable(handle_request):
        req = {
            "type": "add_prompt",
            "session_id": session_id,
            "frame_index": frame_inner_idx,
            "text": text,
            "obj_id": obj_id,
        }
        return handle_request(req)

    raise RuntimeError(
        "SAM3 predictor has no `text` parameter on add_prompt and no handle_request; "
        "text prompts are not supported."
    )


def _clamp_points(
    points: np.ndarray,
    labels: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Ensure total points ≤ MAX_POINTS.  Keep balanced pos/neg."""
    if len(points) <= MAX_POINTS:
        return points, labels
    pos_mask = labels == 1
    neg_mask = labels == 0
    pos_pts, neg_pts = points[pos_mask], points[neg_mask]
    pos_lbl, neg_lbl = labels[pos_mask], labels[neg_mask]
    half = MAX_POINTS // 2
    n_pos = min(len(pos_pts), half)
    n_neg = min(len(neg_pts), MAX_POINTS - n_pos)
    pts = np.concatenate([pos_pts[:n_pos], neg_pts[:n_neg]], axis=0)
    lbl = np.concatenate([pos_lbl[:n_pos], neg_lbl[:n_neg]], axis=0)
    LOGGER.debug(
        "Clamped %d points to %d (%d pos, %d neg)", len(points), len(pts), n_pos, n_neg
    )
    return pts, lbl


# ---------------------------------------------------------------------------
# Single-frame extraction for seed generation
# ---------------------------------------------------------------------------


def _extract_frame_to_jpeg_dir(video_path: str | Path, frame_idx: int) -> str:
    """Extract one frame from a video into a temp JPEG folder.

    Returns the temp directory path.  Caller must clean it up.
    SAM3 start_session accepts a JPEG folder as a "video".
    """
    tmpdir = tempfile.mkdtemp(prefix="sam3_frame_")
    cap = cv2.VideoCapture(str(video_path))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        shutil.rmtree(tmpdir, ignore_errors=True)
        raise RuntimeError(f"Could not read frame {frame_idx} from {video_path}")
    out_path = os.path.join(tmpdir, "00000.jpg")
    cv2.imwrite(out_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    return tmpdir


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------


class NativeSam3Adapter:
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
            raise RuntimeError("sam3 module does not expose build_sam3_predictor.")
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
        """Get a mask for a single frame.

        Extracts just the target frame to a temp JPEG folder so we don't
        load the entire video into VRAM.
        """
        vname = Path(video_path).name
        points, point_labels = _clamp_points(points, point_labels)

        LOGGER.info("segment_frame: extracting frame %d from %s", frame_idx, vname)
        tmpdir = _extract_frame_to_jpeg_dir(video_path, frame_idx)
        try:
            t0 = time.monotonic()
            sid = _start_session(self._predictor, tmpdir)
            LOGGER.info(
                "segment_frame: session started (%.1fs, 1 frame)", time.monotonic() - t0
            )

            try:
                t1 = time.monotonic()
                with _inference_ctx():
                    result = self._predictor.add_prompt(
                        session_id=sid,
                        frame_idx=0,
                        points=points.tolist(),
                        point_labels=point_labels.tolist(),
                        obj_id=obj_id,
                    )
                LOGGER.info("segment_frame: add_prompt (%.1fs)", time.monotonic() - t1)

                mask = _extract_mask(result, obj_id)
                if mask is not None:
                    pct = 100.0 * mask.sum() / max(mask.size, 1)
                    LOGGER.info(
                        "segment_frame: mask from add_prompt (%.1f%% coverage)", pct
                    )
                    return mask

                LOGGER.info("segment_frame: no mask from add_prompt, trying propagate")
                with _inference_ctx():
                    for item in self._predictor.propagate_in_video(
                        session_id=sid,
                        start_frame_idx=0,
                        max_frame_num_to_track=1,
                    ):
                        mask = _extract_mask(item, obj_id)
                        if mask is not None:
                            LOGGER.info("segment_frame: mask from propagate fallback")
                            return mask

                LOGGER.warning(
                    "segment_frame: no mask produced for %s frame %d", vname, frame_idx
                )
            finally:
                self._predictor.close_session(sid)
                _free_gpu_cache()
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)
        return None

    def segment_frame_text(
        self,
        video_path: str | Path,
        frame_idx: int,
        text: str,
        obj_id: int = 1,
    ) -> np.ndarray | None:
        """Get a mask for a single frame using a SAM3 text prompt only."""
        vname = Path(video_path).name
        prompt = (text or "").strip()
        if not prompt:
            raise ValueError("text prompt must be non-empty")

        LOGGER.info(
            "segment_frame_text: extracting frame %d from %s (prompt=%r)",
            frame_idx,
            vname,
            prompt,
        )
        tmpdir = _extract_frame_to_jpeg_dir(video_path, frame_idx)
        try:
            t0 = time.monotonic()
            sid = _start_session(self._predictor, tmpdir)
            LOGGER.info(
                "segment_frame_text: session started (%.1fs, 1 frame)",
                time.monotonic() - t0,
            )

            try:
                t1 = time.monotonic()
                with _inference_ctx():
                    result = _invoke_add_prompt_text(
                        self._predictor, sid, 0, prompt, obj_id
                    )
                LOGGER.info(
                    "segment_frame_text: add_prompt (%.1fs)", time.monotonic() - t1
                )

                mask = _extract_mask(result, obj_id)
                if mask is not None:
                    pct = 100.0 * mask.sum() / max(mask.size, 1)
                    LOGGER.info(
                        "segment_frame_text: mask from add_prompt (%.1f%% coverage)",
                        pct,
                    )
                    return mask

                LOGGER.info(
                    "segment_frame_text: no mask from add_prompt, trying propagate"
                )
                with _inference_ctx():
                    for item in self._predictor.propagate_in_video(
                        session_id=sid,
                        start_frame_idx=0,
                        max_frame_num_to_track=1,
                    ):
                        mask = _extract_mask(item, obj_id)
                        if mask is not None:
                            LOGGER.info(
                                "segment_frame_text: mask from propagate fallback"
                            )
                            return mask

                LOGGER.warning(
                    "segment_frame_text: no mask produced for %s frame %d",
                    vname,
                    frame_idx,
                )
            finally:
                self._predictor.close_session(sid)
                _free_gpu_cache()
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)
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
        """Propagate a point prompt across the video.

        Forward and backward are run in separate sessions to limit memory.
        Each direction is capped at MAX_PROPAGATION_FRAMES.
        """
        vname = Path(video_path).name
        points, point_labels = _clamp_points(points, point_labels)
        cap = min(max_frames or MAX_PROPAGATION_FRAMES, MAX_PROPAGATION_FRAMES)
        outputs: dict[int, np.ndarray] = {}
        t_total = time.monotonic()

        for direction in ("forward", "backward"):
            if direction == "backward" and prompt_frame_idx == 0:
                continue
            track_limit = cap if direction == "forward" else min(cap, prompt_frame_idx)

            LOGGER.info(
                "propagate_video [%s]: %s from frame %d, up to %d frames",
                direction,
                vname,
                prompt_frame_idx,
                track_limit,
            )
            partial = self._propagate_direction(
                video_path,
                prompt_frame_idx,
                points,
                point_labels,
                obj_id,
                track_limit,
                direction,
            )
            outputs.update(partial)

        elapsed = time.monotonic() - t_total
        LOGGER.info(
            "propagate_video: %d masks in %.1fs (%.1f fps) for %s",
            len(outputs),
            elapsed,
            len(outputs) / max(elapsed, 0.001),
            vname,
        )
        return outputs

    def _propagate_direction(
        self,
        video_path: str | Path,
        prompt_frame_idx: int,
        points: np.ndarray,
        point_labels: np.ndarray,
        obj_id: int,
        max_track: int,
        direction: str,
    ) -> dict[int, np.ndarray]:
        sid = _start_session(self._predictor, str(video_path))
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
            try:
                prop_kwargs: dict = {
                    "session_id": sid,
                    "start_frame_idx": prompt_frame_idx,
                    "max_frame_num_to_track": max_track,
                }
                import inspect

                sig = inspect.signature(self._predictor.propagate_in_video)
                if "propagation_direction" in sig.parameters:
                    prop_kwargs["propagation_direction"] = direction

                with _inference_ctx():
                    for item in self._predictor.propagate_in_video(**prop_kwargs):
                        fidx = item.get("frame_index")
                        if fidx is None:
                            continue
                        mask = _extract_mask(item, obj_id)
                        if mask is not None:
                            outputs[int(fidx)] = mask
            except RuntimeError as exc:
                if "out of memory" in str(exc).lower():
                    LOGGER.warning(
                        "propagate_%s: OOM after %d masks — keeping partial",
                        direction,
                        len(outputs),
                    )
                else:
                    raise

            LOGGER.info("propagate_%s: collected %d masks", direction, len(outputs))
            return outputs
        finally:
            self._predictor.close_session(sid)
            _free_gpu_cache()


def build_adapter(
    adapter_module: str = "sam3_adapter",
    checkpoint: str = "",
    device: str = "cuda",
    amp: bool = True,
    model_cfg: str = "",
) -> NativeSam3Adapter:
    if adapter_module in {"training.floor.sam3_adapter", "sam3_adapter"}:
        return NativeSam3Adapter(
            checkpoint=checkpoint,
            device=device,
            amp=amp,
            model_cfg=model_cfg,
        )
    mod = importlib.import_module(adapter_module)
    builder = getattr(mod, "build_adapter", None)
    if builder is None:
        raise RuntimeError(
            f"Custom adapter module `{adapter_module}` missing `build_adapter`."
        )
    return builder(checkpoint=checkpoint, device=device, amp=amp, model_cfg=model_cfg)
