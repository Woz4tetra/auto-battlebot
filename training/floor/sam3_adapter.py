#!/usr/bin/env python3
"""SAM3 adapter — thin wrapper around the known SAM3 base-predictor API.

Targets the concrete API exposed by the installed SAM3 fork at
training/floor/third_party/sam3 (Sam3BasePredictor):

    build_sam3_predictor(version, use_fa3, ...)
    predictor.start_session(resource_path, ...)  -> {"session_id": str}
    predictor.add_prompt(session_id, frame_idx, points, point_labels, ...)
    predictor.propagate_in_video(session_id, ...)  -> yields dicts
    predictor.close_session(session_id)

Set TORCHDYNAMO_DISABLE=1 to skip torch.compile (avoids multi-minute warm-up).
"""

from __future__ import annotations

import importlib
import logging
import os
import time
from pathlib import Path

import numpy as np

LOGGER = logging.getLogger("floor_pipeline")


def _disable_torch_compile_if_requested() -> None:
    """Honour TORCHDYNAMO_DISABLE=1 to bypass torch.compile warm-up."""
    if os.environ.get("TORCHDYNAMO_DISABLE", "") == "1":
        try:
            import torch._dynamo  # noqa: F401

            torch._dynamo.config.disable = True
            LOGGER.info("torch.compile disabled via TORCHDYNAMO_DISABLE=1")
        except Exception:  # noqa: BLE001
            pass


_disable_torch_compile_if_requested()


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


_PROPAGATE_OUTPUT_LOGGED = False


def _extract_mask(item: dict | None, obj_id: int = 1) -> np.ndarray | None:
    """Pull a binary uint8 mask from a SAM3 response dict.

    Handles multiple output formats:
      - add_prompt style:  {"outputs": {"out_binary_masks": tensor}}
      - propagate style 1: {"outputs": {obj_id: tensor}}   (per-object masks)
      - propagate style 2: {"outputs": tensor}              (single-object)
      - propagate style 3: {"outputs": {"pred_masks": tensor}}
    """
    global _PROPAGATE_OUTPUT_LOGGED  # noqa: PLW0603
    if item is None:
        return None

    outputs = item.get("outputs", item) if isinstance(item, dict) else item
    if outputs is None:
        return None

    if not _PROPAGATE_OUTPUT_LOGGED:
        _log_output_structure(outputs)
        _PROPAGATE_OUTPUT_LOGGED = True

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
    """Log the structure of the first propagation output for debugging."""
    if isinstance(outputs, dict):
        summary = {
            k: (f"tensor{tuple(v.shape)}" if hasattr(v, "shape") else type(v).__name__)
            for k, v in outputs.items()
        }
        LOGGER.info("propagation output keys: %s", summary)
    elif hasattr(outputs, "shape"):
        LOGGER.info("propagation output: tensor%s", tuple(outputs.shape))
    else:
        LOGGER.info("propagation output type: %s", type(outputs).__name__)


def _start_session(predictor, video_path: str) -> str:
    """Open a SAM3 session with both video and state offloaded to CPU.

    ``offload_video_to_cpu``  keeps decoded frames on CPU (saves ~1-2 GB).
    ``offload_state_to_cpu``  keeps the per-frame tracking memory bank on CPU
    and only moves the active frame's state to GPU.  Without this the memory
    bank grows ~8-9 MB per frame and overflows the A6000's 48 GB around
    frame 5000.

    The flag is attempted as a kwarg first; if the SAM3 fork's start_session
    doesn't expose it we patch the inference_state dict directly (SAM2/SAM3
    stores it there and checks it during propagation).
    """
    import inspect

    sig = inspect.signature(predictor.start_session)
    accepts_offload = "offload_state_to_cpu" in sig.parameters

    if accepts_offload:
        result = predictor.start_session(
            video_path,
            offload_video_to_cpu=True,
            offload_state_to_cpu=True,
        )
    else:
        result = predictor.start_session(
            video_path,
            offload_video_to_cpu=True,
        )

    sid = result["session_id"]

    if not accepts_offload:
        _patch_offload_state(predictor, sid)

    return sid


def _patch_offload_state(predictor, session_id: str) -> None:
    """Inject offload_state_to_cpu=True into an existing inference state.

    SAM2/SAM3's propagation loop checks ``inference_state["offload_state_to_cpu"]``
    to decide whether to shuttle tracking tensors to CPU between frames.
    If start_session doesn't accept the kwarg, we set it post-hoc.
    """
    states = getattr(predictor, "_all_inference_states", None)
    if states is None:
        LOGGER.warning("Cannot locate _all_inference_states; state offload unavailable")
        return

    inf_state = states.get(session_id)
    if inf_state is None:
        LOGGER.warning(
            "Session %s not in _all_inference_states; state offload unavailable",
            session_id,
        )
        return

    if isinstance(inf_state, dict):
        inf_state["offload_state_to_cpu"] = True
        LOGGER.info("Patched inference state with offload_state_to_cpu=True")
    else:
        if hasattr(inf_state, "offload_state_to_cpu"):
            inf_state.offload_state_to_cpu = True
            LOGGER.info("Patched inference state with offload_state_to_cpu=True")
        else:
            LOGGER.warning(
                "Inference state has no offload_state_to_cpu attribute; "
                "long videos may OOM"
            )


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
        LOGGER.info("segment_frame: opening session for %s", Path(video_path).name)
        t0 = time.monotonic()
        sid = _start_session(self._predictor, str(video_path))
        LOGGER.info("segment_frame: session started (%.1fs)", time.monotonic() - t0)
        try:
            LOGGER.info(
                "segment_frame: calling add_prompt on frame %d "
                "(%d pos, %d neg points) — internal mini-propagation will show a "
                "small progress bar (e.g. 0/2), this is normal",
                frame_idx,
                int((point_labels == 1).sum()),
                int((point_labels == 0).sum()),
            )
            t1 = time.monotonic()
            with _inference_ctx():
                result = self._predictor.add_prompt(
                    session_id=sid,
                    frame_idx=frame_idx,
                    points=points.tolist(),
                    point_labels=point_labels.tolist(),
                    obj_id=obj_id,
                )
            LOGGER.info(
                "segment_frame: add_prompt returned (%.1fs)", time.monotonic() - t1
            )
            mask = _extract_mask(result, obj_id)
            if mask is not None:
                pct = 100.0 * mask.sum() / max(mask.size, 1)
                LOGGER.info(
                    "segment_frame: got mask from add_prompt (%.1f%% coverage)", pct
                )
                return mask

            LOGGER.info(
                "segment_frame: add_prompt returned empty mask; "
                "falling back to 1-frame propagate"
            )
            t2 = time.monotonic()
            with _inference_ctx():
                for item in self._predictor.propagate_in_video(
                    session_id=sid,
                    start_frame_idx=frame_idx,
                    max_frame_num_to_track=1,
                ):
                    mask = _extract_mask(item, obj_id)
                    if mask is not None:
                        LOGGER.info(
                            "segment_frame: fallback propagate produced mask (%.1fs)",
                            time.monotonic() - t2,
                        )
                        return mask
            LOGGER.warning("segment_frame: fallback propagate also produced no mask")
        finally:
            self._predictor.close_session(sid)
            _free_gpu_cache()
        return None

    # OOM consistently around frame ~5000 on A6000 (48 GB).  Cap each
    # direction's propagation to stay safely under that limit.
    MAX_PROPAGATION_FRAMES = 4000

    def propagate_video(
        self,
        video_path: str | Path,
        prompt_frame_idx: int,
        points: np.ndarray,
        point_labels: np.ndarray,
        max_frames: int | None = None,
        obj_id: int = 1,
    ) -> dict[int, np.ndarray]:
        """Add prompt on one frame, propagate forward & backward, return {frame_idx: mask}.

        Each direction (forward / backward) runs in its own session, capped
        at MAX_PROPAGATION_FRAMES to avoid GPU OOM.  The tracker is stateful
        so we cannot skip frames; we simply stop when the cap is reached.
        """
        vname = Path(video_path).name
        cap_frames = max_frames if max_frames is not None else self.MAX_PROPAGATION_FRAMES
        cap_frames = min(cap_frames, self.MAX_PROPAGATION_FRAMES)
        outputs: dict[int, np.ndarray] = {}
        t_total = time.monotonic()

        # --- forward pass: prompt_frame → prompt_frame + cap_frames -----------
        LOGGER.info(
            "propagate_video [forward]: %s from frame %d, up to %d frames",
            vname, prompt_frame_idx, cap_frames,
        )
        fwd = self._propagate_one_direction(
            video_path, prompt_frame_idx, points, point_labels,
            obj_id, max_track=cap_frames, direction="forward",
        )
        outputs.update(fwd)

        # --- backward pass: prompt_frame → prompt_frame - cap_frames ----------
        bwd_limit = min(cap_frames, prompt_frame_idx)
        if bwd_limit > 0:
            LOGGER.info(
                "propagate_video [backward]: %s from frame %d, up to %d frames",
                vname, prompt_frame_idx, bwd_limit,
            )
            bwd = self._propagate_one_direction(
                video_path, prompt_frame_idx, points, point_labels,
                obj_id, max_track=bwd_limit, direction="backward",
            )
            outputs.update(bwd)

        elapsed = time.monotonic() - t_total
        LOGGER.info(
            "propagate_video: complete — %d masks in %.1fs (%.1f fps) for %s",
            len(outputs), elapsed,
            len(outputs) / max(elapsed, 0.001), vname,
        )
        return outputs

    def _propagate_one_direction(
        self,
        video_path: str | Path,
        prompt_frame_idx: int,
        points: np.ndarray,
        point_labels: np.ndarray,
        obj_id: int,
        max_track: int,
        direction: str = "forward",
    ) -> dict[int, np.ndarray]:
        """Run propagation in a single direction in its own session."""
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
                with _inference_ctx():
                    for item in self._predictor.propagate_in_video(
                        session_id=sid,
                        start_frame_idx=prompt_frame_idx,
                        max_frame_num_to_track=max_track,
                        propagation_direction=direction,
                    ):
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
                        direction, len(outputs),
                    )
                else:
                    raise

            LOGGER.info(
                "propagate_%s: collected %d masks", direction, len(outputs),
            )
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
