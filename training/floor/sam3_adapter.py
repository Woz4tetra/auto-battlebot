#!/usr/bin/env python3
"""SAM3 adapter abstraction.

This file defines the adapter contract used by the floor pipeline. It is designed
to keep the project strict-SAM3 while allowing you to swap in the exact SAM3
implementation/API available in your environment.
"""

from __future__ import annotations

import importlib
import inspect
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import cv2
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

    The exact SAM3 API differs between distributions. This adapter supports:
      1) A project-specific `build_floor_segmenter(...)` API (preferred), or
      2) Official-style predictor APIs exposed via `build_sam3_predictor(...)`.
    """

    def __init__(
        self,
        checkpoint: str,
        device: str,
        amp: bool,
        model_cfg: str | None = None,
    ) -> None:
        try:
            sam3 = importlib.import_module("sam3")
        except ModuleNotFoundError as exc:  # pragma: no cover
            raise RuntimeError(
                "SAM3 package not found. Install SAM3 with "
                "`install/install_floor_sam3_environment.sh --sam3-path ...` "
                "or provide a custom adapter module in pipeline.toml."
            ) from exc

        self._device = device
        self._amp = amp
        self._checkpoint = checkpoint
        self._model_cfg = model_cfg or ""
        self._impl = None
        self._predictor = None

        try:
            import torch

            if self._device.startswith("cuda") and not torch.cuda.is_available():
                # Fallback instead of failing hard when local driver/runtime is too old.
                self._device = "cpu"
        except Exception:  # noqa: BLE001
            pass

        builder = getattr(sam3, "build_floor_segmenter", None)
        if callable(builder):
            self._impl = builder(checkpoint=checkpoint, device=self._device, amp=amp)
            for required in ("segment_seed", "propagate_video"):
                if not hasattr(self._impl, required):
                    raise RuntimeError(f"SAM3 segmenter missing required method: {required}")
            return

        predictor_builder = getattr(sam3, "build_sam3_predictor", None)
        if callable(predictor_builder):
            self._predictor = self._build_predictor(predictor_builder, sam3)
            return

        raise RuntimeError(
            "Installed `sam3` module does not expose supported builders "
            "(`build_floor_segmenter` or `build_sam3_predictor`). "
            "Provide a custom adapter module with build_adapter(checkpoint, device, amp)."
        )

    def _discover_model_config(self, sam3_module: object) -> str | None:
        if self._model_cfg:
            cfg_path = Path(self._model_cfg)
            if cfg_path.exists():
                return str(cfg_path)
        candidates = []
        module_path = getattr(sam3_module, "__file__", None)
        if module_path:
            root = Path(module_path).resolve().parent
            candidates.extend(root.rglob("*.yaml"))
            candidates.extend(root.rglob("*.yml"))
        preferred = [
            p
            for p in candidates
            if "sam3" in p.name.lower() and ("hiera" in p.name.lower() or "tiny" in p.name.lower())
        ]
        if preferred:
            return str(sorted(preferred)[0])
        if candidates:
            return str(sorted(candidates)[0])
        return None

    def _call_with_supported_kwargs(self, fn, kwargs: dict):
        sig = inspect.signature(fn)
        filtered = {k: v for k, v in kwargs.items() if k in sig.parameters and v is not None}
        return fn(**filtered)

    def _build_predictor(self, predictor_builder, sam3_module: object):
        cfg = self._discover_model_config(sam3_module)
        sig = inspect.signature(predictor_builder)
        kwargs = {}
        for name in sig.parameters:
            lname = name.lower()
            if "checkpoint" in lname or "ckpt" in lname:
                kwargs[name] = self._checkpoint or None
            elif lname in {"device", "device_type"}:
                kwargs[name] = self._device
            elif lname in {"amp", "use_amp", "mixed_precision"}:
                kwargs[name] = self._amp
            elif "config" in lname or lname in {"cfg", "model_cfg"}:
                kwargs[name] = cfg
        try:
            return self._call_with_supported_kwargs(predictor_builder, kwargs)
        except TypeError:
            # Minimal fallback for implementations with required positional args.
            if self._checkpoint:
                try:
                    return predictor_builder(self._checkpoint)
                except Exception as exc:  # noqa: BLE001
                    raise RuntimeError(f"Unable to construct SAM3 predictor: {exc}") from exc
            raise RuntimeError(
                "Unable to construct SAM3 predictor. "
                "Set `sam3.checkpoint` in config or provide custom adapter."
            )

    @staticmethod
    def _sample_points_from_mask(mask: np.ndarray, max_points: int) -> tuple[np.ndarray, np.ndarray]:
        ys, xs = np.where(mask > 0)
        if len(xs) == 0:
            return np.zeros((0, 2), dtype=np.float32), np.zeros((0,), dtype=np.int32)
        idx = np.linspace(0, len(xs) - 1, num=min(max_points, len(xs)), dtype=int)
        pts = np.stack([xs[idx], ys[idx]], axis=1).astype(np.float32)
        labels = np.ones((pts.shape[0],), dtype=np.int32)
        return pts, labels

    @staticmethod
    def _to_numpy(x) -> np.ndarray:
        if isinstance(x, np.ndarray):
            return x
        if hasattr(x, "detach") and hasattr(x, "cpu") and hasattr(x, "numpy"):
            return x.detach().cpu().numpy()
        return np.asarray(x)

    def _extract_mask_from_item(self, item) -> np.ndarray | None:
        if isinstance(item, dict):
            for key in ("mask", "masks", "mask_logits", "logits", "pred_mask"):
                if key in item:
                    logits = item[key]
                    if isinstance(logits, (list, tuple)) and len(logits) > 0:
                        logits = logits[0]
                    arr = self._to_numpy(logits)
                    if arr.ndim == 3:
                        arr = arr[0]
                    return (arr > 0).astype(np.uint8)
            return None

        if isinstance(item, tuple):
            if len(item) >= 3:
                logits = item[2]
            elif len(item) >= 1:
                logits = item[-1]
            else:
                return None
        else:
            logits = item

        if isinstance(logits, (list, tuple)) and len(logits) > 0:
            logits = logits[0]
        arr = self._to_numpy(logits)
        if arr.ndim == 3:
            arr = arr[0]
        return (arr > 0).astype(np.uint8)

    def _call_with_signature_mapping(self, fn, value_by_name: dict[str, object]):
        """Call a function with best-effort positional/keyword mapping.

        Handles positional-only parameters (common in wrapped/C++ APIs).
        """
        sig = inspect.signature(fn)
        args: list[object] = []
        kwargs: dict[str, object] = {}
        missing_required: list[str] = []

        for name, param in sig.parameters.items():
            if name == "self":
                continue
            value = value_by_name.get(name, None)
            has_value = value is not None
            required = param.default is inspect._empty

            if param.kind is inspect.Parameter.POSITIONAL_ONLY:
                if has_value:
                    args.append(value)
                elif required:
                    missing_required.append(name)
                continue

            if param.kind is inspect.Parameter.VAR_POSITIONAL:
                continue
            if param.kind is inspect.Parameter.VAR_KEYWORD:
                continue

            if has_value:
                kwargs[name] = value
            elif required:
                missing_required.append(name)

        if missing_required:
            raise RuntimeError(
                f"Missing required parameters for {fn.__qualname__}: {missing_required}. "
                f"Signature={sig}"
            )
        return fn(*args, **kwargs)

    def _start_session(
        self,
        predictor,
        video_path: Path,
        async_prefetch: int,
        disable_cache: bool,
    ):
        if not hasattr(predictor, "start_session"):
            raise RuntimeError("Predictor has no start_session")
        start_sig = inspect.signature(predictor.start_session)
        value_by_name: dict[str, object] = {}
        for name in start_sig.parameters:
            lname = name.lower()
            if lname in {
                "video_path",
                "path",
                "source",
                "source_path",
                "video",
                "resource_path",
                "input_path",
            }:
                value_by_name[name] = str(video_path)
            elif lname in {"async_loading_frames", "async_load", "async_mode"}:
                value_by_name[name] = async_prefetch > 0
            elif lname in {"offload_video_to_cpu", "disable_cache", "offload"}:
                value_by_name[name] = bool(disable_cache)

        # First try robust mapped call; fallback to common positional form.
        try:
            return self._call_with_signature_mapping(predictor.start_session, value_by_name)
        except Exception:  # noqa: BLE001
            return predictor.start_session(str(video_path))

    def _reset_session(self, predictor, session) -> None:
        if hasattr(predictor, "reset_session"):
            sig = inspect.signature(predictor.reset_session)
            if len(sig.parameters) == 0:
                predictor.reset_session()
            else:
                predictor.reset_session(session)

    def _add_prompt_session_api(
        self,
        predictor,
        session,
        frame_idx: int,
        points: np.ndarray,
        labels: np.ndarray,
        box_xyxy: np.ndarray | None = None,
    ):
        add_sig = inspect.signature(predictor.add_prompt)
        value_by_name: dict[str, object] = {}
        for name in add_sig.parameters:
            lname = name.lower()
            if lname in {"session", "session_id", "state", "inference_state"}:
                value_by_name[name] = session
            elif lname in {"frame_idx", "frame", "index"}:
                value_by_name[name] = int(frame_idx)
            elif lname in {"points", "point_coords", "coords"}:
                value_by_name[name] = points
            elif lname in {"labels", "point_labels"}:
                value_by_name[name] = labels
            elif lname in {"obj_id", "object_id"}:
                value_by_name[name] = 1
            elif lname in {"box", "bbox", "xyxy"} and box_xyxy is not None:
                value_by_name[name] = box_xyxy
            elif lname in {"prompt", "payload", "request"}:
                value_by_name[name] = {
                    "frame_idx": int(frame_idx),
                    "points": points,
                    "labels": labels,
                    "obj_id": 1,
                    "box": box_xyxy,
                }
        return self._call_with_signature_mapping(predictor.add_prompt, value_by_name)

    def _seed_via_video_api(self, seed: SegmentationSeed) -> np.ndarray:
        """Fallback seed generation using video predictor APIs on a 1-frame temp video."""
        predictor = self._predictor
        if predictor is None:
            raise RuntimeError("SAM3 predictor not initialized.")
        if not (hasattr(predictor, "init_state") and hasattr(predictor, "propagate_in_video")):
            raise RuntimeError("Predictor does not provide video APIs for seed fallback.")

        frame = seed.frame_bgr
        h, w = frame.shape[:2]
        with tempfile.TemporaryDirectory(prefix="sam3_seed_") as td:
            temp_video = Path(td) / "seed.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(temp_video), fourcc, 1.0, (w, h))
            writer.write(frame)
            writer.release()

            init_state_sig = inspect.signature(predictor.init_state)
            init_kwargs = {}
            if "video_path" in init_state_sig.parameters:
                init_kwargs["video_path"] = str(temp_video)
            if "async_loading_frames" in init_state_sig.parameters:
                init_kwargs["async_loading_frames"] = False
            if "offload_video_to_cpu" in init_state_sig.parameters:
                init_kwargs["offload_video_to_cpu"] = True
            inference_state = predictor.init_state(**init_kwargs)
            if hasattr(predictor, "reset_state"):
                predictor.reset_state(inference_state)

            points = np.array(seed.positive_points_xy + seed.negative_points_xy, dtype=np.float32)
            labels = np.array(
                [1] * len(seed.positive_points_xy) + [0] * len(seed.negative_points_xy),
                dtype=np.int32,
            )
            if len(points) == 0:
                # Ensure at least one positive point near arena center.
                if seed.arena_box_xyxy is not None:
                    x1, y1, x2, y2 = seed.arena_box_xyxy
                    cx, cy = int((x1 + x2) * 0.5), int((y1 + y2) * 0.5)
                else:
                    cx, cy = w // 2, h // 2
                points = np.array([[cx, cy]], dtype=np.float32)
                labels = np.array([1], dtype=np.int32)

            if hasattr(predictor, "add_new_points_or_box"):
                add_sig = inspect.signature(predictor.add_new_points_or_box)
                add_kwargs = {}
                if "inference_state" in add_sig.parameters:
                    add_kwargs["inference_state"] = inference_state
                elif "state" in add_sig.parameters:
                    add_kwargs["state"] = inference_state
                if "frame_idx" in add_sig.parameters:
                    add_kwargs["frame_idx"] = 0
                if "obj_id" in add_sig.parameters:
                    add_kwargs["obj_id"] = 1
                if "points" in add_sig.parameters:
                    add_kwargs["points"] = points
                if "point_coords" in add_sig.parameters:
                    add_kwargs["point_coords"] = points
                if "labels" in add_sig.parameters:
                    add_kwargs["labels"] = labels
                if "point_labels" in add_sig.parameters:
                    add_kwargs["point_labels"] = labels
                if seed.arena_box_xyxy is not None:
                    box_np = np.array(seed.arena_box_xyxy, dtype=np.float32)
                    if "box" in add_sig.parameters:
                        add_kwargs["box"] = box_np
                    if "bbox" in add_sig.parameters:
                        add_kwargs["bbox"] = box_np
                predictor.add_new_points_or_box(**add_kwargs)

            for item in predictor.propagate_in_video(inference_state):
                if not isinstance(item, tuple) or len(item) < 3:
                    continue
                mask_logits = item[2]
                if isinstance(mask_logits, (list, tuple)) and len(mask_logits) > 0:
                    mask_arr = np.asarray(mask_logits[0])
                else:
                    mask_arr = np.asarray(mask_logits)
                return (mask_arr > 0).astype(np.uint8)

        raise RuntimeError("Video-API seed fallback produced no mask.")

    def _seed_via_session_api(self, seed: SegmentationSeed) -> np.ndarray:
        predictor = self._predictor
        if predictor is None:
            raise RuntimeError("SAM3 predictor not initialized.")
        if not (
            hasattr(predictor, "start_session")
            and hasattr(predictor, "add_prompt")
            and hasattr(predictor, "propagate_in_video")
        ):
            raise RuntimeError("Predictor does not provide session APIs for seed fallback.")

        frame = seed.frame_bgr
        h, w = frame.shape[:2]
        with tempfile.TemporaryDirectory(prefix="sam3_seed_") as td:
            temp_video = Path(td) / "seed.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(temp_video), fourcc, 1.0, (w, h))
            writer.write(frame)
            writer.release()

            session = self._start_session(
                predictor=predictor,
                video_path=temp_video,
                async_prefetch=0,
                disable_cache=True,
            )
            self._reset_session(predictor, session)

            points = np.array(seed.positive_points_xy + seed.negative_points_xy, dtype=np.float32)
            labels = np.array(
                [1] * len(seed.positive_points_xy) + [0] * len(seed.negative_points_xy),
                dtype=np.int32,
            )
            if len(points) == 0:
                if seed.arena_box_xyxy is not None:
                    x1, y1, x2, y2 = seed.arena_box_xyxy
                    cx, cy = int((x1 + x2) * 0.5), int((y1 + y2) * 0.5)
                else:
                    cx, cy = w // 2, h // 2
                points = np.array([[cx, cy]], dtype=np.float32)
                labels = np.array([1], dtype=np.int32)
            box_np = (
                np.array(seed.arena_box_xyxy, dtype=np.float32)
                if seed.arena_box_xyxy is not None
                else None
            )

            prompt_result = self._add_prompt_session_api(
                predictor=predictor,
                session=session,
                frame_idx=0,
                points=points,
                labels=labels,
                box_xyxy=box_np,
            )
            prompt_mask = self._extract_mask_from_item(prompt_result)
            if prompt_mask is not None:
                return prompt_mask

            prop_sig = inspect.signature(predictor.propagate_in_video)
            prop_map = {}
            if "session" in prop_sig.parameters:
                prop_map["session"] = session
            if "session_id" in prop_sig.parameters:
                prop_map["session_id"] = session
            if "state" in prop_sig.parameters:
                prop_map["state"] = session
            try:
                iterator = self._call_with_signature_mapping(predictor.propagate_in_video, prop_map)
            except Exception:  # noqa: BLE001
                iterator = predictor.propagate_in_video(session)
            for item in iterator:
                mask = self._extract_mask_from_item(item)
                if mask is not None:
                    return mask

        raise RuntimeError("Session-API seed fallback produced no mask.")

    def segment_seed(self, seed: SegmentationSeed) -> np.ndarray:
        if self._impl is not None:
            mask = self._impl.segment_seed(seed)
            return np.asarray(mask, dtype=np.uint8)

        predictor = self._predictor
        if predictor is None:
            raise RuntimeError("SAM3 predictor not initialized.")

        if hasattr(predictor, "segment_seed"):
            return np.asarray(predictor.segment_seed(seed), dtype=np.uint8)

        if hasattr(predictor, "set_image") and hasattr(predictor, "predict"):
            try:
                frame_rgb = seed.frame_bgr[:, :, ::-1]
                predictor.set_image(frame_rgb)
                points = np.array(seed.positive_points_xy + seed.negative_points_xy, dtype=np.float32)
                labels = np.array(
                    [1] * len(seed.positive_points_xy) + [0] * len(seed.negative_points_xy),
                    dtype=np.int32,
                )
                kwargs = {"multimask_output": False}
                sig = inspect.signature(predictor.predict)
                if "point_coords" in sig.parameters:
                    kwargs["point_coords"] = points if len(points) else None
                if "point_labels" in sig.parameters:
                    kwargs["point_labels"] = labels if len(labels) else None
                if seed.arena_box_xyxy is not None and "box" in sig.parameters:
                    kwargs["box"] = np.array(seed.arena_box_xyxy, dtype=np.float32)
                outputs = predictor.predict(**kwargs)
                if isinstance(outputs, tuple) and len(outputs) >= 1:
                    masks = outputs[0]
                else:
                    masks = outputs
                mask = np.asarray(masks[0] if np.ndim(masks) == 3 else masks, dtype=np.uint8)
                return (mask > 0).astype(np.uint8)
            except Exception:  # noqa: BLE001
                # Fall through to video-API fallback.
                pass

        if hasattr(predictor, "init_state") and hasattr(predictor, "propagate_in_video"):
            return self._seed_via_video_api(seed)

        if (
            hasattr(predictor, "start_session")
            and hasattr(predictor, "add_prompt")
            and hasattr(predictor, "propagate_in_video")
        ):
            return self._seed_via_session_api(seed)

        available = [name for name in dir(predictor) if not name.startswith("_")]
        raise RuntimeError(
            "SAM3 predictor does not expose a supported seed segmentation interface. "
            f"Available predictor methods: {available[:80]}"
        )

    def propagate_video(
        self,
        video_path: Path,
        initial_mask: np.ndarray,
        start_frame_idx: int,
        end_frame_idx: int,
        async_prefetch: int,
        disable_cache: bool,
    ) -> dict[int, np.ndarray]:
        if self._impl is not None:
            outputs = self._impl.propagate_video(
                video_path=video_path,
                initial_mask=initial_mask,
                start_frame_idx=start_frame_idx,
                end_frame_idx=end_frame_idx,
                async_prefetch=async_prefetch,
                disable_cache=disable_cache,
            )
            return {int(k): np.asarray(v, dtype=np.uint8) for k, v in outputs.items()}

        predictor = self._predictor
        if predictor is None:
            raise RuntimeError("SAM3 predictor not initialized.")

        if hasattr(predictor, "propagate_video"):
            outputs = predictor.propagate_video(
                video_path=video_path,
                initial_mask=initial_mask,
                start_frame_idx=start_frame_idx,
                end_frame_idx=end_frame_idx,
                async_prefetch=async_prefetch,
                disable_cache=disable_cache,
            )
            return {int(k): np.asarray(v, dtype=np.uint8) for k, v in outputs.items()}

        if hasattr(predictor, "init_state") and hasattr(predictor, "propagate_in_video"):
            init_state_sig = inspect.signature(predictor.init_state)
            init_kwargs = {}
            if "video_path" in init_state_sig.parameters:
                init_kwargs["video_path"] = str(video_path)
            if "async_loading_frames" in init_state_sig.parameters:
                init_kwargs["async_loading_frames"] = async_prefetch > 0
            if "offload_video_to_cpu" in init_state_sig.parameters:
                init_kwargs["offload_video_to_cpu"] = disable_cache
            inference_state = predictor.init_state(**init_kwargs)
            if hasattr(predictor, "reset_state"):
                predictor.reset_state(inference_state)

            points, labels = self._sample_points_from_mask(initial_mask, max_points=32)
            if hasattr(predictor, "add_new_points_or_box"):
                add_sig = inspect.signature(predictor.add_new_points_or_box)
                add_kwargs = {}
                if "inference_state" in add_sig.parameters:
                    add_kwargs["inference_state"] = inference_state
                elif "state" in add_sig.parameters:
                    add_kwargs["state"] = inference_state
                if "frame_idx" in add_sig.parameters:
                    add_kwargs["frame_idx"] = start_frame_idx
                if "obj_id" in add_sig.parameters:
                    add_kwargs["obj_id"] = 1
                if "points" in add_sig.parameters:
                    add_kwargs["points"] = points
                if "point_coords" in add_sig.parameters:
                    add_kwargs["point_coords"] = points
                if "labels" in add_sig.parameters:
                    add_kwargs["labels"] = labels
                if "point_labels" in add_sig.parameters:
                    add_kwargs["point_labels"] = labels
                predictor.add_new_points_or_box(**add_kwargs)

            outputs: dict[int, np.ndarray] = {}
            for item in predictor.propagate_in_video(inference_state):
                if not isinstance(item, tuple) or len(item) < 3:
                    continue
                frame_idx = int(item[0])
                if frame_idx < start_frame_idx or frame_idx > end_frame_idx:
                    continue
                mask_logits = item[2]
                # mask_logits may be tensor-like list/array per object.
                if isinstance(mask_logits, (list, tuple)) and len(mask_logits) > 0:
                    mask_arr = np.asarray(mask_logits[0])
                else:
                    mask_arr = np.asarray(mask_logits)
                outputs[frame_idx] = (mask_arr > 0).astype(np.uint8)
            return outputs

        if (
            hasattr(predictor, "start_session")
            and hasattr(predictor, "add_prompt")
            and hasattr(predictor, "propagate_in_video")
        ):
            session = self._start_session(
                predictor=predictor,
                video_path=video_path,
                async_prefetch=async_prefetch,
                disable_cache=disable_cache,
            )
            self._reset_session(predictor, session)

            points, labels = self._sample_points_from_mask(initial_mask, max_points=32)
            if len(points) == 0:
                # ensure one positive point if mask is empty
                h, w = initial_mask.shape[:2]
                points = np.array([[w // 2, h // 2]], dtype=np.float32)
                labels = np.array([1], dtype=np.int32)

            self._add_prompt_session_api(
                predictor=predictor,
                session=session,
                frame_idx=start_frame_idx,
                points=points,
                labels=labels,
                box_xyxy=None,
            )

            outputs: dict[int, np.ndarray] = {}
            prop_sig = inspect.signature(predictor.propagate_in_video)
            prop_map = {}
            if "session" in prop_sig.parameters:
                prop_map["session"] = session
            if "session_id" in prop_sig.parameters:
                prop_map["session_id"] = session
            if "state" in prop_sig.parameters:
                prop_map["state"] = session
            try:
                iterator = self._call_with_signature_mapping(predictor.propagate_in_video, prop_map)
            except Exception:  # noqa: BLE001
                iterator = predictor.propagate_in_video(session)

            for item in iterator:
                if isinstance(item, tuple) and len(item) > 0:
                    frame_idx = int(item[0])
                else:
                    # If API variant doesn't return frame index tuple, skip.
                    continue
                if frame_idx < start_frame_idx or frame_idx > end_frame_idx:
                    continue
                mask = self._extract_mask_from_item(item)
                if mask is None:
                    continue
                outputs[frame_idx] = mask
            return outputs

        raise RuntimeError("SAM3 predictor does not expose a supported video propagation API.")


def build_adapter(
    adapter_module: str,
    checkpoint: str,
    device: str,
    amp: bool,
    model_cfg: str | None = None,
) -> Sam3AdapterProtocol:
    """Build adapter from module path.

    If adapter_module is this file, it uses `NativeSam3Adapter`.
    Otherwise it imports adapter_module and calls:
      build_adapter(checkpoint: str, device: str, amp: bool) -> Sam3AdapterProtocol
    """
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
            f"Custom adapter module `{adapter_module}` is missing `build_adapter`."
        )
    try:
        adapter = builder(
            checkpoint=checkpoint,
            device=device,
            amp=amp,
            model_cfg=model_cfg,
        )
    except TypeError:
        adapter = builder(checkpoint=checkpoint, device=device, amp=amp)
    for required in ("segment_seed", "propagate_video"):
        if not hasattr(adapter, required):
            raise RuntimeError(
                f"Custom adapter `{adapter_module}` missing required method `{required}`"
            )
    return adapter
