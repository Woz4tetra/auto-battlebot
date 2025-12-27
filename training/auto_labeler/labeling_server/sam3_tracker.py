"""SAM3 video segmentation tracker integration with memory-efficient frame loading."""

import os
import shutil
import tempfile
import cv2
import numpy as np
import torch
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from tqdm import tqdm

# Import SAM3 components
try:
    from sam3.model_builder import build_sam3_video_model
except ImportError:
    print("Warning: SAM3 not installed. Install with: pip install sam3")
    build_sam3_video_model = None


class SAM3Tracker:
    """
    Wrapper for SAM3 video segmentation model with memory-efficient frame loading.

    Instead of loading the entire video into GPU memory, this extracts only the
    frames needed for the current propagation operation into a temporary directory.
    Frames are optionally scaled down to reduce memory usage, and masks are
    scaled back to original resolution after inference.
    """

    def __init__(self, gpu_id: int = 0, inference_scale: float = 0.5):
        """
        Initialize the tracker.

        Args:
            gpu_id: GPU device ID to use
            inference_scale: Scale factor for frames during inference (0.0-1.0)
                           e.g., 0.5 = half resolution
        """
        self.gpu_id = gpu_id
        self.inference_scale = inference_scale
        self.device = self._setup_device()
        self.model = None
        self.predictor = None
        self.inference_state = None

        # Video info (loaded lazily)
        self.video_path: Optional[str] = None
        self.video_width: int = 0
        self.video_height: int = 0
        self.total_frames: int = 0
        self.fps: float = 0.0

        # Scaled dimensions for inference
        self.scaled_width: int = 0
        self.scaled_height: int = 0

        # Temporary directory for frame extraction
        self._temp_dir: Optional[str] = None
        self._extracted_frames: Dict[int, str] = {}  # frame_idx -> file path

    def _setup_device(self) -> torch.device:
        """Set up computation device with optimizations."""
        if torch.cuda.is_available():
            device = torch.device(f"cuda:{self.gpu_id}")
            torch.cuda.set_device(self.gpu_id)

            # Enable optimizations
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
            if torch.cuda.get_device_properties(self.gpu_id).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True
        elif torch.backends.mps.is_available():
            device = torch.device("mps")
        else:
            device = torch.device("cpu")

        print(f"SAM3 Tracker using device: {device}")
        return device

    def load_model(self):
        """Load SAM3 model."""
        if build_sam3_video_model is None:
            raise RuntimeError("SAM3 not installed")

        print("Loading SAM3 model...")
        self.model = build_sam3_video_model()
        self.predictor = self.model.tracker
        self.predictor.backbone = self.model.detector.backbone
        print("SAM3 model loaded successfully")

    def set_video(self, video_path: str):
        """
        Set the video file to work with (does NOT load it into memory).

        Args:
            video_path: Path to the video file
        """
        if self.predictor is None:
            self.load_model()

        self.video_path = video_path

        # Load video metadata only
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            raise ValueError(f"Cannot open video: {video_path}")

        self.video_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.video_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        cap.release()

        # Calculate scaled dimensions
        self.scaled_width = int(self.video_width * self.inference_scale)
        self.scaled_height = int(self.video_height * self.inference_scale)

        # Ensure dimensions are even (required by many video codecs)
        self.scaled_width = self.scaled_width - (self.scaled_width % 2)
        self.scaled_height = self.scaled_height - (self.scaled_height % 2)

        print(
            f"Video: {self.video_width}x{self.video_height}, {self.total_frames} frames"
        )
        print(
            f"Inference scale: {self.inference_scale} -> {self.scaled_width}x{self.scaled_height}"
        )

        # Clean up any previous temp directory
        self._cleanup_temp_dir()

    def _cleanup_temp_dir(self):
        """Clean up temporary frame directory."""
        if self._temp_dir and os.path.exists(self._temp_dir):
            shutil.rmtree(self._temp_dir, ignore_errors=True)
        self._temp_dir = None
        self._extracted_frames = {}

    def _reset_inference_state(self):
        """Reset the inference state to free GPU memory."""
        if self.inference_state is not None:
            # Use SAM3's reset_state if available
            if hasattr(self.predictor, "reset_state"):
                try:
                    self.predictor.reset_state(self.inference_state)
                except Exception:
                    pass  # Ignore errors during reset

            # Clear references to allow garbage collection
            self.inference_state = None

        # Clear GPU memory cache
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    def _extract_frames_for_propagation(
        self,
        start_frame: int,
        num_frames: int,
    ) -> str:
        """
        Extract and scale frames to a temporary directory for SAM3 inference.

        Args:
            start_frame: Starting frame index
            num_frames: Number of frames to extract

        Returns:
            Path to temporary directory containing frames
        """
        # Reset inference state first to free GPU memory
        self._reset_inference_state()

        # Clean up previous extraction
        self._cleanup_temp_dir()

        # Create new temp directory
        self._temp_dir = tempfile.mkdtemp(prefix="sam3_frames_")

        print(f"Extracting {num_frames} frames starting at {start_frame}...")

        cap = cv2.VideoCapture(self.video_path)
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

        end_frame = min(start_frame + num_frames, self.total_frames)

        for frame_idx in tqdm(range(start_frame, end_frame), desc="Extracting frames"):
            ret, frame = cap.read()
            if not ret:
                break

            # Scale down frame
            if self.inference_scale != 1.0:
                frame = cv2.resize(
                    frame,
                    (self.scaled_width, self.scaled_height),
                    interpolation=cv2.INTER_AREA,
                )

            # Save as JPEG (SAM3 expects image files)
            # Use sequential naming starting from 0 for the temp directory
            local_idx = frame_idx - start_frame
            frame_path = os.path.join(self._temp_dir, f"{local_idx:06d}.jpg")
            cv2.imwrite(frame_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

            self._extracted_frames[frame_idx] = frame_path

        cap.release()

        print(f"Extracted {len(self._extracted_frames)} frames to {self._temp_dir}")
        return self._temp_dir

    def _init_inference_state(self, frames_dir: str):
        """Initialize SAM3 inference state with extracted frames directory."""
        print(f"Initializing inference state from: {frames_dir}")
        self.inference_state = self.predictor.init_state(video_path=frames_dir)
        print("Inference state initialized")

    def _scale_mask_to_original(self, mask: np.ndarray) -> np.ndarray:
        """
        Scale a mask from inference resolution back to original video resolution.

        Args:
            mask: Binary mask at inference resolution

        Returns:
            Binary mask at original video resolution
        """
        if self.inference_scale == 1.0:
            return mask

        # Ensure mask is 2D
        if mask.ndim == 3:
            mask = mask.squeeze()

        # Scale up using nearest neighbor to preserve binary values
        scaled = cv2.resize(
            mask.astype(np.uint8),
            (self.video_width, self.video_height),
            interpolation=cv2.INTER_NEAREST,
        )

        return scaled.astype(bool)

    def add_points(
        self,
        local_frame_idx: int,
        obj_id: int,
        points: np.ndarray,
        labels: np.ndarray,
        clear_old: bool = False,
    ) -> Optional[np.ndarray]:
        """
        Add point prompts for an object on a specific frame.

        Args:
            local_frame_idx: Frame index relative to extracted frames (0-based)
            obj_id: Object ID
            points: Points array with normalized coordinates [[x, y], ...]
            labels: Labels array [1, 0, ...] (1=positive, 0=negative)
            clear_old: Whether to clear existing points for this object

        Returns:
            Predicted mask for the frame at ORIGINAL resolution, or None
        """
        if self.inference_state is None:
            raise RuntimeError("Inference state not initialized")

        if len(points) == 0:
            return None

        points_tensor = torch.tensor(points, dtype=torch.float32)
        labels_tensor = torch.tensor(labels, dtype=torch.int32)

        _, out_obj_ids, low_res_masks, video_res_masks = self.predictor.add_new_points(
            inference_state=self.inference_state,
            frame_idx=local_frame_idx,
            obj_id=obj_id,
            points=points_tensor,
            labels=labels_tensor,
            clear_old_points=clear_old,
        )

        # Return the mask for this object, scaled to original size
        if video_res_masks is not None:
            for i, oid in enumerate(out_obj_ids):
                if oid == obj_id:
                    mask = (video_res_masks[i] > 0.0).cpu().numpy()
                    return self._scale_mask_to_original(mask)

        return None

    def propagate_from_frame(
        self,
        source_frame: int,
        points_by_obj: Dict[int, Tuple[np.ndarray, np.ndarray]],
        propagate_length: int,
        callback=None,
    ) -> Dict[int, Dict[int, np.ndarray]]:
        """
        Extract frames, add points, and propagate masks.

        This is the main entry point for propagation. It:
        1. Extracts only the needed frames to a temp directory
        2. Initializes SAM3 with those frames
        3. Adds the point prompts
        4. Propagates through the frames
        5. Scales masks back to original resolution

        Args:
            source_frame: Frame index in the original video to start from
            points_by_obj: Dict of {obj_id: (points, labels)} with normalized coords
            propagate_length: Number of frames to propagate
            callback: Optional callback(frame_idx, progress) for progress updates

        Returns:
            Dict of {original_frame_idx: {obj_id: mask_at_original_resolution}}
        """
        if self.video_path is None:
            raise RuntimeError("No video set. Call set_video() first.")

        # Clamp propagate_length to not exceed video length
        max_frames = min(propagate_length, self.total_frames - source_frame)

        # Extract frames to temp directory
        frames_dir = self._extract_frames_for_propagation(source_frame, max_frames)

        # Initialize inference state with extracted frames
        self._init_inference_state(frames_dir)

        # Add points for each object (at local frame index 0)
        local_source_idx = 0  # Source frame is always at index 0 in temp directory
        for obj_id, (points, labels) in points_by_obj.items():
            if len(points) > 0:
                self.add_points(
                    local_frame_idx=local_source_idx,
                    obj_id=obj_id,
                    points=points,
                    labels=labels,
                    clear_old=True,
                )

        # Propagate through extracted frames
        video_segments = {}

        for (
            local_frame_idx,
            obj_ids,
            low_res_masks,
            video_res_masks,
            obj_scores,
        ) in self.predictor.propagate_in_video(
            self.inference_state,
            start_frame_idx=0,
            max_frame_num_to_track=max_frames,
            reverse=False,
            propagate_preflight=True,
        ):
            # Convert local frame index back to original video frame index
            original_frame_idx = source_frame + local_frame_idx

            # Scale masks to original resolution
            video_segments[original_frame_idx] = {}
            for i, out_obj_id in enumerate(obj_ids):
                mask = (video_res_masks[i] > 0.0).cpu().numpy()
                scaled_mask = self._scale_mask_to_original(mask)
                video_segments[original_frame_idx][out_obj_id] = scaled_mask

            if callback:
                progress = (local_frame_idx + 1) / max_frames
                callback(original_frame_idx, progress)

        # Clean up to free GPU memory and disk space
        self._reset_inference_state()
        self._cleanup_temp_dir()

        return video_segments

    def get_preview_mask(
        self,
        frame_idx: int,
        obj_id: int,
        points: np.ndarray,
        labels: np.ndarray,
    ) -> Optional[np.ndarray]:
        """
        Get a preview mask for points on a single frame.

        This extracts just one frame for preview purposes.

        Args:
            frame_idx: Frame index in original video
            obj_id: Object ID
            points: Normalized point coordinates
            labels: Point labels

        Returns:
            Mask at original resolution, or None
        """
        if len(points) == 0:
            return None

        if self.video_path is None:
            raise RuntimeError("No video set. Call set_video() first.")

        # Extract just a few frames around the target (SAM3 needs some context)
        # We'll extract 3 frames: target-1, target, target+1
        start = max(0, frame_idx - 1)
        num_frames = min(3, self.total_frames - start)
        local_idx = frame_idx - start

        frames_dir = self._extract_frames_for_propagation(start, num_frames)
        self._init_inference_state(frames_dir)

        # Add points and get mask
        mask = self.add_points(
            local_frame_idx=local_idx,
            obj_id=obj_id,
            points=points,
            labels=labels,
            clear_old=True,
        )

        # Clean up to free GPU memory
        self._reset_inference_state()
        self._cleanup_temp_dir()

        return mask

    def get_multi_object_preview(
        self,
        frame_idx: int,
        points_by_obj: Dict[int, Tuple[np.ndarray, np.ndarray]],
    ) -> Dict[int, np.ndarray]:
        """
        Get preview masks for multiple objects on a single frame.

        Args:
            frame_idx: Frame index in original video
            points_by_obj: Dict of {obj_id: (points, labels)}

        Returns:
            Dict of {obj_id: mask_at_original_resolution}
        """
        if self.video_path is None:
            raise RuntimeError("No video set. Call set_video() first.")

        # Extract just a few frames around the target
        start = max(0, frame_idx - 1)
        num_frames = min(3, self.total_frames - start)
        local_idx = frame_idx - start

        frames_dir = self._extract_frames_for_propagation(start, num_frames)
        self._init_inference_state(frames_dir)

        masks = {}
        for obj_id, (points, labels) in points_by_obj.items():
            if len(points) > 0:
                mask = self.add_points(
                    local_frame_idx=local_idx,
                    obj_id=obj_id,
                    points=points,
                    labels=labels,
                    clear_old=True,
                )
                if mask is not None:
                    masks[obj_id] = mask

        # Clean up to free GPU memory
        self._reset_inference_state()
        self._cleanup_temp_dir()

        return masks

    def cleanup(self):
        """Clean up all resources."""
        self._reset_inference_state()
        self._cleanup_temp_dir()

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()
