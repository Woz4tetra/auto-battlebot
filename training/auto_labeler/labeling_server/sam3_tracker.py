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
from natsort import natsorted

# Import SAM3 components
try:
    from sam3.model_builder import build_sam3_video_model
except ImportError:
    print("Warning: SAM3 not installed. Install with: pip install sam3")
    build_sam3_video_model = None


# Supported image extensions
SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp"}


class SAM3Tracker:
    """
    Wrapper for SAM3 video segmentation model with memory-efficient frame loading.

    Works with folders of images. For propagation, creates a temporary directory
    with symlinks to the original images (scaled if needed). This avoids copying
    images while still providing SAM3 with the sequential file naming it expects.
    """

    def __init__(self, gpu_id: int = 0, inference_width: int = 960):
        """
        Initialize the tracker.

        Args:
            gpu_id: GPU device ID to use
            inference_width: Target width in pixels for inference
                           Height is calculated to maintain aspect ratio
        """
        self.gpu_id = gpu_id
        self.inference_width = inference_width
        self.device = self._setup_device()
        self.model = None
        self.predictor = None
        self.inference_state = None

        # Image folder info (loaded lazily)
        self.images_dir: Optional[Path] = None
        self.image_files: List[Path] = []  # Sorted list of image paths
        self.image_width: int = 0
        self.image_height: int = 0
        self.total_frames: int = 0

        # Scaled dimensions for inference
        self.scaled_width: int = 0
        self.scaled_height: int = 0

        # Temporary directory for scaled frames (only used if scaling needed)
        self._temp_dir: Optional[str] = None
        self._original_frames: Dict[
            int, np.ndarray
        ] = {}  # frame_idx -> original resolution frame

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
        Set the images directory to work with.

        Args:
            video_path: Path to directory containing images (named 'video_path' for compatibility)
        """
        self.set_images_dir(video_path)

    def set_images_dir(self, images_dir: str):
        """
        Set the images directory to work with.

        Args:
            images_dir: Path to directory containing images
        """
        if self.predictor is None:
            self.load_model()

        self.images_dir = Path(images_dir)

        if not self.images_dir.exists():
            raise FileNotFoundError(f"Images directory not found: {images_dir}")

        if not self.images_dir.is_dir():
            raise ValueError(f"Path is not a directory: {images_dir}")

        # Load and sort image files
        self.image_files = []
        for ext in SUPPORTED_EXTENSIONS:
            self.image_files.extend(self.images_dir.glob(f"*{ext}"))
            self.image_files.extend(self.images_dir.glob(f"*{ext.upper()}"))

        # Natural sort to handle numeric naming correctly
        self.image_files = natsorted(self.image_files, key=lambda p: p.name)

        if not self.image_files:
            raise ValueError(f"No images found in: {images_dir}")

        self.total_frames = len(self.image_files)

        # Load first image to get dimensions
        first_image = cv2.imread(str(self.image_files[0]))
        if first_image is None:
            raise ValueError(f"Cannot read image: {self.image_files[0]}")

        self.image_height, self.image_width = first_image.shape[:2]

        # Calculate scaled dimensions from target width
        if self.inference_width >= self.image_width:
            # No scaling needed if target is >= original
            self.scaled_width = self.image_width
            self.scaled_height = self.image_height
        else:
            # Scale to target width, maintain aspect ratio
            scale = self.inference_width / self.image_width
            self.scaled_width = self.inference_width
            self.scaled_height = int(self.image_height * scale)

        # Ensure dimensions are even (required by many video codecs)
        self.scaled_width = self.scaled_width - (self.scaled_width % 2)
        self.scaled_height = self.scaled_height - (self.scaled_height % 2)

        print(
            f"Images: {self.image_width}x{self.image_height}, {self.total_frames} frames"
        )
        print(
            f"Inference width: {self.inference_width} -> {self.scaled_width}x{self.scaled_height}"
        )

        # Clean up any previous temp directory
        self._cleanup_temp_dir()

    def _cleanup_temp_dir(self):
        """Clean up temporary frame directory."""
        if self._temp_dir and os.path.exists(self._temp_dir):
            shutil.rmtree(self._temp_dir, ignore_errors=True)
        self._temp_dir = None
        self._original_frames = {}

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

    def _prepare_frames_for_propagation(
        self,
        start_frame: int,
        num_frames: int,
    ) -> str:
        """
        Prepare frames for SAM3 inference by creating a temp directory with
        sequential file naming. Uses symlinks if no scaling needed, otherwise
        creates scaled copies.

        Args:
            start_frame: Starting frame index
            num_frames: Number of frames to process

        Returns:
            Path to directory containing prepared frames
        """
        # Reset inference state first to free GPU memory
        self._reset_inference_state()

        # Clean up previous temp directory
        self._cleanup_temp_dir()

        end_frame = min(start_frame + num_frames, self.total_frames)
        actual_num_frames = end_frame - start_frame

        print(f"Preparing {actual_num_frames} frames starting at {start_frame}...")

        # Check if scaling is needed
        needs_scaling = self.scaled_width != self.image_width

        if needs_scaling:
            # Create temp directory for scaled frames
            self._temp_dir = tempfile.mkdtemp(prefix="sam3_frames_")

            for local_idx, frame_idx in enumerate(
                tqdm(range(start_frame, end_frame), desc="Scaling frames")
            ):
                # Load original image
                image_path = self.image_files[frame_idx]
                frame = cv2.imread(str(image_path))
                if frame is None:
                    raise ValueError(f"Cannot read image: {image_path}")

                # Store original resolution frame for later use
                self._original_frames[frame_idx] = frame.copy()

                # Scale down frame
                scaled_frame = cv2.resize(
                    frame,
                    (self.scaled_width, self.scaled_height),
                    interpolation=cv2.INTER_AREA,
                )

                # Save with sequential naming
                frame_path = os.path.join(self._temp_dir, f"{local_idx:06d}.jpg")
                cv2.imwrite(frame_path, scaled_frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

            print(f"Created {actual_num_frames} scaled frames in {self._temp_dir}")
            return self._temp_dir

        else:
            # No scaling needed - create temp directory with symlinks
            self._temp_dir = tempfile.mkdtemp(prefix="sam3_frames_")

            for local_idx, frame_idx in enumerate(range(start_frame, end_frame)):
                image_path = self.image_files[frame_idx]

                # Load and store original frame for later use
                frame = cv2.imread(str(image_path))
                if frame is None:
                    raise ValueError(f"Cannot read image: {image_path}")
                self._original_frames[frame_idx] = frame

                # Create symlink with sequential naming
                link_path = os.path.join(
                    self._temp_dir, f"{local_idx:06d}{image_path.suffix}"
                )
                os.symlink(image_path.absolute(), link_path)

            print(f"Created {actual_num_frames} symlinks in {self._temp_dir}")
            return self._temp_dir

    def _init_inference_state(self, frames_dir: str):
        """Initialize SAM3 inference state with extracted frames directory."""
        print(f"Initializing inference state from: {frames_dir}")
        self.inference_state = self.predictor.init_state(video_path=frames_dir)
        print("Inference state initialized")

    def _scale_mask_to_original(self, mask: np.ndarray) -> np.ndarray:
        """
        Scale a mask from inference resolution back to original image resolution.

        Args:
            mask: Binary mask at inference resolution

        Returns:
            Binary mask at original image resolution
        """
        # No scaling needed if dimensions match
        if self.scaled_width == self.image_width:
            return mask

        # Ensure mask is 2D
        if mask.ndim == 3:
            mask = mask.squeeze()

        # Scale up using nearest neighbor to preserve binary values
        scaled = cv2.resize(
            mask.astype(np.uint8),
            (self.image_width, self.image_height),
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
    ) -> Tuple[Dict[int, Dict[int, np.ndarray]], Dict[int, np.ndarray]]:
        """
        Prepare frames, add points, and propagate masks.

        This is the main entry point for propagation. It:
        1. Prepares frames in a temp directory (symlinks or scaled copies)
        2. Initializes SAM3 with those frames
        3. Adds the point prompts
        4. Propagates through the frames
        5. Scales masks back to original resolution

        Args:
            source_frame: Frame index to start from
            points_by_obj: Dict of {obj_id: (points, labels)} with normalized coords
            propagate_length: Number of frames to propagate
            callback: Optional callback(frame_idx, progress) for progress updates

        Returns:
            Tuple of:
            - Dict of {frame_idx: {obj_id: mask_at_original_resolution}}
            - Dict of {frame_idx: frame_at_original_resolution}
        """
        if self.images_dir is None:
            raise RuntimeError("No images directory set. Call set_images_dir() first.")

        # Clamp propagate_length to not exceed available frames
        max_frames = min(propagate_length, self.total_frames - source_frame)

        # Prepare frames in temp directory
        frames_dir = self._prepare_frames_for_propagation(source_frame, max_frames)

        # Initialize inference state with prepared frames
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

        # Propagate through frames
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

        # Copy original frames before cleanup
        original_frames = dict(self._original_frames)

        # Clean up to free GPU memory and disk space
        self._reset_inference_state()
        self._cleanup_temp_dir()

        return video_segments, original_frames

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

        if self.images_dir is None:
            raise RuntimeError("No images directory set. Call set_images_dir() first.")

        # Prepare just a few frames around the target (SAM3 needs some context)
        # We'll use 3 frames: target-1, target, target+1
        start = max(0, frame_idx - 1)
        num_frames = min(3, self.total_frames - start)
        local_idx = frame_idx - start

        frames_dir = self._prepare_frames_for_propagation(start, num_frames)
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
            frame_idx: Frame index
            points_by_obj: Dict of {obj_id: (points, labels)}

        Returns:
            Dict of {obj_id: mask_at_original_resolution}
        """
        if self.images_dir is None:
            raise RuntimeError("No images directory set. Call set_images_dir() first.")

        # Prepare just a few frames around the target
        start = max(0, frame_idx - 1)
        num_frames = min(3, self.total_frames - start)
        local_idx = frame_idx - start

        frames_dir = self._prepare_frames_for_propagation(start, num_frames)
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
