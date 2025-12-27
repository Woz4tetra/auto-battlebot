"""SAM3 video segmentation tracker integration with memory-efficient frame loading."""

import os
import shutil
import tempfile
import cv2
import numpy as np
import torch
import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from tqdm import tqdm

# Import SAM3 components
try:
    from sam3.model_builder import build_sam3_video_model
except ImportError:
    print("Warning: SAM3 not installed. Install with: pip install sam3")
    build_sam3_video_model = None


def _worker_propagate(
    gpu_id: int,
    inference_scale: float,
    video_path: str,
    video_width: int,
    video_height: int,
    frames_dir: str,
    start_frame: int,
    num_frames: int,
    local_start_idx: int,
    points_by_obj: Dict[int, Tuple[np.ndarray, np.ndarray]],
) -> Dict[int, Dict[int, np.ndarray]]:
    """
    Worker function for multi-GPU propagation.

    This runs in a separate process with its own GPU.
    """
    if build_sam3_video_model is None:
        raise RuntimeError("SAM3 not installed")

    # Set up device
    device = torch.device(f"cuda:{gpu_id}")
    torch.cuda.set_device(gpu_id)

    # Enable optimizations
    torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
    if torch.cuda.get_device_properties(gpu_id).major >= 8:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True

    print(f"[GPU {gpu_id}] Loading SAM3 model...")
    model = build_sam3_video_model()
    predictor = model.tracker
    predictor.backbone = model.detector.backbone

    # Initialize inference state
    print(
        f"[GPU {gpu_id}] Initializing inference state for frames {start_frame}-{start_frame + num_frames}..."
    )
    inference_state = predictor.init_state(video_path=frames_dir)

    # Add points for each object (at local frame index 0)
    for obj_id, (points, labels) in points_by_obj.items():
        if len(points) > 0:
            points_tensor = torch.tensor(points, dtype=torch.float32)
            labels_tensor = torch.tensor(labels, dtype=torch.int32)

            predictor.add_new_points(
                inference_state=inference_state,
                frame_idx=local_start_idx,
                obj_id=obj_id,
                points=points_tensor,
                labels=labels_tensor,
                clear_old_points=True,
            )

    # Propagate
    video_segments = {}

    for (
        local_frame_idx,
        obj_ids,
        low_res_masks,
        video_res_masks,
        obj_scores,
    ) in predictor.propagate_in_video(
        inference_state,
        start_frame_idx=0,
        max_frame_num_to_track=num_frames,
        reverse=False,
        propagate_preflight=True,
    ):
        original_frame_idx = start_frame + local_frame_idx

        video_segments[original_frame_idx] = {}
        for i, out_obj_id in enumerate(obj_ids):
            mask = (video_res_masks[i] > 0.0).cpu().numpy()

            # Scale mask to original resolution
            if inference_scale != 1.0:
                if mask.ndim == 3:
                    mask = mask.squeeze()
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (video_width, video_height),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)

            video_segments[original_frame_idx][out_obj_id] = mask

    print(f"[GPU {gpu_id}] Completed {len(video_segments)} frames")
    return video_segments


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

        pbar = tqdm(
            self.predictor.propagate_in_video(
                self.inference_state,
                start_frame_idx=0,
                max_frame_num_to_track=max_frames,
                reverse=False,
                propagate_preflight=True,
            ),
            total=max_frames,
            desc="Propagating",
        )

        for (
            local_frame_idx,
            obj_ids,
            low_res_masks,
            video_res_masks,
            obj_scores,
        ) in pbar:
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

        # Clean up temp directory to free disk space
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

        # Clean up
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

        # Clean up
        self._cleanup_temp_dir()

        return masks

    def cleanup(self):
        """Clean up all resources."""
        self._cleanup_temp_dir()
        self.inference_state = None

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()


class MultiGPUTracker:
    """
    Multi-GPU SAM3 tracker that distributes work across multiple GPUs.

    For propagation, the frame range is split across GPUs, with each GPU
    processing a portion of the frames in parallel.
    """

    def __init__(self, gpu_ids: List[int], inference_scale: float = 0.5):
        """
        Initialize the multi-GPU tracker.

        Args:
            gpu_ids: List of GPU device IDs to use
            inference_scale: Scale factor for frames during inference
        """
        self.gpu_ids = gpu_ids
        self.inference_scale = inference_scale
        self.num_gpus = len(gpu_ids)

        # Video info
        self.video_path: Optional[str] = None
        self.video_width: int = 0
        self.video_height: int = 0
        self.total_frames: int = 0
        self.fps: float = 0.0

        # Scaled dimensions
        self.scaled_width: int = 0
        self.scaled_height: int = 0

        # Primary tracker for single-GPU operations (preview, etc.)
        self._primary_tracker: Optional[SAM3Tracker] = None

        # Temp directories for multi-GPU work
        self._temp_dirs: List[str] = []

        print(f"MultiGPUTracker initialized with {self.num_gpus} GPUs: {gpu_ids}")

    def _get_primary_tracker(self) -> SAM3Tracker:
        """Get or create the primary tracker for single-GPU operations."""
        if self._primary_tracker is None:
            self._primary_tracker = SAM3Tracker(
                gpu_id=self.gpu_ids[0],
                inference_scale=self.inference_scale,
            )
            if self.video_path:
                self._primary_tracker.set_video(self.video_path)
        return self._primary_tracker

    def set_video(self, video_path: str):
        """
        Set the video file to work with.

        Args:
            video_path: Path to the video file
        """
        self.video_path = video_path

        # Load video metadata
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
        self.scaled_width = self.scaled_width - (self.scaled_width % 2)
        self.scaled_height = self.scaled_height - (self.scaled_height % 2)

        print(
            f"Video: {self.video_width}x{self.video_height}, {self.total_frames} frames"
        )
        print(
            f"Inference scale: {self.inference_scale} -> {self.scaled_width}x{self.scaled_height}"
        )

        # Update primary tracker if exists
        if self._primary_tracker:
            self._primary_tracker.set_video(video_path)

    def _cleanup_temp_dirs(self):
        """Clean up all temporary directories."""
        for temp_dir in self._temp_dirs:
            if temp_dir and os.path.exists(temp_dir):
                shutil.rmtree(temp_dir, ignore_errors=True)
        self._temp_dirs = []

    def _extract_frames_for_chunk(
        self,
        start_frame: int,
        num_frames: int,
    ) -> str:
        """Extract frames for a chunk to a temporary directory."""
        temp_dir = tempfile.mkdtemp(prefix=f"sam3_chunk_{start_frame}_")
        self._temp_dirs.append(temp_dir)

        cap = cv2.VideoCapture(self.video_path)
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

        end_frame = min(start_frame + num_frames, self.total_frames)

        for frame_idx in range(start_frame, end_frame):
            ret, frame = cap.read()
            if not ret:
                break

            if self.inference_scale != 1.0:
                frame = cv2.resize(
                    frame,
                    (self.scaled_width, self.scaled_height),
                    interpolation=cv2.INTER_AREA,
                )

            local_idx = frame_idx - start_frame
            frame_path = os.path.join(temp_dir, f"{local_idx:06d}.jpg")
            cv2.imwrite(frame_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

        cap.release()
        return temp_dir

    def propagate_from_frame(
        self,
        source_frame: int,
        points_by_obj: Dict[int, Tuple[np.ndarray, np.ndarray]],
        propagate_length: int,
        callback=None,
    ) -> Dict[int, Dict[int, np.ndarray]]:
        """
        Propagate masks using multiple GPUs in parallel.

        The frame range is split into chunks, with each GPU processing
        a chunk. Results are merged at the end.

        Args:
            source_frame: Frame index to start from
            points_by_obj: Dict of {obj_id: (points, labels)}
            propagate_length: Number of frames to propagate
            callback: Progress callback

        Returns:
            Dict of {frame_idx: {obj_id: mask}}
        """
        if self.video_path is None:
            raise RuntimeError("No video set. Call set_video() first.")

        # For single GPU, use the simple tracker
        if self.num_gpus == 1:
            tracker = self._get_primary_tracker()
            return tracker.propagate_from_frame(
                source_frame, points_by_obj, propagate_length, callback
            )

        # Clamp propagate_length
        max_frames = min(propagate_length, self.total_frames - source_frame)

        # Calculate chunk sizes for each GPU
        # We need overlap at boundaries for smooth propagation
        # First GPU starts from source_frame, others start from their chunk
        base_chunk_size = max_frames // self.num_gpus

        chunks = []
        current_frame = source_frame

        for i, gpu_id in enumerate(self.gpu_ids):
            if i == self.num_gpus - 1:
                # Last GPU gets remaining frames
                chunk_frames = max_frames - (current_frame - source_frame)
            else:
                chunk_frames = base_chunk_size

            if chunk_frames > 0:
                chunks.append(
                    {
                        "gpu_id": gpu_id,
                        "start_frame": current_frame,
                        "num_frames": chunk_frames,
                        "local_start_idx": 0 if i == 0 else 0,
                    }
                )

            current_frame += chunk_frames

        print(f"Distributing {max_frames} frames across {len(chunks)} GPUs:")
        for chunk in chunks:
            print(
                f"  GPU {chunk['gpu_id']}: frames {chunk['start_frame']}-{chunk['start_frame'] + chunk['num_frames'] - 1}"
            )

        # Clean up previous temp dirs
        self._cleanup_temp_dirs()

        # Extract frames for each chunk
        print("Extracting frames for each GPU...")
        for chunk in tqdm(chunks, desc="Preparing chunks"):
            chunk["frames_dir"] = self._extract_frames_for_chunk(
                chunk["start_frame"],
                chunk["num_frames"],
            )

        # Convert points to serializable format
        serializable_points = {
            obj_id: (pts.tolist(), lbls.tolist())
            for obj_id, (pts, lbls) in points_by_obj.items()
        }

        # Use multiprocessing to run on multiple GPUs
        # Note: We use spawn context to ensure clean CUDA state
        ctx = mp.get_context("spawn")

        print("Running propagation on multiple GPUs...")

        all_segments = {}

        with ProcessPoolExecutor(max_workers=self.num_gpus, mp_context=ctx) as executor:
            futures = []

            for chunk in chunks:
                # Convert back to numpy for the worker
                np_points = {
                    obj_id: (np.array(pts), np.array(lbls))
                    for obj_id, (pts, lbls) in serializable_points.items()
                }

                future = executor.submit(
                    _worker_propagate,
                    chunk["gpu_id"],
                    self.inference_scale,
                    self.video_path,
                    self.video_width,
                    self.video_height,
                    chunk["frames_dir"],
                    chunk["start_frame"],
                    chunk["num_frames"],
                    chunk["local_start_idx"],
                    np_points,
                )
                futures.append((chunk, future))

            # Collect results
            for chunk, future in tqdm(futures, desc="Collecting results"):
                try:
                    result = future.result()
                    all_segments.update(result)
                except Exception as e:
                    print(f"Error on GPU {chunk['gpu_id']}: {e}")
                    raise

        # Clean up temp directories
        self._cleanup_temp_dirs()

        print(f"Completed propagation: {len(all_segments)} frames total")

        if callback:
            callback(source_frame + max_frames - 1, 1.0)

        return all_segments

    def get_preview_mask(
        self,
        frame_idx: int,
        obj_id: int,
        points: np.ndarray,
        labels: np.ndarray,
    ) -> Optional[np.ndarray]:
        """Get preview mask using primary tracker."""
        return self._get_primary_tracker().get_preview_mask(
            frame_idx, obj_id, points, labels
        )

    def get_multi_object_preview(
        self,
        frame_idx: int,
        points_by_obj: Dict[int, Tuple[np.ndarray, np.ndarray]],
    ) -> Dict[int, np.ndarray]:
        """Get multi-object preview using primary tracker."""
        return self._get_primary_tracker().get_multi_object_preview(
            frame_idx, points_by_obj
        )

    def cleanup(self):
        """Clean up all resources."""
        self._cleanup_temp_dirs()
        if self._primary_tracker:
            self._primary_tracker.cleanup()
            self._primary_tracker = None

    def __del__(self):
        """Destructor."""
        self.cleanup()
