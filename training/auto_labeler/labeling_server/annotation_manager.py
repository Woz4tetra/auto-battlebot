"""Annotation management with COCO format support."""

import json
import os
import cv2
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import traceback
from tqdm import tqdm


@dataclass
class PointPrompt:
    """A single point prompt for segmentation."""

    x: float
    y: float
    label: int  # 1 = positive (include), 0 = negative (exclude)
    obj_id: int


@dataclass
class FrameAnnotation:
    """Annotation data for a single frame."""

    frame_idx: int
    points: List[PointPrompt] = field(default_factory=list)
    is_manual: bool = False  # True if user manually annotated this frame
    masks: Dict[int, np.ndarray] = field(default_factory=dict)  # obj_id -> mask
    propagated_from: Optional[int] = None  # Frame index this was propagated from


class AnnotationManager:
    """Manages annotations in COCO format."""

    def __init__(
        self,
        manual_dir: str,
        generated_dir: str,
        video_width: int,
        video_height: int,
        object_labels: List[dict],
        output_width: int = 0,
    ):
        self.manual_dir = Path(manual_dir)
        self.generated_dir = Path(generated_dir)
        self.video_width = video_width
        self.video_height = video_height
        self.object_labels = object_labels

        # Calculate output dimensions
        if output_width > 0 and output_width < video_width:
            self.output_scale = output_width / video_width
            self.output_width = output_width
            self.output_height = int(video_height * self.output_scale)
            # Ensure even dimensions
            self.output_height = self.output_height - (self.output_height % 2)
        else:
            self.output_scale = 1.0
            self.output_width = video_width
            self.output_height = video_height

        # Frame annotations: frame_idx -> FrameAnnotation
        self.annotations: Dict[int, FrameAnnotation] = {}

        # Track which frames have manual annotations
        self.manual_frames: set = set()

        # Track which frames have generated masks
        self.generated_frames: set = set()

        # State file for session persistence
        self.state_file = self.manual_dir / "annotations" / "session_state.json"
        self.coco_file = self.generated_dir / "annotations" / "annotations.json"

        # Initialize COCO structure
        self._init_coco()

    def _init_coco(self):
        """Initialize COCO annotation structure."""
        self.coco = {
            "info": {
                "description": "Video Labeling Annotations",
                "date_created": datetime.now().isoformat(),
                "version": "1.0",
            },
            "licenses": [],
            "images": [],
            "annotations": [],
            "categories": [
                {
                    "id": label["id"],
                    "name": label["name"],
                    "supercategory": "object",
                }
                for label in self.object_labels
            ],
        }
        self._image_id_counter = 0
        self._annotation_id_counter = 0

    def load_state(self) -> bool:
        """Load annotation state from disk."""
        if not self.state_file.exists():
            return False

        try:
            with open(self.state_file, "r") as f:
                state = json.load(f)

            # Load manual annotations
            for frame_data in state.get("frames", []):
                frame_idx = frame_data["frame_idx"]
                points = [
                    PointPrompt(
                        x=p["x"], y=p["y"], label=p["label"], obj_id=p["obj_id"]
                    )
                    for p in frame_data.get("points", [])
                ]

                self.annotations[frame_idx] = FrameAnnotation(
                    frame_idx=frame_idx,
                    points=points,
                    is_manual=frame_data.get("is_manual", False),
                    propagated_from=frame_data.get("propagated_from"),
                )

                if frame_data.get("is_manual", False):
                    self.manual_frames.add(frame_idx)

            # Load generated masks info
            self.generated_frames = set(state.get("generated_frames", []))

            # Load COCO annotations if they exist
            if self.coco_file.exists():
                with open(self.coco_file, "r") as f:
                    self.coco = json.load(f)
                self._image_id_counter = max(
                    [img["id"] for img in self.coco.get("images", [])] + [0]
                )
                self._annotation_id_counter = max(
                    [ann["id"] for ann in self.coco.get("annotations", [])] + [0]
                )

            # Load mask images from disk for generated frames
            self._load_masks_from_disk()

            return True

        except Exception as e:
            print(f"Error loading state: {e}")
            traceback.print_exc()
            return False

    def _load_masks_from_disk(self):
        """Load saved mask images from disk into memory."""
        masks_dir = self.generated_dir / "masks"
        print(f"Looking for masks in: {masks_dir.resolve()}")
        if not masks_dir.exists():
            print("Masks directory does not exist")
            return

        # Get all object IDs from labels
        obj_ids = [label["id"] for label in self.object_labels]
        print(
            f"Loading masks for {len(self.generated_frames)} frames, {len(obj_ids)} objects each"
        )

        # Build list of all mask files to load
        mask_tasks = []
        for frame_idx in sorted(self.generated_frames):
            for obj_id in obj_ids:
                mask_path = masks_dir / f"frame_{frame_idx:06d}_obj_{obj_id}.png"
                if mask_path.exists():
                    mask_tasks.append((frame_idx, obj_id, mask_path))

        if not mask_tasks:
            print("No mask files found to load")
            return

        def load_single_mask(task):
            """Load a single mask from disk."""
            frame_idx, obj_id, mask_path = task
            mask_img = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if mask_img is not None:
                mask = (mask_img > 127).astype(bool)
                return (frame_idx, obj_id, mask)
            return None

        # Use ThreadPoolExecutor for parallel I/O
        loaded_count = 0
        num_workers = min(os.cpu_count() or 4, 16)  # Cap at 16 workers

        with ThreadPoolExecutor(max_workers=num_workers) as executor:
            futures = {
                executor.submit(load_single_mask, task): task for task in mask_tasks
            }

            for future in tqdm(
                as_completed(futures),
                total=len(futures),
                desc="Loading masks from disk",
                unit="mask",
            ):
                result = future.result()
                if result is not None:
                    frame_idx, obj_id, mask = result

                    # Ensure frame annotation exists
                    if frame_idx not in self.annotations:
                        self.annotations[frame_idx] = FrameAnnotation(
                            frame_idx=frame_idx
                        )

                    self.annotations[frame_idx].masks[obj_id] = mask
                    loaded_count += 1

        print(f"Loaded {loaded_count} masks from disk using {num_workers} workers")

    def save_state(self):
        """Save annotation state to disk."""
        state = {
            "frames": [],
            "generated_frames": list(self.generated_frames),
            "saved_at": datetime.now().isoformat(),
        }

        for frame_idx, ann in self.annotations.items():
            frame_data = {
                "frame_idx": frame_idx,
                "points": [
                    {"x": p.x, "y": p.y, "label": p.label, "obj_id": p.obj_id}
                    for p in ann.points
                ],
                "is_manual": ann.is_manual,
                "propagated_from": ann.propagated_from,
            }
            state["frames"].append(frame_data)

        # Ensure directory exists
        self.state_file.parent.mkdir(parents=True, exist_ok=True)

        with open(self.state_file, "w") as f:
            json.dump(state, f, indent=2)

        # Save COCO annotations
        self.coco_file.parent.mkdir(parents=True, exist_ok=True)
        with open(self.coco_file, "w") as f:
            json.dump(self.coco, f, indent=2)

    def add_point(
        self,
        frame_idx: int,
        x: float,
        y: float,
        label: int,
        obj_id: int,
    ) -> FrameAnnotation:
        """Add a point prompt to a frame."""
        if frame_idx not in self.annotations:
            self.annotations[frame_idx] = FrameAnnotation(frame_idx=frame_idx)

        ann = self.annotations[frame_idx]
        ann.points.append(PointPrompt(x=x, y=y, label=label, obj_id=obj_id))
        ann.is_manual = True
        self.manual_frames.add(frame_idx)

        return ann

    def remove_point(
        self,
        frame_idx: int,
        point_index: int,
    ) -> Optional[FrameAnnotation]:
        """Remove a point prompt by index."""
        if frame_idx not in self.annotations:
            return None

        ann = self.annotations[frame_idx]
        if 0 <= point_index < len(ann.points):
            ann.points.pop(point_index)

        return ann

    def undo_last_point(self, frame_idx: int) -> Optional[FrameAnnotation]:
        """Remove the last added point from a frame."""
        if frame_idx not in self.annotations:
            return None

        ann = self.annotations[frame_idx]
        if ann.points:
            ann.points.pop()

        return ann

    def clear_points(
        self, frame_idx: int, obj_id: Optional[int] = None
    ) -> Optional[FrameAnnotation]:
        """Clear all points from a frame, optionally for a specific object."""
        if frame_idx not in self.annotations:
            return None

        ann = self.annotations[frame_idx]
        if obj_id is not None:
            ann.points = [p for p in ann.points if p.obj_id != obj_id]
        else:
            ann.points = []
            ann.is_manual = False
            self.manual_frames.discard(frame_idx)

        return ann

    def get_points(
        self, frame_idx: int, obj_id: Optional[int] = None
    ) -> List[PointPrompt]:
        """Get points for a frame, optionally filtered by object ID."""
        if frame_idx not in self.annotations:
            return []

        points = self.annotations[frame_idx].points
        if obj_id is not None:
            points = [p for p in points if p.obj_id == obj_id]

        return points

    def get_points_for_sam(
        self,
        frame_idx: int,
        obj_id: int,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get points formatted for SAM model (normalized coordinates)."""
        points = self.get_points(frame_idx, obj_id)

        if not points:
            return np.array([]), np.array([])

        pts = np.array(
            [[p.x / self.video_width, p.y / self.video_height] for p in points]
        )
        labels = np.array([p.label for p in points])

        return pts, labels

    def set_mask(
        self,
        frame_idx: int,
        obj_id: int,
        mask: np.ndarray,
        propagated_from: Optional[int] = None,
    ):
        """Set a mask for a frame/object."""
        if frame_idx not in self.annotations:
            self.annotations[frame_idx] = FrameAnnotation(frame_idx=frame_idx)

        ann = self.annotations[frame_idx]
        ann.masks[obj_id] = mask
        if propagated_from is not None:
            ann.propagated_from = propagated_from

    def get_mask(self, frame_idx: int, obj_id: int) -> Optional[np.ndarray]:
        """Get mask for a frame/object."""
        if frame_idx not in self.annotations:
            return None
        return self.annotations[frame_idx].masks.get(obj_id)

    def get_all_masks(self, frame_idx: int) -> Dict[int, np.ndarray]:
        """Get all masks for a frame."""
        if frame_idx not in self.annotations:
            return {}
        return self.annotations[frame_idx].masks

    def clear_propagated_masks(self, from_frame: int):
        """Clear all masks that were propagated from/after a given frame."""
        to_clear = []
        for frame_idx, ann in self.annotations.items():
            if frame_idx >= from_frame and not ann.is_manual:
                to_clear.append(frame_idx)
            elif ann.propagated_from is not None and ann.propagated_from >= from_frame:
                to_clear.append(frame_idx)

        for frame_idx in to_clear:
            if frame_idx in self.annotations:
                self.annotations[frame_idx].masks = {}
                self.generated_frames.discard(frame_idx)

    def mark_generated(self, frame_idx: int):
        """Mark a frame as having generated masks."""
        self.generated_frames.add(frame_idx)

    def has_masks(self, frame_idx: int) -> bool:
        """Check if a frame has any masks."""
        if frame_idx not in self.annotations:
            return False
        return bool(self.annotations[frame_idx].masks)

    def get_manual_frames(self) -> List[int]:
        """Get sorted list of frames with manual annotations."""
        return sorted(self.manual_frames)

    def get_next_unlabeled_frame(
        self, current_frame: int, total_frames: int
    ) -> Optional[int]:
        """
        Get the next frame that needs manual labeling.

        This returns frames that either:
        - Have no masks generated
        - Need manual verification
        """
        # First, find frames without any masks after current position
        for frame_idx in range(current_frame + 1, total_frames):
            if not self.has_masks(frame_idx):
                return frame_idx

        # If none found after, check before current position
        for frame_idx in range(0, current_frame):
            if not self.has_masks(frame_idx):
                return frame_idx

        return None

    def get_prev_manual_frame(self, current_frame: int) -> Optional[int]:
        """Get previous manually labeled frame."""
        manual = sorted(self.manual_frames, reverse=True)
        for frame_idx in manual:
            if frame_idx < current_frame:
                return frame_idx
        return None

    def get_next_manual_frame(self, current_frame: int) -> Optional[int]:
        """Get next manually labeled frame."""
        manual = sorted(self.manual_frames)
        for frame_idx in manual:
            if frame_idx > current_frame:
                return frame_idx
        return None

    def save_frame_image(
        self,
        frame_idx: int,
        frame: np.ndarray,
        is_manual: bool = False,
    ) -> str:
        """Save a frame image to disk, scaled to output dimensions."""
        if is_manual:
            output_dir = self.manual_dir / "images"
        else:
            output_dir = self.generated_dir / "images"

        output_dir.mkdir(parents=True, exist_ok=True)

        # Scale frame if needed
        if self.output_scale != 1.0:
            frame = cv2.resize(
                frame,
                (self.output_width, self.output_height),
                interpolation=cv2.INTER_AREA,
            )

        filename = f"frame_{frame_idx:06d}.jpg"
        output_path = output_dir / filename
        cv2.imwrite(str(output_path), frame)

        return str(output_path)

    def save_mask_image(
        self,
        frame_idx: int,
        obj_id: int,
        mask: np.ndarray,
    ) -> str:
        """Save a mask as PNG image, scaled to output dimensions."""
        output_dir = self.generated_dir / "masks"
        output_dir.mkdir(parents=True, exist_ok=True)

        filename = f"frame_{frame_idx:06d}_obj_{obj_id}.png"
        output_path = output_dir / filename

        # Save as binary mask (0 or 255)
        mask_img = (mask.squeeze() * 255).astype(np.uint8)

        # Scale mask if needed
        if self.output_scale != 1.0:
            mask_img = cv2.resize(
                mask_img,
                (self.output_width, self.output_height),
                interpolation=cv2.INTER_NEAREST,
            )

        cv2.imwrite(str(output_path), mask_img)

        return str(output_path)

    def _mask_to_polygons(
        self, mask: np.ndarray, epsilon_factor: float = 0.001
    ) -> Tuple[List[List[float]], float, List[float]]:
        """
        Convert a binary mask to polygon contours (COCO segmentation format).

        Coordinates are scaled to output dimensions.

        Args:
            mask: Binary mask array (at video resolution)
            epsilon_factor: Contour approximation factor (smaller = more points, more accurate)

        Returns:
            Tuple of (polygons, area, bbox) where:
            - polygons: List of [x1, y1, x2, y2, ...] coordinate lists (at output resolution)
            - area: Total mask area in pixels (at output resolution)
            - bbox: [x, y, width, height] bounding box (at output resolution)
        """
        mask_binary = mask.squeeze().astype(np.uint8)

        # Find contours
        contours, _ = cv2.findContours(
            mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return [], 0.0, [0, 0, 0, 0]

        polygons = []
        total_area = 0.0
        all_points = []

        for contour in contours:
            # Skip very small contours (noise)
            if cv2.contourArea(contour) < 10:
                continue

            # Approximate contour to reduce points
            epsilon = epsilon_factor * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Need at least 3 points for a valid polygon
            if len(approx) < 3:
                continue

            # Flatten to [x1, y1, x2, y2, ...] format and scale to output dimensions
            scaled_approx = approx.astype(np.float32) * self.output_scale
            polygon = scaled_approx.flatten().tolist()
            polygons.append(polygon)

            # Accumulate area (scaled)
            total_area += cv2.contourArea(contour) * (self.output_scale ** 2)

            # Collect points for bbox calculation (scaled)
            all_points.extend(scaled_approx.reshape(-1, 2).tolist())

        # Calculate bounding box from all points
        if all_points:
            all_points = np.array(all_points)
            x_min, y_min = all_points.min(axis=0)
            x_max, y_max = all_points.max(axis=0)
            bbox = [
                float(x_min),
                float(y_min),
                float(x_max - x_min),
                float(y_max - y_min),
            ]
        else:
            bbox = [0, 0, 0, 0]

        return polygons, total_area, bbox

    def add_to_coco(
        self,
        frame_idx: int,
        frame: np.ndarray,
        masks: Dict[int, np.ndarray],
    ):
        """Add frame and masks to COCO annotations."""
        # Save frame image
        image_path = self.save_frame_image(frame_idx, frame, is_manual=False)

        # Check if image already exists in COCO
        existing_img = None
        for img in self.coco["images"]:
            if img.get("frame_idx") == frame_idx:
                existing_img = img
                break

        if existing_img:
            image_id = existing_img["id"]
            # Remove old annotations for this image
            self.coco["annotations"] = [
                ann for ann in self.coco["annotations"] if ann["image_id"] != image_id
            ]
        else:
            self._image_id_counter += 1
            image_id = self._image_id_counter

            self.coco["images"].append(
                {
                    "id": image_id,
                    "file_name": os.path.basename(image_path),
                    "width": self.output_width,
                    "height": self.output_height,
                    "frame_idx": frame_idx,
                }
            )

        # Add annotations for each mask
        for obj_id, mask in masks.items():
            self._annotation_id_counter += 1

            # Save mask image
            self.save_mask_image(frame_idx, obj_id, mask)

            # Convert mask to polygon contours (much more compact than RLE)
            polygons, area, bbox = self._mask_to_polygons(mask)

            # Skip if no valid polygons
            if not polygons:
                continue

            self.coco["annotations"].append(
                {
                    "id": self._annotation_id_counter,
                    "image_id": image_id,
                    "category_id": obj_id,
                    "segmentation": polygons,
                    "bbox": bbox,
                    "area": area,
                    "iscrowd": 0,
                }
            )

        self.mark_generated(frame_idx)

    def get_status(self) -> dict:
        """Get annotation status summary."""
        return {
            "total_annotated_frames": len(self.annotations),
            "manual_frames": len(self.manual_frames),
            "generated_frames": len(self.generated_frames),
            "manual_frame_indices": sorted(self.manual_frames),
        }
