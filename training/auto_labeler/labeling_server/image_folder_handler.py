"""Image folder handler for frame access."""

import cv2
import json
import numpy as np
from pathlib import Path
from typing import Optional, Tuple, List
from natsort import natsorted


class ImageFolderHandler:
    """Handles image folder operations - frame loading, metadata, etc."""

    # Supported image extensions
    SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp"}

    def __init__(self, images_dir: str):
        self.images_dir = Path(images_dir)
        self._validate_folder()
        self._load_image_list()
        self._load_metadata()

    def _validate_folder(self):
        """Validate that folder exists and contains images."""
        if not self.images_dir.exists():
            raise FileNotFoundError(f"Images directory not found: {self.images_dir}")

        if not self.images_dir.is_dir():
            raise ValueError(f"Path is not a directory: {self.images_dir}")

    def _load_image_list(self):
        """Load and sort list of image files."""
        self.image_files: List[Path] = []

        for ext in self.SUPPORTED_EXTENSIONS:
            self.image_files.extend(self.images_dir.glob(f"*{ext}"))
            self.image_files.extend(self.images_dir.glob(f"*{ext.upper()}"))

        # Natural sort to handle numeric naming correctly (1, 2, 10 not 1, 10, 2)
        self.image_files = natsorted(self.image_files, key=lambda p: p.name)

        if not self.image_files:
            raise ValueError(f"No images found in: {self.images_dir}")

        # Create index mapping: frame_idx -> image_path
        self._frame_to_path = {i: p for i, p in enumerate(self.image_files)}
        # Create reverse mapping: image_name -> frame_idx
        self._name_to_frame = {p.name: i for i, p in enumerate(self.image_files)}

    def _load_metadata(self):
        """Load metadata from first image."""
        first_image = cv2.imread(str(self.image_files[0]))
        if first_image is None:
            raise ValueError(f"Cannot read image: {self.image_files[0]}")

        self.height, self.width = first_image.shape[:2]
        self.total_frames = len(self.image_files)
        self.fps = 30.0  # Default fps for image sequences
        self.duration = self.total_frames / self.fps

    def get_frame(self, frame_idx: int) -> Optional[np.ndarray]:
        """
        Get a specific frame (image) by index.

        Args:
            frame_idx: Frame index (0-based)

        Returns:
            Frame as BGR numpy array, or None if frame cannot be read
        """
        if frame_idx < 0 or frame_idx >= self.total_frames:
            return None

        image_path = self._frame_to_path.get(frame_idx)
        if image_path is None:
            return None

        frame = cv2.imread(str(image_path))
        return frame

    def get_frame_rgb(self, frame_idx: int) -> Optional[np.ndarray]:
        """Get frame in RGB format."""
        frame = self.get_frame(frame_idx)
        if frame is not None:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return None

    def get_frame_jpeg(self, frame_idx: int, quality: int = 90) -> Optional[bytes]:
        """
        Get frame as JPEG bytes.

        Args:
            frame_idx: Frame index
            quality: JPEG quality (0-100)

        Returns:
            JPEG encoded bytes or None
        """
        frame = self.get_frame(frame_idx)
        if frame is not None:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, buffer = cv2.imencode(".jpg", frame, encode_param)
            return buffer.tobytes()
        return None

    def get_image_path(self, frame_idx: int) -> Optional[Path]:
        """Get the original image path for a frame index."""
        return self._frame_to_path.get(frame_idx)

    def get_image_name(self, frame_idx: int) -> Optional[str]:
        """Get the original image filename for a frame index."""
        path = self._frame_to_path.get(frame_idx)
        return path.name if path else None

    def get_frame_idx(self, image_name: str) -> Optional[int]:
        """Get frame index from image filename."""
        return self._name_to_frame.get(image_name)

    def get_metadata(self) -> dict:
        """Get folder metadata as dictionary."""
        return {
            "images_dir": str(self.images_dir),
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "total_frames": self.total_frames,
            "duration": self.duration,
        }

    def get_image_mapping(self) -> dict:
        """
        Get mapping between frame indices and original image filenames.

        Returns:
            Dict with frame_idx as key and image filename as value
        """
        return {idx: path.name for idx, path in self._frame_to_path.items()}

    def save_image_mapping(self, output_path: str) -> None:
        """
        Save image mapping to a JSON file.

        Args:
            output_path: Path to save the mapping file
        """
        mapping = self.get_image_mapping()
        with open(output_path, "w") as f:
            json.dump(mapping, f, indent=2)

    def overlay_mask(
        self,
        frame: np.ndarray,
        mask: np.ndarray,
        color: Tuple[int, int, int],
        alpha: float = 0.5,
    ) -> np.ndarray:
        """
        Overlay a colored mask on a frame.

        Args:
            frame: BGR frame
            mask: Binary mask (H, W) or (1, H, W)
            color: RGB color tuple
            alpha: Transparency (0-1)

        Returns:
            Frame with mask overlay
        """
        # Ensure mask is 2D
        if mask.ndim == 3:
            mask = mask.squeeze()

        mask_bool = mask.astype(bool)

        # Convert RGB color to BGR
        color_bgr = (color[2], color[1], color[0])

        # Create colored mask
        colored_mask = np.zeros_like(frame)
        colored_mask[mask_bool] = color_bgr

        # Blend with original frame
        result = frame.copy()
        result[mask_bool] = cv2.addWeighted(frame, 1 - alpha, colored_mask, alpha, 0)[
            mask_bool
        ]

        return result

    def overlay_masks(
        self, frame: np.ndarray, masks: dict, colors: dict, alpha: float = 0.5
    ) -> np.ndarray:
        """
        Overlay multiple masks on a frame.

        Args:
            frame: BGR frame
            masks: Dict of {obj_id: mask}
            colors: Dict of {obj_id: (R, G, B)}
            alpha: Transparency

        Returns:
            Frame with all masks overlaid
        """
        result = frame.copy()
        for obj_id, mask in masks.items():
            color = colors.get(obj_id, (255, 0, 0))
            result = self.overlay_mask(result, mask, color, alpha)
        return result

    def _get_contrasting_border_color(
        self, frame: np.ndarray, x: int, y: int, radius: int = 15
    ) -> Tuple[int, int, int]:
        """
        Get a border color that contrasts with the local background.

        Samples the area around the point and returns black or white
        based on the average luminance.
        """
        h, w = frame.shape[:2]

        # Define sampling region (clamped to frame bounds)
        x1 = max(0, x - radius)
        x2 = min(w, x + radius)
        y1 = max(0, y - radius)
        y2 = min(h, y + radius)

        # Sample the region
        region = frame[y1:y2, x1:x2]

        if region.size == 0:
            return (0, 0, 0)  # Default to black

        # Calculate average luminance (using standard luminance formula)
        # BGR format: 0.114*B + 0.587*G + 0.299*R
        avg_color = np.mean(region, axis=(0, 1))
        luminance = 0.114 * avg_color[0] + 0.587 * avg_color[1] + 0.299 * avg_color[2]

        # Return black for light backgrounds, white for dark backgrounds
        return (0, 0, 0) if luminance > 127 else (255, 255, 255)

    def draw_points(
        self,
        frame: np.ndarray,
        points: list,
        labels: list,
        obj_id: int = None,
        color: Tuple[int, int, int] = None,
    ) -> np.ndarray:
        """
        Draw point prompts on a frame.

        Args:
            frame: BGR frame
            points: List of (x, y) points
            labels: List of labels (1=positive, 0=negative)
            obj_id: Object ID for display
            color: Optional override color (RGB)

        Returns:
            Frame with points drawn
        """
        result = frame.copy()

        for point, label in zip(points, labels):
            x, y = int(point[0]), int(point[1])

            if color:
                pt_color = (color[2], color[1], color[0])  # RGB to BGR
            else:
                # Green for positive, red for negative
                pt_color = (0, 255, 0) if label == 1 else (0, 0, 255)

            # Get contrasting border color based on local background
            border_color = self._get_contrasting_border_color(frame, x, y)

            # Draw border/outline first (slightly larger)
            cv2.drawMarker(
                result,
                (x, y),
                border_color,
                cv2.MARKER_CROSS,
                markerSize=9,
                thickness=2,
            )
            cv2.circle(result, (x, y), 6, border_color, 2)

            # Draw marker on top
            cv2.drawMarker(
                result, (x, y), pt_color, cv2.MARKER_CROSS, markerSize=7, thickness=1
            )

            # Draw circle around marker
            cv2.circle(result, (x, y), 5, pt_color, 1)

            # Draw + or - sign with border
            sign = "+" if label == 1 else "-"
            # Border for text
            cv2.putText(
                result,
                sign,
                (x + 6, y - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.25,
                border_color,
                2,
            )
            # Text on top
            cv2.putText(
                result,
                sign,
                (x + 6, y - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.25,
                pt_color,
                1,
            )

        return result

    def close(self):
        """Release resources (no-op for image folder)."""
        pass

    def __del__(self):
        self.close()
