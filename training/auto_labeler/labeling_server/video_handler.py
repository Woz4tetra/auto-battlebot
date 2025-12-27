"""Video file handler for frame extraction."""

import cv2
import numpy as np
from pathlib import Path
from typing import Optional, Tuple
import threading


class VideoHandler:
    """Handles video file operations - frame extraction, metadata, etc."""
    
    def __init__(self, video_path: str):
        self.video_path = video_path
        self._validate_video()
        self._load_metadata()
        self._lock = threading.Lock()
        self._cap = None
    
    def _validate_video(self):
        """Validate that video file exists and is readable."""
        if not Path(self.video_path).exists():
            raise FileNotFoundError(f"Video file not found: {self.video_path}")
        
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise ValueError(f"Cannot open video file: {self.video_path}")
        cap.release()
    
    def _load_metadata(self):
        """Load video metadata."""
        cap = cv2.VideoCapture(self.video_path)
        
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        self.total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.duration = self.total_frames / self.fps if self.fps > 0 else 0
        
        cap.release()
    
    def _get_capture(self) -> cv2.VideoCapture:
        """Get or create video capture object."""
        if self._cap is None or not self._cap.isOpened():
            self._cap = cv2.VideoCapture(self.video_path)
        return self._cap
    
    def get_frame(self, frame_idx: int) -> Optional[np.ndarray]:
        """
        Get a specific frame from the video.
        
        Args:
            frame_idx: Frame index (0-based)
            
        Returns:
            Frame as BGR numpy array, or None if frame cannot be read
        """
        if frame_idx < 0 or frame_idx >= self.total_frames:
            return None
        
        with self._lock:
            cap = self._get_capture()
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            ret, frame = cap.read()
            
            if ret:
                return frame
            return None
    
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
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            return buffer.tobytes()
        return None
    
    def save_frame(self, frame_idx: int, output_path: str) -> bool:
        """
        Save a frame to disk.
        
        Args:
            frame_idx: Frame index
            output_path: Output file path
            
        Returns:
            True if successful
        """
        frame = self.get_frame(frame_idx)
        if frame is not None:
            cv2.imwrite(output_path, frame)
            return True
        return False
    
    def get_metadata(self) -> dict:
        """Get video metadata as dictionary."""
        return {
            'video_path': self.video_path,
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'total_frames': self.total_frames,
            'duration': self.duration,
        }
    
    def overlay_mask(
        self, 
        frame: np.ndarray, 
        mask: np.ndarray, 
        color: Tuple[int, int, int],
        alpha: float = 0.5
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
        result[mask_bool] = cv2.addWeighted(
            frame, 1 - alpha, colored_mask, alpha, 0
        )[mask_bool]
        
        return result
    
    def overlay_masks(
        self,
        frame: np.ndarray,
        masks: dict,
        colors: dict,
        alpha: float = 0.5
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
    
    def draw_points(
        self,
        frame: np.ndarray,
        points: list,
        labels: list,
        obj_id: int = None,
        color: Tuple[int, int, int] = None
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
            
            # Draw marker
            cv2.drawMarker(
                result, (x, y), pt_color,
                cv2.MARKER_CROSS, markerSize=15, thickness=2
            )
            
            # Draw circle around marker
            cv2.circle(result, (x, y), 10, pt_color, 2)
            
            # Draw + or - sign
            sign = "+" if label == 1 else "-"
            cv2.putText(
                result, sign, (x + 12, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, pt_color, 2
            )
        
        return result
    
    def close(self):
        """Release video capture resources."""
        with self._lock:
            if self._cap is not None:
                self._cap.release()
                self._cap = None
    
    def __del__(self):
        self.close()
