"""API client for communicating with the labeling server."""

import io
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import requests
from PIL import Image


@dataclass
class ObjectLabel:
    """Object label configuration."""

    id: int
    name: str
    color: Tuple[int, int, int]


@dataclass
class VideoMetadata:
    """Video metadata."""

    path: str
    width: int
    height: int
    fps: float
    total_frames: int
    duration: float


@dataclass
class ServerConfig:
    """Server configuration received from API."""

    object_labels: List[ObjectLabel]
    propagate_length: int
    mask_alpha: float
    video: VideoMetadata


@dataclass
class FrameInfo:
    """Information about a frame."""

    frame_idx: int
    is_manual: bool
    has_masks: bool
    propagated_from: Optional[int]
    points: Dict[int, List[dict]]
    mask_object_ids: List[int]


@dataclass
class AnnotationStatus:
    """Annotation status from server."""

    total_annotated_frames: int
    manual_frames: int
    generated_frames: int
    manual_frame_indices: List[int]


class LabelingAPIClient:
    """Client for communicating with the labeling server."""

    def __init__(self, server_url: str):
        """
        Initialize the API client.

        Args:
            server_url: Base URL of the labeling server (e.g., "http://192.168.1.100:8765")
        """
        self.server_url = server_url.rstrip("/")
        self._session = requests.Session()
        self._config: Optional[ServerConfig] = None

    def _url(self, endpoint: str) -> str:
        """Build full URL for an endpoint."""
        return f"{self.server_url}/api/{endpoint.lstrip('/')}"

    def _get(self, endpoint: str, **params) -> dict:
        """Make a GET request."""
        response = self._session.get(self._url(endpoint), params=params)
        response.raise_for_status()
        return response.json()

    def _post(self, endpoint: str, data: dict = None) -> dict:
        """Make a POST request."""
        response = self._session.post(self._url(endpoint), json=data or {})
        response.raise_for_status()
        return response.json()

    def _get_image(self, endpoint: str, **params) -> Image.Image:
        """Get an image from an endpoint."""
        response = self._session.get(self._url(endpoint), params=params)
        response.raise_for_status()
        return Image.open(io.BytesIO(response.content))

    def connect(self) -> bool:
        """
        Test connection to server and load configuration.

        Returns:
            True if connection successful
        """
        try:
            data = self._get("config")

            # Parse object labels
            labels = [
                ObjectLabel(
                    id=l["id"],
                    name=l["name"],
                    color=tuple(l["color"]),
                )
                for l in data["object_labels"]
            ]

            # Parse video metadata
            video = VideoMetadata(
                path=data["video"]["video_path"],
                width=data["video"]["width"],
                height=data["video"]["height"],
                fps=data["video"]["fps"],
                total_frames=data["video"]["total_frames"],
                duration=data["video"]["duration"],
            )

            self._config = ServerConfig(
                object_labels=labels,
                propagate_length=data["propagate_length"],
                mask_alpha=data["mask_alpha"],
                video=video,
            )

            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    @property
    def config(self) -> ServerConfig:
        """Get cached server configuration."""
        if self._config is None:
            raise RuntimeError("Not connected. Call connect() first.")
        return self._config

    def get_status(self) -> AnnotationStatus:
        """Get annotation status."""
        data = self._get("status")
        return AnnotationStatus(
            total_annotated_frames=data["total_annotated_frames"],
            manual_frames=data["manual_frames"],
            generated_frames=data["generated_frames"],
            manual_frame_indices=data["manual_frame_indices"],
        )

    def get_frame(
        self,
        frame_idx: int,
        include_masks: bool = False,
        include_points: bool = False,
        quality: int = 90,
    ) -> Image.Image:
        """
        Get a frame image from the server.

        Args:
            frame_idx: Frame index
            include_masks: Whether to overlay masks
            include_points: Whether to draw points
            quality: JPEG quality

        Returns:
            PIL Image
        """
        params = {
            "masks": str(include_masks).lower(),
            "points": str(include_points).lower(),
            "quality": quality,
        }
        return self._get_image(f"frame/{frame_idx}", **params)

    def get_frame_info(self, frame_idx: int) -> FrameInfo:
        """Get information about a frame."""
        data = self._get(f"frame/{frame_idx}/info")
        return FrameInfo(
            frame_idx=data["frame_idx"],
            is_manual=data["is_manual"],
            has_masks=data["has_masks"],
            propagated_from=data["propagated_from"],
            points=data["points"],
            mask_object_ids=data["mask_object_ids"],
        )

    def add_point(
        self,
        frame_idx: int,
        x: float,
        y: float,
        label: int,
        obj_id: int,
    ) -> dict:
        """
        Add a point prompt.

        Args:
            frame_idx: Frame index
            x: X coordinate in image pixels
            y: Y coordinate in image pixels
            label: 1 for positive (include), 0 for negative (exclude)
            obj_id: Object ID

        Returns:
            Response dict with success status
        """
        return self._post(
            "points",
            {
                "frame_idx": frame_idx,
                "x": x,
                "y": y,
                "label": label,
                "obj_id": obj_id,
            },
        )

    def undo_point(self, frame_idx: int) -> dict:
        """Undo the last point on a frame."""
        return self._post("points/undo", {"frame_idx": frame_idx})

    def clear_points(self, frame_idx: int, obj_id: Optional[int] = None) -> dict:
        """Clear points from a frame, optionally for a specific object."""
        data = {"frame_idx": frame_idx}
        if obj_id is not None:
            data["obj_id"] = obj_id
        return self._post("points/clear", data)

    def delete_point(self, frame_idx: int, point_index: int) -> dict:
        """Delete a specific point by index."""
        return self._post(
            "points/delete",
            {
                "frame_idx": frame_idx,
                "point_index": point_index,
            },
        )

    def get_preview(self, frame_idx: int, quality: int = 85) -> Image.Image:
        """Get a preview of the mask for current points."""
        response = self._session.post(
            self._url("preview"),
            json={"frame_idx": frame_idx, "quality": quality},
        )
        response.raise_for_status()
        return Image.open(io.BytesIO(response.content))

    def propagate(self, frame_idx: int, length: Optional[int] = None) -> dict:
        """
        Propagate masks from a frame.

        Args:
            frame_idx: Starting frame index
            length: Number of frames to propagate (uses server default if None)

        Returns:
            Response dict with propagation results
        """
        data = {"frame_idx": frame_idx}
        if length is not None:
            data["length"] = length
        return self._post("propagate", data)

    def get_next_unlabeled(self, current: int) -> Optional[int]:
        """Get the next frame that needs labeling."""
        data = self._get("navigation/next-unlabeled", current=current)
        if data["found"]:
            return data["frame_idx"]
        return None

    def get_manual_frames(self) -> List[int]:
        """Get list of manually labeled frames."""
        data = self._get("navigation/manual-frames")
        return data["frames"]

    def get_prev_manual(self, current: int) -> Optional[int]:
        """Get previous manually labeled frame."""
        data = self._get("navigation/prev-manual", current=current)
        if data["found"]:
            return data["frame_idx"]
        return None

    def get_next_manual(self, current: int) -> Optional[int]:
        """Get next manually labeled frame."""
        data = self._get("navigation/next-manual", current=current)
        if data["found"]:
            return data["frame_idx"]
        return None

    def save(self) -> dict:
        """Force save current state."""
        return self._post("save")

    def export_coco(self) -> dict:
        """Export annotations in COCO format."""
        return self._get("export/coco")
