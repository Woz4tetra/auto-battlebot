"""Server configuration management."""

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml


@dataclass
class ObjectLabel:
    """Configuration for a tracked object label."""

    id: int
    name: str
    color: List[int] = field(default_factory=lambda: [255, 0, 0])  # RGB


@dataclass
class ServerConfig:
    """Server configuration loaded from YAML file."""

    # Video settings
    video_path: str

    # Object labels to track
    object_labels: List[ObjectLabel]

    # Output paths
    manual_annotations_dir: str
    generated_annotations_dir: str

    # Propagation settings
    propagate_length: int = 300  # Number of frames to propagate forward

    # Server settings
    host: str = "0.0.0.0"
    port: int = 7547

    # GPU settings - can use single gpu_id or list of gpu_ids for multi-GPU
    gpu_ids: List[int] = field(default_factory=lambda: [0])

    # Mask overlay settings
    mask_alpha: float = 0.5

    # Inference scale - scale down frames for SAM3 inference to save GPU memory
    # e.g., 0.5 = half resolution, 1.0 = full resolution
    # Masks are scaled back to original size after inference
    inference_scale: float = 0.5

    @classmethod
    def from_yaml(cls, config_path: str) -> "ServerConfig":
        """Load configuration from YAML file."""
        with open(config_path, "r") as f:
            data = yaml.safe_load(f)

        # Parse object labels
        object_labels = []
        for label_data in data.get("object_labels", []):
            label = ObjectLabel(
                id=label_data["id"],
                name=label_data["name"],
                color=label_data.get("color", [255, 0, 0]),
            )
            object_labels.append(label)

        # Ensure directories exist
        manual_dir = data.get("manual_annotations_dir", "./manual_annotations")
        generated_dir = data.get("generated_annotations_dir", "./generated_annotations")

        # Parse GPU IDs - support both single gpu_id and list gpu_ids
        if "gpu_ids" in data:
            gpu_ids = data["gpu_ids"]
        elif "gpu_id" in data:
            gpu_ids = [data["gpu_id"]]
        else:
            gpu_ids = [0]

        return cls(
            video_path=data["video_path"],
            object_labels=object_labels,
            manual_annotations_dir=manual_dir,
            generated_annotations_dir=generated_dir,
            propagate_length=data.get("propagate_length", 300),
            host=data.get("host", "0.0.0.0"),
            port=data.get("port", 7547),
            gpu_ids=gpu_ids,
            mask_alpha=data.get("mask_alpha", 0.5),
            inference_scale=data.get("inference_scale", 0.5),
        )

    def ensure_directories(self):
        """Create output directories if they don't exist."""
        Path(self.manual_annotations_dir).mkdir(parents=True, exist_ok=True)
        Path(self.generated_annotations_dir).mkdir(parents=True, exist_ok=True)

        # Create subdirectories for images and annotations
        Path(self.manual_annotations_dir, "images").mkdir(exist_ok=True)
        Path(self.manual_annotations_dir, "annotations").mkdir(exist_ok=True)
        Path(self.generated_annotations_dir, "images").mkdir(exist_ok=True)
        Path(self.generated_annotations_dir, "annotations").mkdir(exist_ok=True)
        Path(self.generated_annotations_dir, "masks").mkdir(exist_ok=True)

    def get_label_by_id(self, label_id: int) -> Optional[ObjectLabel]:
        """Get object label by ID."""
        for label in self.object_labels:
            if label.id == label_id:
                return label
        return None

    def get_label_by_name(self, name: str) -> Optional[ObjectLabel]:
        """Get object label by name."""
        for label in self.object_labels:
            if label.name == name:
                return label
        return None

    def to_dict(self) -> Dict:
        """Convert config to dictionary for API responses."""
        return {
            "video_path": self.video_path,
            "object_labels": [
                {"id": l.id, "name": l.name, "color": l.color}
                for l in self.object_labels
            ],
            "propagate_length": self.propagate_length,
            "mask_alpha": self.mask_alpha,
            "inference_scale": self.inference_scale,
        }
