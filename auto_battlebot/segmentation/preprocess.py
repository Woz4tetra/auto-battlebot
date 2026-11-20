"""DeepLab field-mask preprocessing paths: the training pipeline and the deployed C++ one."""

from __future__ import annotations

import dataclasses
import enum

import cv2
import numpy as np
import torch

from auto_battlebot.segmentation.model_config import ModelConfig

DATASET_MEAN = (0.4611, 0.4359, 0.3905)
DATASET_STD = (0.2193, 0.2150, 0.2109)
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


class ChannelOrder(enum.Enum):
    RGB = "rgb"
    BGR = "bgr"


class Normalization(enum.Enum):
    DATASET = "dataset"
    IMAGENET = "imagenet"


class Geometry(enum.Enum):
    TRAINING = "training"
    DEPLOYED = "deployed"


class Resolution(enum.Enum):
    MODEL = "model"
    NATIVE = "native"


@dataclasses.dataclass(frozen=True)
class Preprocess:
    """How a BGR camera frame becomes a model input, and how the output maps back."""

    channels: ChannelOrder
    normalization: Normalization
    geometry: Geometry

    @classmethod
    def parse(cls, spec: str) -> Preprocess:
        """Parse `channels,normalization,geometry`, e.g. `bgr,imagenet,deployed`."""
        parts = [part.strip().lower() for part in spec.split(",")]
        if len(parts) != 3:
            raise ValueError(f"preprocess spec {spec!r} needs channels,normalization,geometry")
        return cls(ChannelOrder(parts[0]), Normalization(parts[1]), Geometry(parts[2]))

    @property
    def spec(self) -> str:
        return f"{self.channels.value},{self.normalization.value},{self.geometry.value}"

    @property
    def mean_std(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        if self.normalization is Normalization.DATASET:
            return DATASET_MEAN, DATASET_STD
        return IMAGENET_MEAN, IMAGENET_STD


TRAINING = Preprocess(ChannelOrder.RGB, Normalization.DATASET, Geometry.TRAINING)
DEPLOYED_UNTIL_2026_11 = Preprocess(ChannelOrder.BGR, Normalization.IMAGENET, Geometry.DEPLOYED)


def model_input(bgr: np.ndarray, config: ModelConfig, preprocess: Preprocess) -> torch.Tensor:
    """CHW float tensor for one 8-bit BGR frame."""
    image = bgr if preprocess.channels is ChannelOrder.BGR else cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    pad = config.pad_size
    if preprocess.geometry is Geometry.TRAINING:
        size = (config.image_size, config.image_size)
        image = cv2.resize(image, size, interpolation=cv2.INTER_NEAREST)
        image = cv2.copyMakeBorder(image, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=0)
    else:
        if pad > 0:
            image = cv2.copyMakeBorder(image, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
        image = cv2.resize(image, (config.input_size, config.input_size))
    mean, std = preprocess.mean_std
    scaled = image.astype(np.float32) / 255.0
    normalized = (scaled - np.array(mean, np.float32)) / np.array(std, np.float32)
    return torch.from_numpy(np.ascontiguousarray(normalized.transpose(2, 0, 1)))


def output_labels(
    labels: np.ndarray,
    native_shape: tuple[int, int],
    config: ModelConfig,
    preprocess: Preprocess,
    resolution: Resolution,
) -> np.ndarray:
    """Map an input-sized label map back to the frame, at model or native resolution."""
    pad = config.pad_size
    height, width = native_shape
    if preprocess.geometry is Geometry.TRAINING:
        cropped = (
            labels[pad : labels.shape[0] - pad, pad : labels.shape[1] - pad] if pad else labels
        )
        if resolution is Resolution.MODEL:
            return cropped
        return cv2.resize(cropped, (width, height), interpolation=cv2.INTER_NEAREST)
    padded = cv2.resize(
        labels, (width + 2 * pad, height + 2 * pad), interpolation=cv2.INTER_NEAREST
    )
    native = padded[pad : pad + height, pad : pad + width] if pad else padded
    if resolution is Resolution.NATIVE:
        return native
    size = (config.image_size, config.image_size)
    return cv2.resize(native, size, interpolation=cv2.INTER_NEAREST)
