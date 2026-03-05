from pathlib import Path

import numpy as np
import torch
from torchvision.models.segmentation import (
    DeepLabV3,
    deeplabv3_mobilenet_v3_large,
    deeplabv3_resnet50,
    deeplabv3_resnet101,
)
from torchvision import transforms

from model_config import ModelConfig, load_model_config

BACKBONE_BUILDERS = {
    "mbv3": deeplabv3_mobilenet_v3_large,
    "r50": deeplabv3_resnet50,
    "r101": deeplabv3_resnet101,
}


def seed_everything(seed_value: int) -> None:
    np.random.seed(seed_value)
    torch.manual_seed(seed_value)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = True


def common_transforms(
    mean: tuple[float, float, float] = (0.4611, 0.4359, 0.3905),
    std: tuple[float, float, float] = (0.2193, 0.2150, 0.2109),
    pad_size: int = 20,
) -> transforms.Compose:
    return transforms.Compose(
        [
            transforms.ToPILImage(),
            transforms.Pad(pad_size),
            transforms.ToTensor(),
            transforms.Normalize(mean, std),
        ]
    )


def build_model(
    backbone: str, num_classes: int, device: torch.device
) -> DeepLabV3:
    if backbone not in BACKBONE_BUILDERS:
        raise ValueError(
            f"Unknown backbone '{backbone}'. Must be one of: {list(BACKBONE_BUILDERS)}"
        )
    model: DeepLabV3 = BACKBONE_BUILDERS[backbone](
        num_classes=num_classes, aux_loss=True
    )
    model.to(device)
    return model


def load_model(
    checkpoint_path: Path, device: torch.device
) -> tuple[DeepLabV3, ModelConfig]:
    """Load a trained DeepLab model and its config from the sibling .toml."""
    cfg = load_model_config(checkpoint_path)

    model = build_model(cfg.backbone, cfg.num_classes, device)
    state = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(state, strict=False)
    model.eval()

    warmup = torch.randn((1, 3, cfg.input_size, cfg.input_size)).to(device)
    _ = model(warmup)

    return model, cfg
