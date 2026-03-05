from pathlib import Path

import numpy as np
import segmentation_models_pytorch as smp
import torch
import torch.nn as nn
from torchvision.models.segmentation import (
    deeplabv3_mobilenet_v3_large,
    deeplabv3_resnet50,
    deeplabv3_resnet101,
)
from torchvision import transforms

from model_config import ModelConfig, load_model_config

TORCHVISION_BACKBONE_BUILDERS = {
    "mbv3": deeplabv3_mobilenet_v3_large,
    "r50": deeplabv3_resnet50,
    "r101": deeplabv3_resnet101,
}

SMP_ENCODER_NAMES = {
    "mbv3": "timm-mobilenetv3_large_100",
    "r50": "resnet50",
    "r101": "resnet101",
}

VALID_DECODERS = ("v3", "v3plus")


class SegModelWrapper(nn.Module):
    """Normalizes output to a plain tensor regardless of backend.

    Torchvision DeepLabV3 returns ``{"out": tensor, ...}`` while SMP returns a
    plain tensor.  Wrapping at build time keeps all downstream code uniform.
    """

    def __init__(self, model: nn.Module, dict_output: bool) -> None:
        super().__init__()
        self.model = model
        self.dict_output = dict_output

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.model(x)
        return out["out"] if self.dict_output else out


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


def _build_torchvision(backbone: str, num_classes: int) -> nn.Module:
    if backbone not in TORCHVISION_BACKBONE_BUILDERS:
        raise ValueError(
            f"Unknown backbone '{backbone}'. "
            f"Must be one of: {list(TORCHVISION_BACKBONE_BUILDERS)}"
        )
    return TORCHVISION_BACKBONE_BUILDERS[backbone](
        num_classes=num_classes, aux_loss=True
    )


def _build_smp(backbone: str, num_classes: int) -> nn.Module:
    encoder = SMP_ENCODER_NAMES.get(backbone)
    if encoder is None:
        raise ValueError(
            f"Unknown backbone '{backbone}' for SMP. "
            f"Must be one of: {list(SMP_ENCODER_NAMES)}"
        )
    return smp.DeepLabV3Plus(
        encoder_name=encoder,
        encoder_weights="imagenet",
        classes=num_classes,
    )


def build_model(
    backbone: str,
    num_classes: int,
    device: torch.device,
    decoder: str = "v3",
) -> SegModelWrapper:
    if decoder not in VALID_DECODERS:
        raise ValueError(
            f"Unknown decoder '{decoder}'. Must be one of: {list(VALID_DECODERS)}"
        )
    if decoder == "v3plus":
        raw_model = _build_smp(backbone, num_classes)
        dict_output = False
    else:
        raw_model = _build_torchvision(backbone, num_classes)
        dict_output = True
    raw_model.to(device)
    return SegModelWrapper(raw_model, dict_output).to(device)


def load_model(
    checkpoint_path: Path, device: torch.device
) -> tuple[SegModelWrapper, ModelConfig]:
    """Load a trained model and its config from the sibling .toml."""
    cfg = load_model_config(checkpoint_path)

    wrapper = build_model(cfg.backbone, cfg.num_classes, device, decoder=cfg.decoder)
    state = torch.load(checkpoint_path, map_location=device)
    wrapper.model.load_state_dict(state, strict=False)
    wrapper.eval()

    warmup = torch.randn((1, 3, cfg.input_size, cfg.input_size)).to(device)
    _ = wrapper(warmup)

    return wrapper, cfg
