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

from constants import IMAGE_SIZE, NUM_CLASSES, PAD_SIZE


def seed_everything(seed_value: int) -> None:
    np.random.seed(seed_value)
    torch.manual_seed(seed_value)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = True


def common_transforms(
    mean: tuple[float, float, float] = (0.4611, 0.4359, 0.3905),
    std: tuple[float, float, float] = (0.2193, 0.2150, 0.2109),
) -> transforms.Compose:
    return transforms.Compose(
        [
            transforms.ToPILImage(),
            transforms.Pad(PAD_SIZE),
            transforms.ToTensor(),
            transforms.Normalize(mean, std),
        ]
    )


def load_model(checkpoint_path: Path, device: torch.device) -> DeepLabV3:
    model_name = checkpoint_path.stem.split("_")[1]
    model: DeepLabV3 = {
        "mbv3": deeplabv3_mobilenet_v3_large,
        "r50": deeplabv3_resnet50,
        "r101": deeplabv3_resnet101,
    }[model_name](num_classes=NUM_CLASSES, aux_loss=True)

    model.to(device)
    checkpoints = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(checkpoints, strict=False)
    model.eval()

    warmup_input = torch.randn((1, 3, IMAGE_SIZE, IMAGE_SIZE)).to(device)
    _ = model(warmup_input)

    return model
