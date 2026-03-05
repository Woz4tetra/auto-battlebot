"""Read and write the TOML metadata file that accompanies a DeepLab checkpoint.

Every trained model produces a sibling `<stem>.toml` file next to the `.pth`
(or `.engine`) that records the parameters needed to load and preprocess:

    [model]
    backbone    = "mbv3"
    image_size  = 256
    pad_size    = 10
    num_classes = 2

All scripts (training, export, inference) and the C++ runtime read this file
so the values stay in sync.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]

import tomli_w


@dataclasses.dataclass
class ModelConfig:
    backbone: str
    image_size: int
    pad_size: int
    num_classes: int

    @property
    def input_size(self) -> int:
        return self.image_size + 2 * self.pad_size


def config_path_for(model_path: Path) -> Path:
    """Return the expected .toml config path for a given model file."""
    return model_path.with_suffix(".toml")


def load_model_config(model_path: Path) -> ModelConfig:
    """Load a ModelConfig from the .toml file next to *model_path*."""
    toml_path = config_path_for(model_path)
    if not toml_path.exists():
        raise FileNotFoundError(
            f"Model config not found: {toml_path}\n"
            "Re-train or create a .toml config file alongside the model."
        )
    with open(toml_path, "rb") as f:
        data = tomllib.load(f)

    section = data.get("model", data)
    return ModelConfig(
        backbone=section["backbone"],
        image_size=section["image_size"],
        pad_size=section["pad_size"],
        num_classes=section["num_classes"],
    )


def save_model_config(model_path: Path, config: ModelConfig) -> Path:
    """Write a ModelConfig as a .toml file next to *model_path*.

    Returns the path to the written file.
    """
    toml_path = config_path_for(model_path)
    data = {"model": dataclasses.asdict(config)}
    toml_path.write_bytes(tomli_w.dumps(data).encode())
    return toml_path
