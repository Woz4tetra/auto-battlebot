from __future__ import annotations

import re
from pathlib import Path

from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.segmentation.model_config import (
    ModelConfig,
    load_model_config,
    save_model_config,
)

REPO = REPO_ROOT


def test_sidecar_round_trips_and_matches_cpp_keys(tmp_path: Path) -> None:
    config = ModelConfig("mbv3", 256, 10, 2, "v3")
    toml_path = save_model_config(tmp_path / "m.pth", config)
    assert load_model_config(tmp_path / "m.pth") == config
    text = toml_path.read_text()
    cpp = (REPO / "src/mask_model/deeplab_mask_model.cpp").read_text()
    for key in re.findall(r'model_section\["(\w+)"\]', cpp):
        assert f"{key} = " in text
