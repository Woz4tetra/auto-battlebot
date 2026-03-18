from __future__ import annotations

from pathlib import Path

import dacite

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]

from config.root import SimConfig


def load_sim_config(path: str | Path) -> SimConfig:
    """Load a TOML config file and return a fully-typed SimConfig."""
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    return dacite.from_dict(data_class=SimConfig, data=raw)
