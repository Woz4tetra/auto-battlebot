from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import TypeVar

import dacite

from auto_battlebot.compat import tomllib

T = TypeVar("T")


def load_config(path: str | Path, data_class: type[T], strict: bool = True) -> T:
    """Load a TOML file into a typed dataclass via dacite, strict by default."""
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    parsed: T = dacite.from_dict(
        data_class=data_class, data=raw, config=dacite.Config(strict=strict, cast=[Enum])
    )
    return parsed
