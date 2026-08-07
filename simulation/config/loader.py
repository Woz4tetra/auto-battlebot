from __future__ import annotations

import sys
from pathlib import Path
from typing import TypeVar

import dacite

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

T = TypeVar("T")


def load_config(path: str | Path, data_class: type[T], strict: bool = True) -> T:
    """Load a TOML file into a typed dataclass, validating via dacite.

    With strict=True, unknown keys raise (catches config typos); pass strict=False to tolerate
    extra keys.
    """
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    parsed: T = dacite.from_dict(
        data_class=data_class, data=raw, config=dacite.Config(strict=strict)
    )
    return parsed
