"""The parts of an app profile the sysid monitor needs: tag mounts, rest pitch, field size.

Profiles are TOML files under ``config/`` that ``extends`` a base. ``load_profile`` follows the
chain with the same merge as ``src/config/config.cpp``: tables merge key by key, scalars and
arrays from the overlay replace the base, and a table whose ``type`` differs replaces the base
table outright.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from auto_battlebot.compat import tomllib
from auto_battlebot.perception.tag_pose_smoother import TagMount, tag_mount_from_table

CONFIG_ROOT = Path(__file__).resolve().parents[3] / "config"
DEFAULT_PROFILE = "mr_stabs_mk2_sysid_zed_box"


def _merge(base: dict[str, Any], overlay: dict[str, Any]) -> None:
    for key, value in overlay.items():
        current = base.get(key)
        if isinstance(value, dict) and isinstance(current, dict):
            if "type" in value and "type" in current and value["type"] != current["type"]:
                base[key] = value
            else:
                _merge(current, value)
        else:
            base[key] = value


def load_profile_toml(name: str, config_root: Path = CONFIG_ROOT) -> dict[str, Any]:
    """The merged TOML of a profile, by name (``mr_stabs_mk2_sysid_zed_box``) or path."""
    path = Path(name)
    if path.suffix != ".toml":
        path = config_root / f"{name}.toml"
    chain: list[Path] = []
    return _load(path, config_root, chain)


def _load(path: Path, config_root: Path, chain: list[Path]) -> dict[str, Any]:
    canonical = path.resolve()
    if canonical in chain:
        raise ValueError(f"circular config inheritance: {[p.name for p in chain]} -> {path.name}")
    chain.append(canonical)
    with open(path, "rb") as handle:
        data: dict[str, Any] = tomllib.load(handle)
    extends = data.pop("extends", None)
    if extends is None:
        return data
    base_path = config_root / extends
    if base_path.suffix != ".toml":
        base_path = base_path.with_name(base_path.name + ".toml")
    merged = _load(base_path, config_root, chain)
    _merge(merged, data)
    return merged


@dataclass
class SysidProfile:
    name: str
    field_size_x_m: float
    field_size_y_m: float
    mounts: dict[int, TagMount]
    rest_pitch_rad: float

    @property
    def half_extent_m(self) -> float:
        return min(self.field_size_x_m, self.field_size_y_m) / 2.0


def load_profile(name: str = DEFAULT_PROFILE, config_root: Path = CONFIG_ROOT) -> SysidProfile:
    data = load_profile_toml(name, config_root)
    field = data.get("field_filter", {})
    model = data.get("keypoint_model", {})
    mounts = {
        int(tag["id"]): tag_mount_from_table(int(tag["id"]), tag) for tag in model.get("tags", [])
    }
    if not mounts:
        raise ValueError(f"profile {name} has no [[keypoint_model.tags]] mounts")
    return SysidProfile(
        name=Path(name).stem,
        field_size_x_m=float(field["field_size_x"]),
        field_size_y_m=float(field["field_size_y"]),
        mounts=mounts,
        rest_pitch_rad=float(model.get("rest_pitch_rad", 0.0)),
    )
