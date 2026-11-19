"""The parts of an app profile the sysid monitor needs: tag mounts, rest pitch, field size."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from auto_battlebot.config_profile import CONFIG_ROOT, load_profile_toml
from auto_battlebot.perception.tag_pose_smoother import TagMount, tag_mount_from_table

DEFAULT_PROFILE = "mr_stabs_mk2_sysid_zed_box"


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
