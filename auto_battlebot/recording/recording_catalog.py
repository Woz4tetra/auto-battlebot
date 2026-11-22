"""Playback recording catalog shared by C++ replays and eval dataset builds."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT

DEFAULT_CATALOG = REPO_ROOT / "config" / "playback" / "recordings.toml"


@dataclass(frozen=True)
class Recording:
    path: Path
    start_frame: int = 0
    note: str = ""


def load_catalog(path: Path = DEFAULT_CATALOG) -> dict[str, Recording]:
    """Map recording name (file stem) to its repo-relative path, start frame, and note."""
    if not path.exists():
        print(f"Warning: recording catalog {path} not found; using start frame 0 for all")
        return {}
    catalog: dict[str, Recording] = {}
    for name, entry in tomllib.loads(path.read_text()).get("recordings", {}).items():
        if not isinstance(entry.get("path"), str):
            raise ValueError(f"{path}: [recordings.{name!r}] needs a string `path`")
        catalog[str(name)] = Recording(
            path=Path(entry["path"]),
            start_frame=int(entry.get("start_frame", 0)),
            note=str(entry.get("note", "")),
        )
    return catalog


def load_start_frames(path: Path = DEFAULT_CATALOG) -> dict[str, int]:
    """Map recording stem to start frame; recordings not listed start at 0."""
    return {name: rec.start_frame for name, rec in load_catalog(path).items()}


def start_frame_for(svo_path: Path | str, table: dict[str, int]) -> int:
    return table.get(Path(svo_path).stem, 0)
