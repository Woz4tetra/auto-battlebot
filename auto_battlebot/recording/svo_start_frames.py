"""The per-recording svo_start_frame table shared by playback replays and eval dataset builds."""

from __future__ import annotations

from pathlib import Path

from auto_battlebot.compat import tomllib

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_START_FRAMES = REPO_ROOT / "config" / "playback" / "svo_start_frames.toml"


def load_start_frames(path: Path = DEFAULT_START_FRAMES) -> dict[str, int]:
    """Map SVO stem (no dir, no .svo2) to svo_start_frame; recordings not listed start at 0."""
    if not path.exists():
        print(f"Warning: start frame table {path} not found; using start frame 0 for all")
        return {}
    return {str(k): int(v) for k, v in tomllib.loads(path.read_text())["start_frames"].items()}


def start_frame_for(svo_path: Path | str, table: dict[str, int]) -> int:
    return table.get(Path(svo_path).stem, 0)
