"""The dropout video overlay projects measurements onto the height the estimator uses."""

from __future__ import annotations

from pathlib import Path

from auto_battlebot.compat import tomllib
from playground.opponent_prediction import render_dropout_videos as videos

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_replay_plane_height_matches_keypoint_height_config() -> None:
    config = tomllib.loads((REPO_ROOT / "config" / "_common.toml").read_text())
    keypoint_height_m = config["robot_filter"]["keypoint_height_meters"]
    replay_scenes = [s for s in videos.SCENES.values() if s.kind is videos.SceneKind.REPLAY]
    assert replay_scenes
    assert {s.plane_z_m for s in replay_scenes} == {keypoint_height_m}
