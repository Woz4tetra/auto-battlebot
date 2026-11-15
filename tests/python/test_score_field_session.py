"""score_field_session scores the same arc, margin and engagement gap as the RL environment."""

from __future__ import annotations

import math

from auto_battlebot.rl.envs.warp_nav import WarpNavConfig
from playground.rl import score_field_session


def test_constants_match_warp_nav_config() -> None:
    cfg = WarpNavConfig()
    assert score_field_session.ARC_HALF_RAD == math.radians(cfg.arc_half_angle_deg)
    assert score_field_session.DANGER_MARGIN_M == cfg.danger_margin_m
    assert score_field_session.ENGAGEMENT_GAP_S == cfg.engagement_gap_s
    assert score_field_session.STILL_SPEED_MPS == cfg.still_speed_mps
