"""Mr Stabs Mk2 sysid sessions as opponent-filter evaluation data."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from auto_battlebot.perception import yolo_field_fixes
from auto_battlebot.perception.tag_pose_smoother import up_from_field
from auto_battlebot.recording.sysid_io import load_field_from_camera, load_robot_tags

REPO_ROOT = Path(__file__).resolve().parents[2]
BUNDLES = REPO_ROOT / "playground/calibration/out/mujoco_sysid"
RECORDINGS = REPO_ROOT / "data/recordings"
OUT = REPO_ROOT / "playground/opponent_prediction/out"

SESSIONS = {
    "2026-10-15_00-29-00": "train",
    "2026-10-15_00-20-58": "holdout",
}
PERCEPTION_HZ = 30.0
TRUTH_MAX_GAP_NS = 50_000_000


@dataclass
class SessionData:
    name: str
    role: str
    meas_tag_clock_stamp_ns: np.ndarray
    meas_axle_up_x: np.ndarray
    meas_axle_up_y: np.ndarray
    meas_axle_up_yaw: np.ndarray
    truth_5ms_grid: pd.DataFrame
    truth_valid: np.ndarray

    def truth_at(self, stamp_ns: np.ndarray) -> dict[str, np.ndarray]:
        """Truth interpolated to `stamp_ns`, with `valid` false where it is not trusted."""
        t = self.truth_5ms_grid["stamp_ns"].to_numpy(np.int64)
        s = np.asarray(stamp_ns, dtype=np.int64)
        out = {
            k: np.interp(s, t, self.truth_5ms_grid[k].to_numpy(float))
            for k in ("x", "y", "vx", "vy", "yaw_rate")
        }
        out["yaw"] = np.angle(
            np.exp(1j * np.interp(s, t, np.unwrap(self.truth_5ms_grid["yaw"].to_numpy(float))))
        )
        nearest = np.clip(np.searchsorted(t, s), 0, len(t) - 1)
        out["valid"] = self.truth_valid[nearest] & (s >= t[0]) & (s <= t[-1])
        return out


def _mcap_for(name: str) -> Path:
    matches = sorted(RECORDINGS.glob(f"*sysid*{name}.mcap"))
    if not matches:
        raise FileNotFoundError(f"no sysid recording for {name} under {RECORDINGS}")
    return matches[0]


def _measurements(name: str) -> pd.DataFrame:
    cache = OUT / name / "measurements_yolo30.csv"
    if cache.exists():
        return pd.read_csv(cache)
    mcap = _mcap_for(name)
    bundle = BUNDLES / f"{name}_yolo"
    cal = json.loads((bundle / "session.json").read_text())["yolo"]["calibration"]
    det = pd.read_csv(bundle / "yolo.csv")
    camera_matrix = load_robot_tags(mcap)[0].camera.matrix
    field_from_camera = load_field_from_camera(mcap)
    t_up_field, _ = up_from_field(field_from_camera)

    cand = det[det["conf"] >= yolo_field_fixes.YoloFixSettings().min_conf].reset_index(drop=True)
    stamps = cand["stamp_ns"].to_numpy(np.int64)
    up_from_camera = np.asarray(t_up_field @ field_from_camera.at(stamps))
    proj = yolo_field_fixes.project_detections(
        cand, camera_matrix, up_from_camera, cal["plane_height_m"], cal["axle_offset_m"]
    )
    diag = np.hypot(cand["x2"] - cand["x1"], cand["y2"] - cand["y1"]).to_numpy()
    sized = (diag >= cal["box_diag_px"][0]) & (diag <= cal["box_diag_px"][1])
    sized &= (proj["sep"] >= cal["keypoint_sep_m"][0]) & (proj["sep"] <= cal["keypoint_sep_m"][1])
    cand = cand[sized].assign(x=proj["x"][sized], y=proj["y"][sized], yaw=proj["yaw"][sized])
    cand = cand.sort_values(["stamp_ns", "conf"], ascending=[True, False])
    cand = cand.drop_duplicates("stamp_ns", keep="first")
    cand["stamp_ns"] = cand["stamp_ns"].to_numpy(np.int64) + int(round(cal["offset_ms"] * 1e6))

    slot = ((cand["stamp_ns"] - cand["stamp_ns"].iloc[0]) * PERCEPTION_HZ // 1_000_000_000).astype(
        np.int64
    )
    cand = cand.assign(slot=slot).drop_duplicates("slot", keep="first")
    out = cand[["stamp_ns", "x", "y", "yaw", "conf"]].reset_index(drop=True)
    cache.parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(cache, index=False)
    return out


def _truth(name: str) -> tuple[pd.DataFrame, np.ndarray]:
    bundle = BUNDLES / f"{name}_tag"
    truth = pd.read_csv(bundle / "smoothed.csv")
    meas = pd.read_csv(bundle / "measurements.csv")
    if (meas["source"] == "yolo").any():
        raise ValueError(f"{bundle} holds YOLO fixes; truth must come from the tag-only bundle")
    tags = meas[(meas["source"] == "tag") & (meas["rejected"] == 0)]["stamp_ns"].to_numpy(np.int64)
    tags = np.unique(tags)
    t = truth["stamp_ns"].to_numpy(np.int64)
    j = np.clip(np.searchsorted(tags, t), 1, len(tags) - 1)
    gap = np.minimum(np.abs(tags[j] - t), np.abs(t - tags[j - 1]))
    valid = (gap <= TRUTH_MAX_GAP_NS) & (truth["upside_down"].to_numpy() == 0)
    return truth, valid


def load_session(name: str) -> SessionData:
    meas = _measurements(name)
    truth, valid = _truth(name)
    return SessionData(
        name=name,
        role=SESSIONS.get(name, "other"),
        meas_tag_clock_stamp_ns=meas["stamp_ns"].to_numpy(np.int64),
        meas_axle_up_x=meas["x"].to_numpy(float),
        meas_axle_up_y=meas["y"].to_numpy(float),
        meas_axle_up_yaw=meas["yaw"].to_numpy(float),
        truth_5ms_grid=truth,
        truth_valid=valid,
    )
