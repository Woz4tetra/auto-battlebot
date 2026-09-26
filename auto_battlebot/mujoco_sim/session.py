"""Load smooth_tag_poses.py session bundles into rollout windows for the MuJoCo fit.

A bundle is the directory playground/calibration/smooth_tag_poses.py writes per recording:
smoothed.csv (uniform-grid smoothed pose), measurements.csv (raw tag detections), commands.csv
(ESP32 events on the app clock), windows.csv (gated windows with initial states) and
session.json (noise floor, clock fit, ...).

The fit's loss reads the raw measurements, never the smoothed poses; the smoothed state only
supplies each window's initial conditions.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from auto_battlebot.mujoco_sim.actuator import (
    NOMINAL_PACK_V,
    PlantParams,
    delay_tape,
    shape_command,
    side_gains,
)
from auto_battlebot.mujoco_sim.rollout import InitialState

# Tape history kept before each window start, so the delay shift never runs off the front.
TAPE_PAD_S = 0.12
DEFAULT_HORIZONS = (0.2, 0.4, 1.0)
# A measurement counts for a horizon when it lands within this of it (60 fps is 16.7 ms).
HORIZON_TOLERANCE_S = 0.02


@dataclass
class Session:
    name: str
    path: Path
    smoothed: pd.DataFrame
    measurements: pd.DataFrame
    commands: pd.DataFrame
    windows: pd.DataFrame
    meta: dict[str, Any]

    @property
    def noise_xy(self) -> float:
        floor = self.meta.get("noise_floor", {})
        return float(floor.get("sigma_xy", floor.get("sigma_x", 0.002)))

    @property
    def noise_yaw(self) -> float:
        return float(self.meta.get("noise_floor", {}).get("sigma_yaw", 0.01))

    @property
    def radio_delay_s(self) -> float | None:
        """Stick-to-ESP32 delay from the throttle channel, which a punch or hold moves most."""
        radio = self.meta.get("radio_delay") or {}
        lag = (radio.get("linear") or {}).get("lag_s")
        return float(lag) if lag is not None else None

    @property
    def noise_pitch(self) -> float:
        return float(self.meta.get("noise_floor", {}).get("sigma_pitch", 0.01))


def load_session(path: Path) -> Session:
    path = Path(path)
    meta = json.loads((path / "session.json").read_text())
    return Session(
        name=path.name,
        path=path,
        smoothed=pd.read_csv(path / "smoothed.csv"),
        measurements=pd.read_csv(path / "measurements.csv"),
        commands=pd.read_csv(path / "commands.csv"),
        windows=pd.read_csv(path / "windows.csv"),
        meta=meta,
    )


def motor_polarity(session: Session) -> float:
    """+1 if a positive mean per-motor command drives the robot forward, else -1.

    The firmware mixer writes left = -a + turn, and the ESC wiring decides which way that
    turns the wheel; the recording settles it by correlating the mean command with the
    smoothed forward speed.
    """
    s = session.smoothed
    forward = s["vx"] * np.cos(s["yaw"]) + s["vy"] * np.sin(s["yaw"])
    cmd = session.commands
    mean_cmd = np.interp(
        s["stamp_ns"].to_numpy(float),
        cmd["stamp_ns"].to_numpy(float),
        (0.5 * (cmd["left_cmd"] + cmd["right_cmd"])).to_numpy(float),
    )
    corr = float(np.corrcoef(mean_cmd, forward.to_numpy(float))[0, 1])
    return 1.0 if not np.isfinite(corr) or corr >= 0 else -1.0


@dataclass
class WindowBatch:
    """Windows ready for rollout. All tape arrays share one grid: `dt`, `pad` steps of history
    before the window start, then `steps` steps of rollout."""

    dt: float
    steps: int
    pad: int
    horizons: tuple[float, ...]
    session: np.ndarray  # (N,) index into `sessions`
    window_id: np.ndarray  # (N,)
    kind: np.ndarray  # (N,) "flat" or "nose_lift"
    start_ns: np.ndarray  # (N,)
    u_raw: np.ndarray  # (N, pad + steps, 2) per-motor command in [-1, 1], forward-positive
    vbat: np.ndarray  # (N, pad + steps)
    state: InitialState
    # Logged firmware signals on the same grid, for the closed-loop check: a_percent,
    # b_percent, pid_output, left_cmd, right_cmd (percent), orientation_x (degrees).
    firmware: np.ndarray = field(default_factory=lambda: np.zeros((0, 0, 6)))
    # Per window: +1 if a positive logged motor command drives forward (see motor_polarity).
    polarity: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Per window, raw measurements inside (0, horizon]: time since start and values.
    meas_t: list[np.ndarray] = field(default_factory=list)
    meas_x: list[np.ndarray] = field(default_factory=list)
    meas_y: list[np.ndarray] = field(default_factory=list)
    meas_yaw: list[np.ndarray] = field(default_factory=list)
    meas_pitch: list[np.ndarray] = field(default_factory=list)
    noise_xy: np.ndarray = field(default_factory=lambda: np.zeros(0))  # (N,)
    noise_yaw: np.ndarray = field(default_factory=lambda: np.zeros(0))
    noise_pitch: np.ndarray = field(default_factory=lambda: np.zeros(0))
    sessions: list[str] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.window_id)

    def take(self, idx: np.ndarray) -> WindowBatch:
        idx = np.asarray(idx, dtype=int)
        pick = lambda seq: [seq[i] for i in idx]  # noqa: E731
        return WindowBatch(
            dt=self.dt,
            steps=self.steps,
            pad=self.pad,
            horizons=self.horizons,
            session=self.session[idx],
            window_id=self.window_id[idx],
            kind=self.kind[idx],
            start_ns=self.start_ns[idx],
            u_raw=self.u_raw[idx],
            vbat=self.vbat[idx],
            firmware=self.firmware[idx] if len(self.firmware) else self.firmware,
            polarity=self.polarity[idx] if len(self.polarity) else self.polarity,
            state=InitialState(**{k: v[idx] for k, v in vars(self.state).items()}),
            meas_t=pick(self.meas_t),
            meas_x=pick(self.meas_x),
            meas_y=pick(self.meas_y),
            meas_yaw=pick(self.meas_yaw),
            meas_pitch=pick(self.meas_pitch),
            noise_xy=self.noise_xy[idx],
            noise_yaw=self.noise_yaw[idx],
            noise_pitch=self.noise_pitch[idx],
            sessions=self.sessions,
        )

    def tapes(self, params: PlantParams | list[PlantParams]) -> tuple[np.ndarray, np.ndarray]:
        """Voltage and zero-command tapes, (N, steps, 2) each, for one parameter set per window
        (or one shared). Applies delay, deadzone, throttle curve, left/right ratio and vbat."""
        n = len(self)
        plist = params if isinstance(params, list) else [params] * n
        delay = np.array([p.delay_s for p in plist])
        dz = np.array([[p.deadzone_left, p.deadzone_right] for p in plist])
        curve = np.array([p.curve for p in plist])[:, None]
        gain_l, gain_r = side_gains(np.array([p.lr_gain_ratio for p in plist]))
        volts = np.empty((n, self.steps, 2))
        zero = np.empty((n, self.steps, 2))
        # Delay per unique value (candidates share it across their windows).
        for d in np.unique(delay):
            rows = np.nonzero(delay == d)[0]
            shifted = delay_tape(np.moveaxis(self.u_raw[rows], 1, 2), self.dt, float(d))
            shifted = np.moveaxis(shifted, 2, 1)[:, self.pad :]
            vb = self.vbat[rows, self.pad :]
            for k, gain in ((0, gain_l[rows]), (1, gain_r[rows])):
                shaped = shape_command(shifted[..., k], dz[rows, k : k + 1], curve[rows])
                zero[rows, :, k] = shaped == 0.0
                volts[rows, :, k] = shaped * vb * gain[:, None]
        return volts, zero


def build_batch(
    sessions: list[Session],
    horizon_s: float = max(DEFAULT_HORIZONS),
    dt: float = 1e-3,
    kinds: tuple[str, ...] = ("flat", "nose_lift"),
    horizons: tuple[float, ...] = DEFAULT_HORIZONS,
    track_half_width: float = 0.06526,
    wheel_radius: float = 0.025,
) -> WindowBatch:
    steps = int(round(horizon_s / dt))
    pad = int(round(TAPE_PAD_S / dt))
    rows: dict[str, list[Any]] = {
        k: [] for k in ("session", "wid", "kind", "start", "u", "vbat", "firmware", "polarity")
    }
    state_cols: dict[str, list[float]] = {k: [] for k in InitialState.__dataclass_fields__}
    meas: dict[str, list[np.ndarray]] = {k: [] for k in ("t", "x", "y", "yaw", "pitch")}
    noise: dict[str, list[float]] = {"xy": [], "yaw": [], "pitch": []}
    for si, session in enumerate(sessions):
        polarity = motor_polarity(session)
        cmd = session.commands.sort_values("stamp_ns")
        cmd_t = cmd["stamp_ns"].to_numpy(np.int64)
        left = polarity * cmd["left_cmd"].to_numpy(float) / 100.0
        right = polarity * cmd["right_cmd"].to_numpy(float) / 100.0
        vbat_col = cmd["vbat"].to_numpy(float) if "vbat" in cmd else np.full(len(cmd), np.nan)
        fw_cols = ["a_percent", "b_percent", "pid_output", "left_cmd", "right_cmd", "orientation_x"]
        firmware = np.stack(
            [cmd[c].to_numpy(float) if c in cmd else np.full(len(cmd), np.nan) for c in fw_cols],
            axis=-1,
        )
        vbat_fill = float(np.nanmedian(vbat_col)) if np.isfinite(vbat_col).any() else NOMINAL_PACK_V
        meas_df = session.measurements
        if "rejected" in meas_df:
            meas_df = meas_df[~meas_df["rejected"].astype(bool)]
        m_t = meas_df["stamp_ns"].to_numpy(np.int64)
        for _, win in session.windows.iterrows():
            if str(win["kind"]) not in kinds:
                continue
            start = int(win["start_ns"])
            grid = start + ((np.arange(-pad, steps) * dt) * 1e9).astype(np.int64)
            # Zero-order hold of the latest event at or before each grid time.
            at = np.searchsorted(cmd_t, grid, side="right") - 1
            valid = at >= 0
            at = np.clip(at, 0, len(cmd_t) - 1)
            u = np.stack([np.where(valid, left[at], 0.0), np.where(valid, right[at], 0.0)], -1)
            vb = np.where(valid & np.isfinite(vbat_col[at]), vbat_col[at], vbat_fill)
            rows["session"].append(si)
            rows["wid"].append(int(win["window_id"]))
            rows["kind"].append(str(win["kind"]))
            rows["start"].append(start)
            rows["u"].append(u)
            rows["vbat"].append(vb)
            rows["firmware"].append(np.where(valid[:, None], firmware[at], np.nan))
            rows["polarity"].append(polarity)
            state_cols["x"].append(float(win["x0"]))
            state_cols["y"].append(float(win["y0"]))
            state_cols["yaw"].append(float(win["yaw0"]))
            state_cols["vx"].append(float(win["vx0"]))
            state_cols["vy"].append(float(win["vy0"]))
            state_cols["yaw_rate"].append(float(win["yaw_rate0"]))
            state_cols["pitch"].append(float(win["pitch0"]))
            # Wheel speeds from the no-slip relation, recomputed here in the MJCF's convention
            # (positive rolls forward) rather than read from windows.csv.
            forward = float(win["vx0"]) * np.cos(float(win["yaw0"])) + float(win["vy0"]) * np.sin(
                float(win["yaw0"])
            )
            spin = float(win["yaw_rate0"]) * track_half_width
            state_cols["wheel_left"].append((forward - spin) / wheel_radius)
            state_cols["wheel_right"].append((forward + spin) / wheel_radius)
            lo = np.searchsorted(m_t, start, side="right")
            hi = np.searchsorted(m_t, start + int(horizon_s * 1e9), side="right")
            chunk = meas_df.iloc[lo:hi]
            meas["t"].append((chunk["stamp_ns"].to_numpy(np.int64) - start) / 1e9)
            meas["x"].append(chunk["x"].to_numpy(float))
            meas["y"].append(chunk["y"].to_numpy(float))
            meas["yaw"].append(chunk["yaw"].to_numpy(float))
            meas["pitch"].append(chunk["pitch"].to_numpy(float))
            noise["xy"].append(session.noise_xy)
            noise["yaw"].append(session.noise_yaw)
            noise["pitch"].append(session.noise_pitch)
    if not rows["u"]:
        raise ValueError("no windows of the requested kinds in these sessions")
    return WindowBatch(
        dt=dt,
        steps=steps,
        pad=pad,
        horizons=horizons,
        session=np.array(rows["session"]),
        window_id=np.array(rows["wid"]),
        kind=np.array(rows["kind"]),
        start_ns=np.array(rows["start"], dtype=np.int64),
        u_raw=np.stack(rows["u"]),
        vbat=np.stack(rows["vbat"]),
        firmware=np.stack(rows["firmware"]),
        polarity=np.array(rows["polarity"]),
        state=InitialState(**{k: np.array(v) for k, v in state_cols.items()}),
        meas_t=meas["t"],
        meas_x=meas["x"],
        meas_y=meas["y"],
        meas_yaw=meas["yaw"],
        meas_pitch=meas["pitch"],
        noise_xy=np.array(noise["xy"]),
        noise_yaw=np.array(noise["yaw"]),
        noise_pitch=np.array(noise["pitch"]),
        sessions=[s.name for s in sessions],
    )
