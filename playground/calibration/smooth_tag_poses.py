"""Smooth one Mr Stabs Mk2 sysid session into the bundle the MuJoCo fit reads.

Reads an MCAP recorded with the ``mr_stabs_mk2_sysid_zed_box`` profile (``/apriltag/robot_tags``,
``/robot/esp32_diagnostics``, the transmitter channels, ``/tf``) and writes, under
``playground/calibration/out/mujoco_sysid/<session_name>/``:

- ``smoothed.csv``: RTS-smoothed axle-center pose on a 5 ms grid
- ``measurements.csv``: one row per robot tag detection, raw pose and gating provenance
- ``commands.csv``: one row per ESP32 event, stamped on the app clock by the clock fit
- ``windows.csv``: gated 1 to 2 s fit windows with their initial state
- ``session.json``: noise floor, process noise and its cross-validation curve, clock fit,
  radio delay, IMU agreement, IPPE close calls, rejections, window counts and drop reasons

Conventions (details in ``auto_battlebot/perception/tag_pose_smoother.py``):

- Frame: the field frame with z up. ``FiducialFieldFilter`` records field z into the floor; that
  frame is turned 180 degrees about field x (y and z negated) so the output is right-handed with
  z up. ``session.json`` ``output_frame`` says whether the turn was applied.
- Pose: the axle midpoint, FLU body frame. ``yaw`` is counter-clockwise about field +z,
  unwrapped. ``pitch`` and ``roll`` are ZYX Euler angles: pitch about body +y (nose down is
  positive, an upright robot at rest reads about +0.198 rad), roll about body +x. With
  ``upside_down`` set they are relative to the inverted rest pose instead (body turned 180
  degrees about x), so they read near zero when it drives inverted.
- Units: metres, radians, seconds; stamps are app-clock nanoseconds. Wheel speeds in
  ``windows.csv`` are rad/s, positive rolling forward, from the no-slip relation.
- ``sigma_xy`` in ``measurements.csv`` is the RMS of the along-view and across-view sigmas; the
  split and its gains are in ``session.json`` ``noise_model``.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/smooth_tag_poses.py path/to/session.mcap --plots
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from auto_battlebot.compat import tomllib
from auto_battlebot.perception.sysid_windows import WindowOptions, make_windows
from auto_battlebot.perception.tag_pose_smoother import (
    SessionSmoothing,
    SmootherOptions,
    load_mass_properties,
    smooth_session,
)
from auto_battlebot.recording.esp32_clock import crsf_to_unit, fit_robot_clock, radio_link_delay
from auto_battlebot.recording.sysid_io import (
    field_transform_constancy,
    load_esp32_diagnostics,
    load_field_from_camera,
    load_robot_tags,
    load_transmitter_channels,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_DIR = REPO_ROOT / "simulation" / "assets" / "robots" / "mr_stabs_mk2"
DEFAULT_MASS_PROPERTIES = ROBOT_DIR / "mass_properties.toml"
DEFAULT_COLLISION = ROBOT_DIR / "collision" / "collision.toml"
DEFAULT_OUT_ROOT = REPO_ROOT / "playground" / "calibration" / "out" / "mujoco_sysid"

SMOOTHED_COLUMNS = [
    "stamp_ns",
    "x",
    "y",
    "yaw",
    "vx",
    "vy",
    "yaw_rate",
    "ax",
    "ay",
    "pitch",
    "roll",
    "sigma_x",
    "sigma_y",
    "sigma_yaw",
    "has_measurement",
    "upside_down",
]
MEASUREMENT_COLUMNS = [
    "stamp_ns",
    "tag_id",
    "x",
    "y",
    "z",
    "yaw",
    "pitch",
    "roll",
    "reprojection_error_px",
    "tag_size_px",
    "sigma_xy",
    "sigma_yaw",
    "ippe_close_call",
    "rejected",
]
COMMAND_COLUMNS = [
    "stamp_ns",
    "timestamp_ms",
    "left_cmd",
    "right_cmd",
    "a_percent",
    "b_percent",
    "vbat",
    "pid_setpoint",
    "pid_output",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "accel_x",
    "accel_y",
    "accel_z",
    "loop_us",
    "is_upside_down",
    "flip_switch",
    "armed",
    "radio_connected",
]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("mcap", type=Path)
    parser.add_argument("--session-name", default=None, help="default: the MCAP file stem")
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
    parser.add_argument("--mass-properties", type=Path, default=DEFAULT_MASS_PROPERTIES)
    parser.add_argument(
        "--rest-pitch-rad",
        type=float,
        default=None,
        help="resting pitch, nose-down positive; default: rest_pitch_rad in collision.toml",
    )
    parser.add_argument("--fuse-imu", action="store_true", help="fuse BNO055 yaw rate")
    parser.add_argument("--linear-channel", type=int, default=0, help="stick channel of a_percent")
    parser.add_argument("--angular-channel", type=int, default=1, help="stick channel of b_percent")
    parser.add_argument("--window-s", type=float, default=1.0)
    parser.add_argument("--coverage-min", type=float, default=0.7)
    parser.add_argument("--box-half-extent", type=float, default=0.76)
    parser.add_argument("--box-center", type=float, nargs=2, default=(0.0, 0.0))
    parser.add_argument("--robot-length", type=float, default=0.15)
    parser.add_argument("--pitch-limit-deg", type=float, default=4.0)
    parser.add_argument("--esp32-max-gap-s", type=float, default=0.05)
    parser.add_argument("--plots", action="store_true", help="write diagnostic PNGs")
    return parser.parse_args(argv)


def default_rest_pitch() -> float:
    if DEFAULT_COLLISION.exists():
        with open(DEFAULT_COLLISION, "rb") as handle:
            value = tomllib.load(handle).get("rest_pitch_rad")
        if value is not None:
            return float(value)
    return SmootherOptions.rest_pitch_rad


def jsonable(value: Any) -> Any:
    """Plain JSON types: numpy scalars and arrays unwrapped, NaN and infinities to null."""
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, (float, np.floating)):
        f = float(value)
        return None if math.isnan(f) or math.isinf(f) else f
    return value


def commands_frame(esp32: pd.DataFrame) -> pd.DataFrame:
    return esp32[COMMAND_COLUMNS].copy()


def radio_delays(
    esp32: pd.DataFrame, sticks: pd.DataFrame, linear_channel: int, angular_channel: int
) -> dict[str, Any]:
    """Stick channel -> ESP32 a/b_percent lag, one estimate per stick."""
    out: dict[str, Any] = {}
    if esp32.empty or sticks.empty:
        return {"available": False}
    for name, channel, column in (
        ("linear", linear_channel, "a_percent"),
        ("angular", angular_channel, "b_percent"),
    ):
        lag = radio_link_delay(
            sticks["stamp_ns"].to_numpy(),
            crsf_to_unit(sticks[f"ch{channel}"].to_numpy()),
            esp32["stamp_ns"].to_numpy(),
            esp32[column].to_numpy(),
        )
        out[name] = {"channel": channel, "esp32_field": column, **lag.as_dict()}
    return out


def build_session_json(
    result: SessionSmoothing,
    clock: dict[str, Any] | None,
    radio: dict[str, Any],
    constancy: dict[str, float],
    window_report: dict[str, Any],
    frames_info: dict[str, Any],
    options: SmootherOptions,
) -> dict[str, Any]:
    return {
        "output_frame": {
            "name": "field_up",
            "recorded_field_z_down": result.field_flipped,
            "note": (
                "recorded field frame turned 180 deg about x (y, z negated)"
                if result.field_flipped
                else "recorded field frame unchanged"
            ),
            "T_up_field": result.t_up_field,
        },
        "noise_floor": result.checks.get("noise_floor"),
        "noise_model": result.noise.as_dict(),
        "process_noise": {
            "planar": result.cv_planar.as_dict(),
            "tilt": result.cv_tilt.as_dict(),
        },
        "clock_fit": clock,
        "radio_delay": radio,
        "imu_yaw_rate": result.checks.get("imu_yaw_rate"),
        "imu_fused": result.checks.get("imu_fused"),
        "residual_whiteness": result.checks.get("residual_whiteness"),
        "ippe": result.checks.get("ippe"),
        "gating": result.checks.get("gating"),
        "field_transform_constancy": constancy,
        "windows": window_report,
        "frames": frames_info,
        "smoother_options": {
            k: v for k, v in asdict(options).items() if not isinstance(v, np.ndarray)
        },
    }


def frames_summary(frames: list[Any], mounts: dict[int, Any]) -> dict[str, Any]:
    if not frames:
        return {"frames": 0}
    lag = np.array([f.log_time_ns - f.image_stamp_ns for f in frames], dtype=np.float64) / 1e6
    stamps = np.array([f.image_stamp_ns for f in frames], dtype=np.int64)
    sizes = sorted({f.tag_size_m for f in frames})
    mount_sizes = {tag_id: m.size_m for tag_id, m in mounts.items()}
    return {
        "frames": len(frames),
        "frames_with_detection": int(sum(1 for f in frames if f.detections)),
        "camera_fps": float(1e9 / np.median(np.diff(stamps))) if stamps.size > 1 else None,
        "log_minus_image_stamp_ms_median": float(np.median(lag)),
        "tag_size_m_recorded": sizes,
        "tag_size_m_mass_properties": mount_sizes,
    }


def write_plots(out_dir: Path, result: SessionSmoothing) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    written = []
    meas = result.measurements
    grid = result.grid

    # Raw vs smoothed on the longest still segment the noise model was calibrated on.
    if result.still_segments:
        i0, i1 = max(result.still_segments, key=lambda seg: seg[1] - seg[0])
        t0 = int(meas["stamp_ns"].iloc[i0])
        t1 = int(meas["stamp_ns"].iloc[i1])
        g = grid[(grid["stamp_ns"] >= t0) & (grid["stamp_ns"] <= t1)]
        m = meas.iloc[i0 : i1 + 1]
        fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        for ax, col, scale, unit in (
            (axes[0], "x", 1e3, "mm"),
            (axes[1], "y", 1e3, "mm"),
            (axes[2], "yaw", math.degrees(1.0), "deg"),
        ):
            ax.plot((m["stamp_ns"] - t0) / 1e9, m[col] * scale, ".", ms=3, label="raw")
            ax.plot((g["stamp_ns"] - t0) / 1e9, g[col] * scale, "-", lw=1.5, label="smoothed")
            ax.set_ylabel(f"{col} ({unit})")
        axes[0].legend()
        axes[-1].set_xlabel("time in still segment (s)")
        fig.tight_layout()
        path = out_dir / "still_raw_vs_smoothed.png"
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path.name)

    fig, ax = plt.subplots(figsize=(7, 4))
    lags = np.arange(result.residual_acf.size)
    ax.stem(lags, result.residual_acf, label="residual (raw - smoothed)")
    ax.plot(lags, result.innovation_acf, "o-", ms=3, label="forward innovation")
    n = max(int((~meas["rejected"]).sum()), 1)
    ax.axhline(1.96 / math.sqrt(n), color="grey", ls="--")
    ax.axhline(-1.96 / math.sqrt(n), color="grey", ls="--")
    ax.set_xlabel("lag (detections)")
    ax.set_ylabel("autocorrelation, x")
    ax.legend()
    fig.tight_layout()
    path = out_dir / "residual_autocorrelation.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    written.append(path.name)

    if result.imu is not None and result.imu.stamp_ns.size:
        sign = float(result.imu_lag["sign"]) if result.imu_lag else 1.0
        fig, ax = plt.subplots(figsize=(10, 4))
        t_ref = int(grid["stamp_ns"].iloc[0])
        imu_rate = np.where(result.imu.valid, result.imu.rate * sign, np.nan)
        ax.plot((result.imu.stamp_ns - t_ref) / 1e9, imu_rate, lw=0.8, label="BNO055")
        fused = bool(result.checks.get("imu_fused"))
        label = "smoothed (IMU fused)" if fused else "tag smoothed"
        ax.plot((grid["stamp_ns"] - t_ref) / 1e9, grid["yaw_rate"], lw=0.8, label=label)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("yaw rate (rad/s)")
        ax.legend()
        fig.tight_layout()
        path = out_dir / "imu_vs_tag_yaw_rate.png"
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path.name)
    return written


def main(argv: list[str] | None = None) -> Path:
    args = parse_args(argv)
    session = args.session_name or args.mcap.stem
    out_dir = args.out_root / session
    out_dir.mkdir(parents=True, exist_ok=True)

    mounts, geometry = load_mass_properties(args.mass_properties)
    rest_pitch = args.rest_pitch_rad if args.rest_pitch_rad is not None else default_rest_pitch()
    frames = load_robot_tags(args.mcap)
    field = load_field_from_camera(args.mcap)
    esp32 = load_esp32_diagnostics(args.mcap)
    sticks = load_transmitter_channels(args.mcap)
    print(
        f"{len(frames)} tag frames, {len(esp32)} ESP32 events, {len(sticks)} stick updates, "
        f"{len(field)} camera poses"
    )

    clock_summary: dict[str, Any] | None = None
    if not esp32.empty:
        clock = fit_robot_clock(esp32["timestamp_ms"].to_numpy(), esp32["host_receive_ns"])
        esp32["stamp_ns"] = clock.stamp_ns
        esp32 = esp32.sort_values("stamp_ns", kind="stable").reset_index(drop=True)
        clock_summary = clock.summary()
    else:
        esp32["stamp_ns"] = pd.Series(dtype="int64")

    options = SmootherOptions(rest_pitch_rad=rest_pitch, fuse_imu=args.fuse_imu)
    result = smooth_session(frames, field, mounts, options, esp32 if not esp32.empty else None)

    window_opts = WindowOptions(
        window_s=args.window_s,
        coverage_min=args.coverage_min,
        box_half_extent_m=args.box_half_extent,
        box_center_xy=(float(args.box_center[0]), float(args.box_center[1])),
        robot_length_m=args.robot_length,
        rest_pitch_rad=rest_pitch,
        pitch_limit_deg=args.pitch_limit_deg,
        esp32_max_gap_s=args.esp32_max_gap_s,
    )
    meas = result.measurements
    windows, window_report = make_windows(
        result.grid,
        result.frame_stamps_ns,
        meas.loc[~meas["rejected"], "stamp_ns"].to_numpy(),
        esp32["stamp_ns"].to_numpy(),
        geometry,
        window_opts,
    )

    grid = result.grid[SMOOTHED_COLUMNS].copy()
    grid["has_measurement"] = grid["has_measurement"].astype(int)
    grid["upside_down"] = grid["upside_down"].astype(int)
    grid.to_csv(out_dir / "smoothed.csv", index=False)
    m_out = meas[MEASUREMENT_COLUMNS].copy()
    m_out["ippe_close_call"] = m_out["ippe_close_call"].astype(int)
    m_out["rejected"] = m_out["rejected"].astype(int)
    m_out.to_csv(out_dir / "measurements.csv", index=False)
    cmds = commands_frame(esp32)
    for col in ("is_upside_down", "armed", "radio_connected"):
        cmds[col] = cmds[col].astype(int)
    cmds.to_csv(out_dir / "commands.csv", index=False)
    windows.to_csv(out_dir / "windows.csv", index=False)

    radio = radio_delays(esp32, sticks, args.linear_channel, args.angular_channel)
    summary = build_session_json(
        result,
        clock_summary,
        radio,
        field_transform_constancy(field),
        window_report,
        frames_summary(frames, mounts),
        options,
    )
    summary["mcap"] = str(args.mcap)
    if args.plots:
        summary["plots"] = write_plots(out_dir, result)
    with open(out_dir / "session.json", "w") as handle:
        json.dump(jsonable(summary), handle, indent=2)

    floor = result.checks.get("noise_floor", {})
    print(f"wrote {out_dir}")
    print(
        f"  detections {len(meas)}, rejected {int(meas['rejected'].sum())}, "
        f"IPPE close calls {result.ippe.close_calls}"
    )
    print(
        f"  q_xy {result.cv_planar.chosen['xy']:.3g}, q_yaw {result.cv_planar.chosen['yaw']:.3g}, "
        f"q_tilt {result.cv_tilt.chosen['tilt']:.3g}"
    )
    if floor.get("detections"):
        print(
            f"  noise floor x {floor['sigma_x'] * 1e3:.2f} mm, y {floor['sigma_y'] * 1e3:.2f} mm, "
            f"yaw {math.degrees(floor['sigma_yaw']):.2f} deg"
        )
    print(f"  windows {window_report['windows']}, dropped {window_report['windows_dropped']}")
    return out_dir


if __name__ == "__main__":
    main()
