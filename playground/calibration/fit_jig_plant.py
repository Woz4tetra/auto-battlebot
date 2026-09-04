"""Fit the drivetrain plant model from velocity jig sessions.

Input is one or more session directories written by `velocity_jig_drive.py`. Each holds the
downloaded `LOG-N.TXT` files, a sidecar TOML per log carrying the experiment parameters, and
a command CSV. Output is a plant parameter TOML, a printed report, and optionally a
self-contained HTML report.

Runs reach the right fit by the waveform kind, channel and role their sidecar declares, so a
new excitation needs a catalog entry rather than a change here.

The fit machinery lives in `auto_battlebot/calibration/jig_fit.py`; this file is the command line
over it.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/fit_jig_plant.py \\
        playground/calibration/out/2026-08-17-garage \\
        --out playground/calibration/out/plant_params.toml \\
        --report playground/calibration/out/jig_fit.html
"""

from __future__ import annotations

import argparse
import dataclasses
from pathlib import Path

import numpy as np

from auto_battlebot.plant import (
    FULL_MODEL,
    MODEL_LADDER,
    PlantParams,
    predict_windows,
)
from auto_battlebot.velocity_jig import CALIBRATION_TEMPLATE, JigCalibration

DEFAULT_CALIBRATION = Path(__file__).resolve().parent / "jig_calibration.toml"

from auto_battlebot.calibration.jig_fit import (
    DelayProfile,
    FitWeights,
    HorizonReport,
    build_windows,
    cross_checks,
    joint_fit,
    load_all,
    print_horizon_table,
    print_stage_a,
    profile_delay,
    residual_autocorrelation,
    score,
    stage_a,
    window_delay,
    write_params,
)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "sessions",
        type=Path,
        nargs="*",
        help="session directories written by velocity_jig_drive.py",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help="jig calibration TOML (runbook block 0)",
    )
    parser.add_argument("--out", type=Path, default=None, help="write fitted parameters here")
    parser.add_argument(
        "--report", type=Path, default=None, help="write a self-contained HTML report here"
    )
    parser.add_argument(
        "--fit-hz",
        type=float,
        default=200.0,
        help=(
            "uniform analysis grid rate. This sets how finely the delay can be resolved: the pose"
            " integration lands the response within half a grid step, so a 200 Hz grid reports"
            " the delay to about 5 ms and a 500 Hz grid to about 1 ms, at 2.5x the fit time."
        ),
    )
    parser.add_argument(
        "--smooth-ms",
        type=float,
        default=20.0,
        help=(
            "encoder velocity smoothing. Wider is quieter against count quantization but rounds"
            " the corner of a step, which the fit pays back as a few ms of extra transport delay."
            " Worth sweeping once on real data."
        ),
    )
    parser.add_argument("--stride-ms", type=float, default=50.0, help="window start stride")
    parser.add_argument("--max-windows", type=int, default=4000)
    parser.add_argument(
        "--delay-windows", type=int, default=1200, help="windows for the delay profile"
    )
    parser.add_argument("--delay-step-ms", type=float, default=2.0)
    parser.add_argument("--delay-max-ms", type=float, default=120.0)
    parser.add_argument(
        "--track-width", type=float, default=0.10, help="meters, for the k_ang bound"
    )
    parser.add_argument("--model", default="M4", choices=[m.name for m in MODEL_LADDER])
    parser.add_argument(
        "--pin-deadzones",
        action="store_true",
        help="shorthand for --pin dz_lin_fwd,dz_lin_rev,dz_ang_l,dz_ang_r",
    )
    parser.add_argument(
        "--pin",
        default="",
        help="comma-separated parameters to hold at their stage A values instead of letting the"
        " joint fit move them. Use it where a dedicated excitation measures a parameter"
        " directly and the window objective only sees it through its effect on everything"
        " else, which is where the two disagree worst.",
    )
    parser.add_argument("--ladder", action="store_true", help="fit and score every model rung")
    parser.add_argument("--no-joint", action="store_true", help="stop after stage A")
    parser.add_argument("--no-delay-profile", action="store_true", help="keep the stage A delay")
    parser.add_argument("--keep-bad", action="store_true", help="do not exclude runs that fail QC")
    parser.add_argument(
        "--commands",
        choices=("measured", "requested"),
        default="measured",
        help=(
            "which command stream to fit. 'measured' is the radio's own report of what it"
            " sent, which is what the robot received. Use 'requested' when a run's readback"
            " was mis-decoded, since a wrong channel sign corrupts the measured stream while"
            " leaving what the tool asked for exact."
        ),
    )
    parser.add_argument(
        "--max-saturation",
        type=float,
        default=0.0,
        help=(
            "fraction of clipped IMU samples a run may have. 0 rejects any saturation, on"
            " the grounds that a clipped sample means an impact and its true value is"
            " unknown. Raise it to keep a run anyway."
        ),
    )
    parser.add_argument("--name", default="", help="report title")
    parser.add_argument(
        "--detail",
        choices=("none", "failing", "all"),
        default="failing",
        help="per-run measured-vs-predicted figures in the report (default: failing runs only)",
    )
    parser.add_argument(
        "--bootstrap",
        type=int,
        default=0,
        help=(
            "bootstrap resamples for the learning curve, which answers how much more data is"
            " needed. 0 runs the cheap leave-one-run-out jackknife only; 8 is a sensible start."
        ),
    )
    parser.add_argument(
        "--write-calibration-template",
        type=Path,
        default=None,
        help="write a blank jig calibration TOML and exit",
    )
    args = parser.parse_args()

    if args.write_calibration_template:
        args.write_calibration_template.write_text(CALIBRATION_TEMPLATE, encoding="utf-8")
        print(f"wrote {args.write_calibration_template}")
        return
    if not args.sessions:
        parser.error("at least one session directory is required")
    if not args.calibration.exists():
        parser.error(
            f"{args.calibration} not found. Fill it in from runbook block 0, or write a "
            "blank one with --write-calibration-template."
        )

    calib = JigCalibration.from_toml(args.calibration)
    print(f"calibration: {calib.meters_per_count:.8g} m/count, gyro scale {calib.gyro_scale:.4f}")

    loaded = load_all(
        args.sessions,
        calib,
        args.fit_hz,
        args.keep_bad,
        args.smooth_ms / 1000.0,
        commands=args.commands,
        max_saturation=args.max_saturation,
    )
    print(f"\nloaded {len(loaded.runs)} runs from {len(loaded.sessions)} session(s)")
    for name, why in loaded.excluded:
        print(f"  excluded {name}: {why}")
    if not loaded.runs:
        print("nothing to fit")
        return

    stage = stage_a(loaded, PlantParams())
    print_stage_a(stage)

    structure = next(m for m in MODEL_LADDER if m.name == args.model)
    pin = [n.strip() for n in args.pin.split(",") if n.strip()]
    if args.pin_deadzones:
        pin += ["dz_lin_fwd", "dz_lin_rev", "dz_ang_l", "dz_ang_r"]
    if pin:
        known = set(FULL_MODEL.free_names())
        unknown = [n for n in pin if n not in known]
        if unknown:
            raise SystemExit(f"--pin: not fittable parameters: {', '.join(unknown)}")
        pinned = tuple(dict.fromkeys(pin))
        structure = dataclasses.replace(structure, pinned=pinned)
        print(f"  pinned at their stage A values: {', '.join(pinned)}")
    params = stage.params
    profile: DelayProfile | None = None
    reports: dict[str, HorizonReport] = {}

    # Train and holdout come from the role each waveform declares, so a holdout stays a
    # holdout even if the same excitation is later reused for fitting.
    fit_runs = loaded.select(roles=("fit",))
    test_runs = loaded.select(roles=("holdout",))
    print(f"\ntrain runs: {len(fit_runs)}, holdout runs: {len(test_runs)}")

    # The plan's objective horizons, plus 400 ms because that is where acceptance criteria C2 and
    # C3 are stated and a table should measure the horizon it reports.
    horizons = (0.033, 0.066, 0.100, 0.200, 0.300, 0.400, 0.500)
    stride_s = args.stride_ms / 1000.0
    weights = FitWeights()

    if not args.no_joint and fit_runs:
        print("\n=== Stage B: joint fit by simulation error ===")
        if not args.no_delay_profile and structure.use_delay:
            grid = np.arange(0.0, args.delay_max_ms + 1e-9, args.delay_step_ms) / 1000.0
            print(f"  profiling delay over {len(grid)} points")
            profile = profile_delay(
                fit_runs,
                params,
                structure,
                weights,
                horizons,
                stride_s=stride_s,
                max_windows=args.delay_windows,
                delay_grid=grid,
                max_nfev=30,
            )
            params = profile.best_params
            print(f"  profile minimum at {profile.best_delay * 1000:.1f} ms")

        windows = build_windows(
            fit_runs, window_delay(params, structure), horizons, stride_s, args.max_windows
        )
        if windows is None:
            print("  no usable windows; keeping stage A parameters")
        else:
            print(f"  fitting {windows.count()} windows on {len(structure.free_names())} params")
            params, cost = joint_fit(windows, params, structure, weights)
            print(f"  final cost {cost:.4f}")
            reports["train"] = score(predict_windows(windows, params, structure))

            # Weight sensitivity: the fit should not depend much on how position and heading
            # residuals are traded off. A parameter that moves a lot here is being set by the
            # weighting rather than by the data.
            alt, _ = joint_fit(windows, params, structure, FitWeights(0.05, 0.025))
            moved = {
                n: abs(getattr(alt, n) - getattr(params, n)) / max(abs(getattr(params, n)), 1e-9)
                for n in structure.free_names()
            }
            worst = max(moved.items(), key=lambda kv: kv[1])
            print(f"  weight check (heading weight 2x): largest move {worst[0]} {worst[1]:.1%}")

    if test_runs:
        test_windows = build_windows(
            test_runs, window_delay(params, structure), horizons, stride_s, args.max_windows
        )
        if test_windows is not None:
            err = predict_windows(test_windows, params, structure)
            reports["holdout"] = score(err)
            print(f"\nholdout windows: {test_windows.count()}")
            print(
                f"  lag-1 residual autocorrelation at 500 ms: {residual_autocorrelation(err):.2f}"
            )

    print("\n=== Fitted parameters ===")
    for name, value in params.to_dict().items():
        print(f"  {name:12} {value:.6g}")

    for title, report in reports.items():
        print_horizon_table(f"{title} error vs horizon", report)

    if reports.get("holdout") is not None:
        rep = reports["holdout"]
        idx_100 = int(np.argmin(np.abs(rep.horizons - 0.100)))
        idx_400 = int(np.argmin(np.abs(rep.horizons - 0.400)))
        print("\n=== Acceptance criteria (holdout) ===")
        print(
            f"  C1 position RMSE at {rep.horizons[idx_100] * 1e3:.0f} ms:"
            f" {rep.rows['pos_rmse_mm'][idx_100]:.1f} mm (target < 15)"
        )
        print(
            f"  C2 position RMSE at {rep.horizons[idx_400] * 1e3:.0f} ms:"
            f" {rep.rows['pos_rmse_mm'][idx_400]:.1f} mm (target < 80)"
        )
        print(
            f"  C3 heading RMSE at {rep.horizons[idx_400] * 1e3:.0f} ms:"
            f" {rep.rows['head_rmse_deg'][idx_400]:.2f} deg (target < 8)"
        )

    if args.ladder and fit_runs and test_runs:
        print("\n=== Model ladder ===")
        print("    model | train 500 ms RMSE mm | holdout 500 ms RMSE mm | params")
        prev_holdout = None
        for rung in MODEL_LADDER:
            rung_params = stage.params
            train_ws = build_windows(
                fit_runs, window_delay(rung_params, rung), horizons, stride_s, args.delay_windows
            )
            if train_ws is None:
                continue
            rung_params, _ = joint_fit(train_ws, rung_params, rung, weights, max_nfev=60)
            test_ws = build_windows(
                test_runs, window_delay(rung_params, rung), horizons, stride_s, args.delay_windows
            )
            train_rmse = score(predict_windows(train_ws, rung_params, rung)).rows["pos_rmse_mm"][-1]
            hold_rmse = (
                score(predict_windows(test_ws, rung_params, rung)).rows["pos_rmse_mm"][-1]
                if test_ws is not None
                else float("nan")
            )
            gain = (
                ""
                if prev_holdout is None or not np.isfinite(hold_rmse)
                else f"  ({100 * (prev_holdout - hold_rmse) / prev_holdout:+.1f}% vs previous)"
            )
            print(
                f"    {rung.name:5} | {train_rmse:20.1f} | {hold_rmse:22.1f} |"
                f" {len(rung.free_names())}{gain}"
            )
            if np.isfinite(hold_rmse):
                prev_holdout = hold_rmse
        print("    A term that does not buy 5% at the long horizon does not ship.")

    if len(loaded.sessions) > 1:
        print("\n=== Leave-one-session-out ===")
        for held in loaded.sessions:
            train, test = loaded.split_session(held.session_id)
            delay = window_delay(params, structure)
            train_ws = build_windows(train, delay, horizons, stride_s, args.delay_windows)
            test_ws = build_windows(test, delay, horizons, stride_s, args.delay_windows)
            if train_ws is None or test_ws is None:
                print(f"  {held.session_id}: not enough windows")
                continue
            fitted, _ = joint_fit(train_ws, stage.params, structure, weights, max_nfev=80)
            rmse = score(predict_windows(test_ws, fitted, structure)).rows["pos_rmse_mm"][-1]
            print(f"  holding out {held.session_id}: 500 ms position RMSE {rmse:.1f} mm")

    print("\n=== Cross-checks ===")
    for line in cross_checks(params, args.track_width):
        print(f"  {line}")

    if args.out:
        note = f"fit from {', '.join(str(s) for s in args.sessions)} with model {structure.name}"
        write_params(args.out, params, stage, note, structure)
    if args.report:
        # Imported here so a fit that writes no report never pays for matplotlib.
        from jig_report import render_report

        render_report(
            args.report,
            loaded=loaded,
            stage=stage,
            params=params,
            structure=structure,
            profile=profile,
            reports=reports,
            fit_runs=fit_runs,
            test_runs=test_runs,
            horizons=horizons,
            stride_s=stride_s,
            max_windows=args.max_windows,
            weights=weights,
            detail=args.detail,
            bootstrap=args.bootstrap,
            title=args.name or ", ".join(s.name for s in args.sessions),
            track_width=args.track_width,
        )


if __name__ == "__main__":
    main()
