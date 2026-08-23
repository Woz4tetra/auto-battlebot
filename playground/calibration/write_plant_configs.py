"""Push the fitted plant numbers from plant_stageA.toml into every consumer that hard-codes them.

The plant numbers have three homes: the fit (`playground/calibration/out/plant_stageA.toml`), the
kinematic sim (`simulation/sim_mrs_buff_mk3.toml`), and the controller defaults
(`include/navigation/config.hpp`). Only the first is authored. They drifted apart once already,
which is how MotionProfileNavigation ended up running Mr Stabs Mk2's time constants on Mrs Buff
Mk3, so this script makes the other two derived.

It rewrites the numeric literal of each mapped field in place and touches nothing else. The
hand-written doc comments around those fields explain what each number does and why, which a
block regenerator would flatten, so they are left alone. That also means adding a field to the
fit does not automatically give it a home here: add it to the tables below and write the comment
by hand.

`--check` rewrites nothing and exits 1 if any consumer disagrees with the fit. Run it after a
refit, and in CI if the fit ever starts changing on its own.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/write_plant_configs.py           # rewrite
    python playground/calibration/write_plant_configs.py --check   # report drift, change nothing
"""

from __future__ import annotations

import argparse
import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_FIT = REPO_ROOT / "playground" / "calibration" / "out" / "plant_stageA.toml"
SIM_CONFIG = REPO_ROOT / "simulation" / "sim_mrs_buff_mk3.toml"
NAV_CONFIG = REPO_ROOT / "include" / "navigation" / "config.hpp"

# Consumer field name -> fit parameter name. The sim models the plant, so it takes every term
# including the ones the controller deliberately ignores. `delay_ms` is derived below; the sim
# takes the transport delay in milliseconds under [latency] rather than in seconds under
# [our_robot]. Matching is by field name over the whole file, not per section, so these names have
# to stay unique within the config.
SIM_FIELDS = {
    "command_ms": "delay_ms",
    "max_linear_speed": "k_fwd",  # symmetric fallback, unused once _fwd/_rev are set
    "max_angular_speed": "k_ang",
    "tau_linear": "tau_lin_a",  # fallback
    "tau_angular": "tau_ang_a",  # fallback
    "max_linear_speed_fwd": "k_fwd",
    "max_linear_speed_rev": "k_rev",
    "tau_linear_accel": "tau_lin_a",
    "tau_linear_decel": "tau_lin_d",
    "tau_angular_accel": "tau_ang_a",
    "tau_angular_decel": "tau_ang_d",
    "steer_brake_coeff": "c_sb",
    "angular_droop_coeff": "c_ad",
    "deadzone_linear": "dz_lin_fwd",
    "deadzone_linear_rev": "dz_lin_rev",
    "deadzone_angular": "dz_ang_l",
    "deadzone_angular_right": "dz_ang_r",
}

# The controller inverts the plant, so it takes the same terms under its own names.
#
# `angular_droop_coeff` is absent on purpose. The plant does droop (c_ad = 0.463) but compensating
# for it measured worse: on a 90-degree turning approach it took terminal error from 0.008 m to
# 0.067 m and time-to-goal from 1.10 s to 2.83 s, because the harder turn it commands is exactly
# what the steer-brake term then charges forward speed for. The controller default stays 0.0 and
# is not derived from the fit. `c_drift`/`c_drift_bias` are absent because model M4 disables them.
NAV_MOTION_PROFILE = {
    "max_linear_speed_fwd": "k_fwd",
    "max_linear_speed_rev": "k_rev",
    "tau_accel": "tau_lin_a",
    "tau_decel": "tau_lin_d",
    "latency": "delay_s",
    "steer_brake_coeff": "c_sb",
    "angular_deadzone_left": "dz_ang_l",
    "angular_deadzone_right": "dz_ang_r",
    "max_angular_speed": "k_ang",
    "tau_angular_accel": "tau_ang_a",
    "tau_angular_decel": "tau_ang_d",
}


@dataclass
class Change:
    path: Path
    field: str
    old: str
    new: str


def _format(value: float) -> str:
    """Round-trip float formatting, trimmed. Keeps the file readable without losing the fit."""
    text = repr(round(value, 9))
    return text.rstrip("0").rstrip(".") if "." in text and "e" not in text else text


def _rewrite(
    text: str, pattern: str, fields: dict[str, str], plant: dict[str, float], path: Path
) -> tuple[str, list[Change]]:
    """Replace the numeric literal of each mapped field, leaving comments and layout alone."""
    changes: list[Change] = []
    for field_name, fit_name in fields.items():
        if fit_name not in plant:
            raise SystemExit(f"{path}: fit has no parameter '{fit_name}' (for '{field_name}')")
        new = _format(plant[fit_name])
        field_re = re.compile(pattern.format(field=re.escape(field_name)), re.MULTILINE)
        match = field_re.search(text)
        if match is None:
            raise SystemExit(f"{path}: no field '{field_name}' to write (renamed or removed?)")
        old = match.group("value")
        if old != new:
            changes.append(Change(path, field_name, old, new))
            text = text[: match.start("value")] + new + text[match.end("value") :]
    return text, changes


NUMBER = r"(?P<value>-?\d+\.?\d*(?:[eE][-+]?\d+)?)"
TOML_FIELD = r"^\s*{field}\s*=\s*" + NUMBER
CPP_FIELD = r"^\s*double\s+{field}\s*=\s*" + NUMBER


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit", type=Path, default=DEFAULT_FIT, help="fitted plant TOML")
    parser.add_argument(
        "--check", action="store_true", help="report drift and exit 1; write nothing"
    )
    args = parser.parse_args()

    if not args.fit.is_file():
        raise SystemExit(f"no fit at {args.fit}; run playground/calibration/fit_jig_plant.py first")
    plant = tomllib.loads(args.fit.read_text())["plant"]
    plant["delay_ms"] = plant["delay_s"] * 1000.0

    all_changes: list[Change] = []
    for path, pattern, fields in (
        (SIM_CONFIG, TOML_FIELD, SIM_FIELDS),
        (NAV_CONFIG, CPP_FIELD, NAV_MOTION_PROFILE),
    ):
        text = path.read_text()
        updated, changes = _rewrite(text, pattern, fields, plant, path)
        all_changes.extend(changes)
        if changes and not args.check:
            path.write_text(updated)

    if not all_changes:
        print(f"Up to date with {args.fit.relative_to(REPO_ROOT)}.")
        return 0

    verb = "drifted from" if args.check else "updated from"
    print(f"{len(all_changes)} field(s) {verb} {args.fit.relative_to(REPO_ROOT)}:")
    for change in all_changes:
        rel = change.path.relative_to(REPO_ROOT)
        print(f"  {rel}: {change.field}  {change.old} -> {change.new}")
    if args.check:
        print("\nRun without --check to write these, then rebuild.")
        return 1
    print("\nRebuild for the C++ defaults to take effect: ./scripts/build.sh")
    return 0


if __name__ == "__main__":
    sys.exit(main())
