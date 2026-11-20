"""Push the fitted plant numbers from plant_stageA.toml into every consumer that hard-codes them."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

from auto_battlebot.compat import tomllib

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_FIT = REPO_ROOT / "playground" / "calibration" / "out" / "plant_stageA.toml"
SIM_CONFIG = REPO_ROOT / "simulation" / "sim_mrs_buff_mk3.toml"
PLANT_CONFIG = REPO_ROOT / "config" / "robots" / "mrs_buff_mk3.toml"
MR_STABS_PLANT_CONFIG = REPO_ROOT / "config" / "robots" / "mr_stabs_mk2.toml"

SIM_FIELDS = {
    "command_ms": "delay_ms",
    "max_linear_speed": "k_fwd",
    "max_angular_speed": "k_ang",
    "tau_linear": "tau_lin_a",
    "tau_angular": "tau_ang_a",
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

PLANT_FIELDS = {
    name: name
    for name in (
        "dz_lin_fwd",
        "dz_lin_rev",
        "dz_ang_l",
        "dz_ang_r",
        "k_fwd",
        "k_rev",
        "k_ang",
        "tau_lin_a",
        "tau_lin_d",
        "tau_ang_a",
        "tau_ang_d",
        "delay_s",
        "c_sb",
        "c_ad",
        "c_drift",
        "c_drift_bias",
    )
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
    if "." not in text or "e" in text:
        return text
    text = text.rstrip("0").rstrip(".")
    return text if "." in text else text + ".0"


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
        matches = list(field_re.finditer(text))
        if len(matches) > 1:
            raise SystemExit(f"{path}: field '{field_name}' appears {len(matches)} times")
        match = matches[0] if matches else None
        if match is None:
            raise SystemExit(f"{path}: no field '{field_name}' to write (renamed or removed?)")
        old = match.group("value")
        if old != new:
            changes.append(Change(path, field_name, old, new))
            text = text[: match.start("value")] + new + text[match.end("value") :]
    return text, changes


NUMBER = r"(?P<value>-?\d+\.?\d*(?:[eE][-+]?\d+)?)"
TOML_FIELD = r"^\s*{field}\s*=\s*" + NUMBER


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit", type=Path, default=None, help="fitted plant TOML")
    parser.add_argument(
        "--robot",
        choices=("mrs_buff_mk3", "mr_stabs_mk2"),
        default="mrs_buff_mk3",
        help="which robot's configs to write (mr_stabs_mk2 needs --fit)",
    )
    parser.add_argument(
        "--check", action="store_true", help="report drift and exit 1; write nothing"
    )
    args = parser.parse_args()

    if args.fit is None:
        if args.robot == "mr_stabs_mk2":
            raise SystemExit("--robot mr_stabs_mk2 needs --fit (distill_mujoco_plant.py output)")
        args.fit = DEFAULT_FIT
    args.fit = args.fit.resolve()
    if not args.fit.is_file():
        raise SystemExit(f"no fit at {args.fit}; run playground/calibration/fit_jig_plant.py first")
    plant = tomllib.loads(args.fit.read_text())["plant"]
    plant["delay_ms"] = plant["delay_s"] * 1000.0

    all_changes: list[Change] = []
    if args.robot == "mr_stabs_mk2":
        targets = [(MR_STABS_PLANT_CONFIG, TOML_FIELD, PLANT_FIELDS)]
    else:
        targets = [(SIM_CONFIG, TOML_FIELD, SIM_FIELDS), (PLANT_CONFIG, TOML_FIELD, PLANT_FIELDS)]
    for path, pattern, fields in targets:
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
