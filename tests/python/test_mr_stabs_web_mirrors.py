"""The Mr Stabs diagnostics page mirrors firmware constants and CSV column order."""

from __future__ import annotations

import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP_JS = (ROOT / "firmware/mr_stabs_mk2/web/app.js").read_text()
MOTOR_GUARD_H = (ROOT / "firmware/mr_stabs_mk2/lib/motor_guard/motor_guard.h").read_text()
TWITCH_PY = (ROOT / "firmware/mr_stabs_mk2/scripts/log_drive_twitch.py").read_text()


def js_header() -> list[str]:
    match = re.search(r'const HEADER =\s*"([^"]+)"', APP_JS)
    assert match
    return match.group(1).split(",")


def twitch_columns() -> list[str]:
    values: dict[str, list[str]] = {}
    for node in ast.parse(TWITCH_PY).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in ("COLUMNS", "GUARD_COLUMNS") and isinstance(node.value, ast.List):
                values[name] = [ast.literal_eval(e) for e in node.value.elts]
            elif name == "COLUMNS" and isinstance(node.value, ast.BinOp):
                values[name] = values["COLUMNS"] + values["GUARD_COLUMNS"] + ["seq"]
    return values["COLUMNS"]


def js_constant(name: str) -> float:
    match = re.search(rf"const {name} = ([0-9.]+);", APP_JS)
    assert match, name
    return float(match.group(1))


def motor_guard_default(name: str) -> float:
    match = re.search(rf"float {name} = ([0-9.]+)f;", MOTOR_GUARD_H)
    assert match, name
    return float(match.group(1))


def test_events_csv_header_matches_jetson_logger_columns() -> None:
    assert js_header() == twitch_columns()


def test_heat_panel_bands_match_motor_guard_defaults() -> None:
    assert js_constant("RISE_SOFT_C") == motor_guard_default("rise_soft_c")
    assert js_constant("RISE_MAX_C") == motor_guard_default("rise_max_c")
