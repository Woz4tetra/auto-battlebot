from __future__ import annotations

import re
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from auto_battlebot.recording import diag_io
from auto_battlebot.recording.mcap_io import DiagnosticStatus

REPO = Path(__file__).resolve().parents[2]
TICK_SCHEMA_KEY = "angle_error_deg"


def _status(hardware_id: str, name: str, **values: Any) -> DiagnosticStatus:
    return {
        "level": 0,
        "name": name,
        "message": "",
        "hardware_id": hardware_id,
        "values": values,
    }


def test_nav_hw_ids_cover_every_registered_navigation_logger() -> None:
    logged = set()
    for source in sorted((REPO / "src" / "navigation").glob("*.cpp")):
        text = source.read_text()
        if TICK_SCHEMA_KEY not in text:
            continue
        logged.update(re.findall(r'get_logger\("([a-z_]+)"\)', text))
    assert logged, "no navigation logger found"
    assert logged == set(diag_io.NAV_HW_IDS)


def test_frame_ids_match_frame_id_enum_header() -> None:
    header = (REPO / "include" / "enums" / "frame_id.hpp").read_text()
    body = re.search(r"enum class FrameId\s*\{(.*?)\}", header, re.S)
    assert body is not None
    names = [n.strip() for n in body.group(1).split(",") if n.strip()]
    assert diag_io.FRAME_IDS == names


def test_unrelated_status_adds_no_row() -> None:
    rows: dict[int, dict[str, Any]] = defaultdict(dict)
    diag_io._merge_status(rows, 10, _status("perception_node", "detector", loss=0.5))
    diag_io._merge_status(rows, 11, _status(diag_io.RUNNER_HW_ID, "unlisted", other=1))
    assert len(rows) == 0
    diag_io._merge_status(rows, 12, _status(diag_io.PURSUIT_NAV_HW_ID, "pursuit_nav", distance=1))
    assert dict(rows) == {12: {"distance": 1}}


def test_transmitter_channels_forward_fill() -> None:
    df = pd.DataFrame(
        {
            "ch_linear": [0.5, np.nan, np.nan],
            "ch_angular": [-0.25, np.nan, 0.75],
            "ch15": [1024, np.nan, np.nan],
        }
    )
    diag_io._fill_latched_channels(df)
    assert list(df["ch_linear"]) == [0.5, 0.5, 0.5]
    assert list(df["ch_angular"]) == [-0.25, -0.25, 0.75]
    assert list(df["is_auto"]) == [True, True, True]
