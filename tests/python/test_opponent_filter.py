"""The Python opponent filter mirror against the golden fixture the C++ test also replays."""

from __future__ import annotations

import math
import re

import numpy as np
import pytest
import torch

from auto_battlebot.perception.opponent_filter import OpponentFilter, OpponentFilterConfig
from playground.opponent_prediction import make_opponent_filter_golden_fixture as golden


def _parse_header() -> dict[str, np.ndarray]:
    text = golden.OUT_HEADER.read_text()
    arrays: dict[str, np.ndarray] = {}
    for name, body in re.findall(r"inline constexpr \w+ (\w+)\[\] = \{(.*?)\};", text, re.S):
        values = [v.strip() for v in body.replace("\n", " ").split(",") if v.strip()]
        arrays[name] = np.array(
            [math.inf if v == "kInf" else -math.inf if v == "-kInf" else float(v) for v in values]
        )
    return arrays


def test_mirror_reproduces_golden_fixture() -> None:
    arrays = _parse_header()
    events = golden.scenario()
    np.testing.assert_array_equal(arrays["kEventKind"], [e.kind for e in events])
    np.testing.assert_allclose(arrays["kEventTime"], [e.time for e in events], atol=1e-12)

    records = golden.run_mirror(events)
    np.testing.assert_array_equal(arrays["kRecorded"], records.recorded)
    render = np.array(records.render)
    expected = arrays["kRender"].reshape(render.shape)
    np.testing.assert_allclose(render, expected, atol=1e-9)
    state = np.array(records.state)
    np.testing.assert_allclose(state, arrays["kState"].reshape(state.shape), atol=1e-9)


def _run_rows(filt: OpponentFilter, events: list[golden.Event]) -> list[np.ndarray]:
    rows = []
    b = filt.batch
    for ev in events:
        if ev.kind == golden.KIND_COAST:
            out = filt.coast(ev.time)
        else:
            out, _ = filt.update(
                ev.time,
                torch.full((b,), ev.kind == golden.KIND_MEASURED),
                torch.tensor([[ev.x, ev.y]] * b, dtype=torch.float64),
                torch.tensor([ev.yaw] * b, dtype=torch.float64),
                torch.full((b,), bool(ev.keypoint)),
                (golden.KEYPOINT_SIGMA if ev.keypoint else golden.BLOB_SIGMA) ** 2,
            )
        rows.append(torch.stack([out.x, out.y, out.yaw, out.position_sigma_m], dim=1).numpy())
    return rows


@pytest.mark.parametrize("row", [0, 1, 2])
def test_batched_rows_match_single_row_runs(row: int) -> None:
    """Rows with different tuning in one batch equal each tuning run alone."""
    events = golden.scenario()[:400]
    psd = [10.0, 30.0, 90.0]
    yaw_psd = [30.0, 100.0, 300.0]
    half = torch.tensor([[2.0, 2.0]] * 3, dtype=torch.float64)
    batched = OpponentFilter(
        3,
        OpponentFilterConfig(
            opponent_accel_psd=torch.tensor(psd),
            opponent_yaw_accel_psd=torch.tensor(yaw_psd),
            opponent_max_coast_s=0.2,
        ),
        field_half_extent=half,
        margin=0.2,
    )
    single = OpponentFilter(
        1,
        OpponentFilterConfig(
            opponent_accel_psd=psd[row],
            opponent_yaw_accel_psd=yaw_psd[row],
            opponent_max_coast_s=0.2,
        ),
        field_half_extent=half[:1],
        margin=0.2,
    )
    for a, b in zip(_run_rows(batched, events), _run_rows(single, events), strict=True):
        np.testing.assert_allclose(a[row], b[0], atol=1e-12)
