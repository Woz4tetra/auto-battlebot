"""make_print_tags prints the board the floor solve expects and a robot tag the floor cannot claim."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from auto_battlebot.calibration.apriltag import apriltag_detect as ad
from playground.calibration import make_print_tags

COLS, ROWS, MARKER_M, SEP_M, FIRST_ID = 3, 5, 0.065, 0.015, 160


def _printed_image(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> np.ndarray:
    saved: dict[str, Any] = {}
    monkeypatch.setattr(
        make_print_tags, "save_pdf", lambda img, px_per_mm, out: saved.update(img=img)
    )
    make_print_tags.floor_grid_pdf(
        COLS, ROWS, MARKER_M, SEP_M, FIRST_ID, 20.0, 8.0, tmp_path / "floor.pdf"
    )
    return saved["img"]


def test_floor_ids_match_board(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    printed = _printed_image(monkeypatch, tmp_path)
    board = ad.make_floor_board(COLS, ROWS, MARKER_M, SEP_M, FIRST_ID)
    margin = round(20.0 * 8.0)
    expected = board.generateImage((printed.shape[1], printed.shape[0]), marginSize=margin)
    np.testing.assert_array_equal(printed, expected)


def test_robot_tag_id_is_outside_floor_range(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: dict[str, Any] = {}
    monkeypatch.setattr(make_print_tags, "floor_grid_pdf", lambda *a, **k: captured.update(floor=a))
    monkeypatch.setattr(make_print_tags, "robot_tag_pdf", lambda *a, **k: captured.update(tag=a))
    monkeypatch.setattr(sys, "argv", ["make_print_tags.py"])
    make_print_tags.main()
    cols, rows, _size, _sep, first_id, *_ = captured["floor"]
    floor_ids = range(first_id, first_id + cols * rows)
    assert captured["tag"][0] not in floor_ids
