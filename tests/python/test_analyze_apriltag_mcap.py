"""The truth CSV header carries stick or command columns by name, only when present."""

from __future__ import annotations

from pathlib import Path

from playground.calibration import analyze_apriltag_mcap as analyze

BASE = ["t", "x", "y", "yaw", "visible"]


def _rows(with_sticks: bool, with_cmd: bool) -> list[dict]:
    rows: list[dict] = []
    for i in range(3):
        row: dict = {"t": 1.0 + i, "visible": 1, "x": 0.1, "y": 0.2, "yaw": 0.3}
        if with_sticks:
            row.update({"sample_t": 0.99 + i, "channels": [1, 2]})
        if with_cmd:
            row.update(
                {"cmd_lin": 0.5, "cmd_ang": 0.0, "trainer_lin": 3, "trainer_ang": 4, "label": "a"}
            )
        rows.append(row)
    return rows


def _header(path: Path) -> list[str]:
    return path.read_text().splitlines()[0].split(",")


def test_truth_csv_columns(tmp_path: Path) -> None:
    plain = tmp_path / "plain.csv"
    analyze.write_csv(plain, _rows(False, False))
    assert _header(plain) == BASE

    sticks = tmp_path / "sticks.csv"
    analyze.write_csv(sticks, _rows(True, False))
    assert _header(sticks) == [*BASE, "tx_sample_t", "ch0", "ch1"]

    drive = tmp_path / "drive.csv"
    analyze.write_csv(drive, _rows(False, True))
    assert _header(drive) == [*BASE, *analyze.CMD_COLUMNS]
    assert drive.read_text().splitlines()[1].endswith("0.5000,0.0000,3,4,a")
