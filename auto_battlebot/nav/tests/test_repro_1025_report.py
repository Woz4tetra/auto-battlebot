"""The repro_1025 report CLI turns arm traces into the reverse-window table."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from auto_battlebot.paths import REPO_ROOT


def test_report_cli_summarizes_each_arm_against_the_field(tmp_path: Path) -> None:
    field = {
        "windows": 12,
        "seconds": 30.0,
        "median_wheel_pct": -55.0,
        "ci_low": -60.0,
        "ci_high": -50.0,
        "mean_commanded": -0.9,
        "mean_sent": -0.7,
        "chatter": 0.1,
    }
    (tmp_path / "field.json").write_text(json.dumps({"pooled": field}))
    (tmp_path / "arms.json").write_text(json.dumps({"both": [1.33, 0.48], "neither": [100.0, 1.0]}))
    forward = [[0.4, 0.4, 20.0]] * 50
    reverse = [[-1.0, -0.6, -40.0]] * 150
    episodes = [
        {"outcome": "TIMEOUT", "extra": {"trace": [v for row in forward + reverse for v in row]}},
        {"outcome": "CLIENT_LOST", "extra": {"trace": [-1.0, -1.0, -90.0]}},
    ]
    (tmp_path / "arm_both.jsonl").write_text("\n".join(json.dumps(e) for e in episodes) + "\n")
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "auto_battlebot.nav.repro_1025",
            "report",
            "--out-dir",
            str(tmp_path),
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert "| field 10/25 |" in result.stdout
    assert "| both | 1.33 | 0.48 | 1 | 1 |" in result.stdout
    assert "| neither |" not in result.stdout
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["both"]["episodes"] == 1
    assert summary["both"]["windows"] == 1
    assert summary["both"]["median_wheel_pct"] == -40.0
    assert summary["field"] == field
