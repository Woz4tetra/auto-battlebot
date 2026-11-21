"""sim_stop_test reads the sim port from the C++ profile instead of repeating it."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from auto_battlebot.calibration import sim_stop_test as stop
from auto_battlebot.compat import tomllib


def test_python_sim_server_listens_where_the_cpp_profile_connects(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(stop, "sim_port", lambda: 15999)
    case = SimpleNamespace(start=SimpleNamespace(x_m=0.0, y_m=0.0, yaw_deg=0.0), goal=(1.0, 0.0))
    out = tmp_path / "sim.toml"
    stop.write_sim_config(case, tmp_path / "fit.json", 20.0, 5.0, tmp_path / "trace.csv", out)  # type: ignore[arg-type]
    written = tomllib.loads(out.read_text())
    assert written["server"]["port"] == 15999


def test_sim_port_comes_from_the_cpp_profile() -> None:
    assert stop.sim_port() == 14882
