"""Build-time guards for the TX16S HR10 clamp and mount parts (build123d, skill venv)."""

import dataclasses
import importlib
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

HARDWARE = Path(__file__).resolve().parents[2] / "hardware"


def _load(monkeypatch: pytest.MonkeyPatch, name: str) -> ModuleType:
    pytest.importorskip("build123d")
    monkeypatch.syspath_prepend(str(HARDWARE / name))
    return importlib.import_module(name)


def _check_exit_code(part: Path, params: str) -> int:
    recipe = pytest.importorskip("recipe")
    check = Path(recipe.__file__).parent / "check.py"
    cmd = [sys.executable, str(check), str(part), "--no-render", "--no-export", "--params", params]
    return subprocess.run(cmd, capture_output=True, check=False).returncode


def test_handle_clamp_builds_with_the_default_step_nudge(monkeypatch: pytest.MonkeyPatch) -> None:
    clamp = _load(monkeypatch, "tx16s_hr10_handle_clamp")
    assert clamp.build(clamp.Params()).is_valid


def test_handle_clamp_bore_loft_fails_without_the_step_nudge(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clamp = _load(monkeypatch, "tx16s_hr10_handle_clamp")
    params = dataclasses.replace(clamp.Params(), step_nudge=0.0)
    with pytest.raises(Exception, match="not done"):
        clamp.bore_tool(params, upper=True)


def test_mount_passes_with_the_tight_back_right_corner() -> None:
    part = HARDWARE / "tx16s_hr10_mount" / "tx16s_hr10_mount.py"
    assert _check_exit_code(part, "{}") == 0


def test_mount_leaves_a_knife_edge_with_a_uniform_corner_radius() -> None:
    part = HARDWARE / "tx16s_hr10_mount" / "tx16s_hr10_mount.py"
    assert _check_exit_code(part, '{"r_taper_corner": 3.0}') == 1


def test_rod_cylinder_faces_all_report_a_radius(monkeypatch: pytest.MonkeyPatch) -> None:
    build123d = pytest.importorskip("build123d")

    clamp = _load(monkeypatch, "tx16s_hr10_rod_clamp")
    rod = clamp.rod(clamp.Params())
    radii = [f.radius for f in rod.faces() if f.geom_type == build123d.GeomType.CYLINDER]
    assert radii
    assert None not in radii


def test_a_swept_straight_hole_reports_no_radius() -> None:
    build123d = pytest.importorskip("build123d")
    cutter = build123d.Solid.sweep(
        build123d.Wire.make_circle(3, build123d.Plane(origin=(-30, 0, 0), z_dir=(1, 0, 0))),
        build123d.Wire([build123d.Edge.make_line((-30, 0, 0), (30, 0, 0))]),
    )
    block = build123d.Box(40, 40, 10) - cutter
    radii = [f.radius for f in block.faces() if f.geom_type == build123d.GeomType.CYLINDER]
    assert radii == [None]
