"""Cable routing guards in hardware/cage_pack/cage_pack_assembly.py."""

import importlib
import math
import os
import subprocess
import sys
from types import ModuleType

import pytest

from auto_battlebot.paths import REPO_ROOT

CAGE_PACK = REPO_ROOT / "hardware" / "cage_pack"


@pytest.fixture()
def assembly(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    pytest.importorskip("build123d")
    pytest.importorskip("recipe")
    monkeypatch.syspath_prepend(str(CAGE_PACK))
    return importlib.import_module("cage_pack_assembly")


def test_run_rejects_corner_below_bend_radius(assembly: ModuleType) -> None:
    min_bend_r = assembly.CABLES["usb"].min_bend_r
    short_leg = min_bend_r / 4
    points = [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (100.0, short_leg, 0.0), (100.0, 100.0, 0.0)]
    with pytest.raises(ValueError, match="corners need"):
        assembly._run("usb", points)


def test_run_builds_a_corner_wider_than_the_bend_radius(assembly: ModuleType) -> None:
    points = [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (100.0, 100.0, 0.0)]
    _, length = assembly._run("usb", points)
    assert length < 200.0


def test_run_rejects_a_segment_with_no_straight_between_fillets(assembly: ModuleType) -> None:
    radius = assembly.CABLES["usb"].min_bend_r
    setback = assembly._fillet_setback(radius, math.pi / 2)
    points = [
        (0.0, 0.0, 0.0),
        (50.0, 0.0, 0.0),
        (50.0, 2 * setback, 0.0),
        (100.0, 2 * setback, 0.0),
    ]
    with pytest.raises(ValueError, match="corners need"):
        assembly._run("usb", points)


def test_opencascade_cannot_sweep_a_fillet_path_with_no_straight(assembly: ModuleType) -> None:
    from build123d import Circle, FilletPolyline, Plane, Transition, Vector, sweep

    radius = assembly.CABLES["usb"].min_bend_r
    setback = assembly._fillet_setback(radius, math.pi / 2)
    pts = [Vector(*q) for q in [(0, 0, 0), (50, 0, 0), (50, 2 * setback, 0), (100, 2 * setback, 0)]]
    path = FilletPolyline(*pts, radius=radius)
    start = path.edges()[0]
    plane = Plane(origin=start.position_at(0), z_dir=start.tangent_at(0))
    with pytest.raises(Exception):  # noqa: B017
        sweep(plane * Circle(1.0), path=path, transition=Transition.ROUND)


def test_every_cable_run_sweeps_valid_with_round_transitions(assembly: ModuleType) -> None:
    for name, (shape, _) in assembly._cable_runs().items():
        assert shape.is_valid, name


def test_tube_builds_through_collinear_and_repeated_points(assembly: ModuleType) -> None:
    connectors = importlib.import_module("connectors")
    collinear = [(0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (100.0, 0.0, 0.0), (100.0, 50.0, 0.0)]
    repeated = [(0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 0.0, 0.0), (100.0, 50.0, 0.0)]
    for points in (collinear, repeated):
        assert connectors.tube(points, 3.0, bend_r=5.0).is_valid


def test_hr10_plug_keeps_only_solids_behind_the_receptacle_front(assembly: ModuleType) -> None:
    plug = assembly._hr10_plug()
    body = [s for s in plug.solids() if s.bounding_box().max.Z < 0.0]
    assert len(body) == len(plug.solids())


def test_cutting_the_hr10_nose_from_the_receptacle_crashes_opencascade(
    assembly: ModuleType,
) -> None:
    script = (
        "import cage_pack_assembly as m, hr10\n"
        "full = m._vendor('grabcad/hr10_plug_6p.step', (0, 0, 4.28), (1, 0, 0), (0, 0, 1))\n"
        "nose = [s for s in full.solids() if s.bounding_box().max.Z >= 0.0]\n"
        "assert nose\n"
        "hr10.receptacle(m.cl.HR10_PANEL_T) - nose[0]\n"
    )
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)}
    result = subprocess.run([sys.executable, "-c", script], env=env, cwd=CAGE_PACK)
    assert result.returncode < 0
