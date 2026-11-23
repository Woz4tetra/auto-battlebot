"""lump_mr_stabs_mass_properties writes inertia in MuJoCo's fullinertia order."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from auto_battlebot.mujoco_sim import lump_mr_stabs_mass_properties as lump
from auto_battlebot.mujoco_sim.onshape_export import MassBody


def test_fullinertia_order() -> None:
    inertia = np.array([[1.0, 4.0, 5.0], [4.0, 2.0, 6.0], [5.0, 6.0, 3.0]])
    body = lump._body(MassBody(1.0, np.zeros(3), inertia))
    assert body["fullinertia_kg_m2"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_cli_regenerates_the_committed_mass_table(tmp_path: Path) -> None:
    import subprocess
    import sys

    from auto_battlebot.paths import REPO_ROOT

    out = tmp_path / "mass_properties.toml"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "auto_battlebot.mujoco_sim.lump_mr_stabs_mass_properties",
            "--out",
            str(out),
        ],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
    )
    committed = REPO_ROOT / "simulation/assets/robots/mr_stabs_mk2/mass_properties.toml"
    assert out.read_text() == committed.read_text()
