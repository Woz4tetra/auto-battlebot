"""lump_mr_stabs_mass_properties writes inertia in MuJoCo's fullinertia order."""

from __future__ import annotations

import numpy as np

from auto_battlebot.mujoco_sim import lump_mr_stabs_mass_properties as lump
from auto_battlebot.mujoco_sim.onshape_export import MassBody


def test_fullinertia_order() -> None:
    inertia = np.array([[1.0, 4.0, 5.0], [4.0, 2.0, 6.0], [5.0, 6.0, 3.0]])
    body = lump._body(MassBody(1.0, np.zeros(3), inertia))
    assert body["fullinertia_kg_m2"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
