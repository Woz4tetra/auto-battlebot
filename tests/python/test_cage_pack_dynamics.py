"""Time-step guard in hardware/cage_pack/dynamics.py."""

import importlib
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

CAGE_PACK = Path(__file__).resolve().parents[2] / "hardware" / "cage_pack"


@pytest.fixture()
def dynamics(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    monkeypatch.syspath_prepend(str(CAGE_PACK))
    return importlib.import_module("dynamics")


def test_default_time_step_is_accepted(dynamics: ModuleType) -> None:
    dynamics.check_stable_step(1e-5, dynamics.omegas())


def test_time_step_too_coarse_for_top_mode_raises(dynamics: ModuleType) -> None:
    w = np.array([1000.0, 20000.0])
    with pytest.raises(ValueError, match="too coarse"):
        dynamics.check_stable_step(1e-4, w)


def test_simulate_rejects_time_step_too_coarse(dynamics: ModuleType) -> None:
    case = dynamics.Case(
        name="coarse",
        momentum_ns=1.0,
        tau_s=0.004,
        hit_xy_m=(0.1, 0.1),
        pack_x_m=0.0,
        k_cup_n_per_m=1e5,
    )
    with pytest.raises(ValueError, match="too coarse"):
        dynamics.simulate(case, dynamics.mass_budget(), dt=1e-3)
