"""plot_jig_runs reconstructs the radio channels by inverting the tank un-mix."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from auto_battlebot.calibration.jig.drive_protocol import MixConfig
from auto_battlebot.calibration.jig.velocity_jig import JigLog
from playground.calibration import plot_jig_runs


def test_unmix_inverts_from_channels() -> None:
    mix = MixConfig(mode="tank", reverse_angular=True, read_invert_a=False, read_invert_b=False)
    a = np.array([0.5, -0.25, 0.0, 1.0])
    b = np.array([0.5, 0.25, 0.75, -1.0])
    body = [mix.from_channels(float(x), float(y)) for x, y in zip(a, b)]
    record = SimpleNamespace(
        command_source="measured",
        cmd_lin=np.array([lin for lin, _ in body]),
        cmd_ang=np.array([ang for _, ang in body]),
    )
    got_a, got_b = plot_jig_runs._radio_channels(record)
    np.testing.assert_allclose(got_a, a)
    np.testing.assert_allclose(got_b, b)


def test_saturation_matches_jiglog() -> None:
    rng = np.random.default_rng(0)
    n = 200
    log = SimpleNamespace(
        t=np.arange(n) * 0.001,
        gyro_raw=rng.integers(-40000, 40000, size=(n, 3)),
        accel_raw=rng.integers(-9000, 9000, size=(n, 3)),
        header=SimpleNamespace(accel_raw_limit=6000.0),
    )
    report = JigLog.saturation(log)
    for prefix in ("g", "a"):
        mask, axes = plot_jig_runs.saturated_mask(log, sensors=prefix)
        counted = {r.name: r.count for r in report if r.name[0] == prefix}
        assert sorted(axes) == sorted(name for name, count in counted.items() if count)
        assert int(mask.sum()) <= sum(counted.values())
        assert int(mask.sum()) >= max(counted.values())
