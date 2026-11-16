"""write_plant_configs number formatting."""

from __future__ import annotations

import pytest

from playground.calibration import write_plant_configs as writer


@pytest.mark.parametrize(
    ("value", "text"),
    [(0.0, "0.0"), (1.0, "1.0"), (0.25, "0.25"), (-3.0, "-3.0"), (0.0572, "0.0572")],
)
def test_format_always_keeps_a_decimal_point(value: float, text: str) -> None:
    assert writer._format(value) == text
