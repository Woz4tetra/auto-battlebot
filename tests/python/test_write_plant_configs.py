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


def test_every_sim_field_appears_exactly_once_in_the_sim_config() -> None:
    text = writer.SIM_CONFIG.read_text()
    plant = {fit_name: 1.0 for fit_name in writer.SIM_FIELDS.values()}

    _, changes = writer._rewrite(
        text, writer.TOML_FIELD, writer.SIM_FIELDS, plant, writer.SIM_CONFIG
    )

    assert len({change.field for change in changes}) == len(changes)


def test_rewrite_refuses_a_field_name_that_appears_in_two_sections() -> None:
    text = "[a]\nk_fwd = 1.0\n[b]\nk_fwd = 2.0\n"

    with pytest.raises(SystemExit, match="appears 2 times"):
        writer._rewrite(
            text, writer.TOML_FIELD, {"k_fwd": "k_fwd"}, {"k_fwd": 3.0}, writer.SIM_CONFIG
        )
