"""write_plant_configs number formatting."""

from __future__ import annotations

from pathlib import Path

import pytest

from auto_battlebot.calibration import write_plant_configs as writer


@pytest.mark.parametrize(
    ("value", "text"),
    [(0.0, "0.0"), (1.0, "1.0"), (0.25, "0.25"), (-3.0, "-3.0"), (0.0572, "0.0572")],
)
def test_format_always_keeps_a_decimal_point(value: float, text: str) -> None:
    assert writer._format(value) == text


@pytest.mark.parametrize(
    ("path", "fields"),
    [
        (writer.SIM_CONFIG, writer.SIM_FIELDS),
        (writer.PLANT_CONFIG, writer.PLANT_FIELDS),
        (writer.MR_STABS_PLANT_CONFIG, writer.PLANT_FIELDS),
    ],
    ids=["sim", "mrs_buff_mk3", "mr_stabs_mk2"],
)
def test_every_mapped_field_appears_exactly_once_in_its_target_config(
    path: Path, fields: dict[str, str]
) -> None:
    text = path.read_text()
    plant = {fit_name: 1.0 for fit_name in fields.values()}
    _, changes = writer._rewrite(text, writer.TOML_FIELD, fields, plant, path)
    assert len({change.field for change in changes}) == len(changes)


def test_rewrite_refuses_a_field_name_that_appears_in_two_sections() -> None:
    text = "[a]\nk_fwd = 1.0\n[b]\nk_fwd = 2.0\n"

    with pytest.raises(SystemExit, match="appears 2 times"):
        writer._rewrite(
            text, writer.TOML_FIELD, {"k_fwd": "k_fwd"}, {"k_fwd": 3.0}, writer.SIM_CONFIG
        )
