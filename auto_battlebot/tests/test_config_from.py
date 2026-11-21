"""config_profile expands ``from`` section files the way src/config/config.cpp does."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from auto_battlebot.config_profile import load_profile_toml


def _write(root: Path, rel: str, text: str) -> Path:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def test_loads_section_with_subtables(tmp_path: Path) -> None:
    _write(
        tmp_path,
        "robots/robot.toml",
        "[plant]\nk_fwd = 5.0\n\n[plant.process_noise]\nq_along = 1.0\n\n"
        '[navigation]\ntype = "X"\n',
    )
    child = _write(tmp_path, "child.toml", '[plant]\nfrom = "robots/robot"\n')

    merged = load_profile_toml(str(child), tmp_path)

    assert merged["plant"] == {"k_fwd": 5.0, "process_noise": {"q_along": 1.0}}
    assert "navigation" not in merged


def test_replaces_base_section_wholesale(tmp_path: Path) -> None:
    _write(tmp_path, "robots/robot.toml", "[plant]\nk_fwd = 5.0\n")
    _write(
        tmp_path,
        "base.toml",
        "[plant]\nk_fwd = 1.0\nc_sb = 2.0\n\n[plant.process_noise]\nq_along = 3.0\n",
    )
    child = _write(
        tmp_path, "child.toml", 'extends = "base"\n\n[plant]\nfrom = "robots/robot.toml"\n'
    )

    assert load_profile_toml(str(child), tmp_path)["plant"] == {"k_fwd": 5.0}


def test_replaces_nested_section_only(tmp_path: Path) -> None:
    _write(
        tmp_path,
        "robots/robot.toml",
        '[robot_filter.motion_estimator]\ntype = "K"\nmax_coast_s = 0.2\n',
    )
    _write(
        tmp_path,
        "base.toml",
        '[robot_filter]\ntype = "F"\n\n[robot_filter.motion_estimator]\ntype = "K"\n'
        "opponent_max_coast_s = 0.1\n",
    )
    child = _write(
        tmp_path,
        "child.toml",
        'extends = "base"\n\n[robot_filter.motion_estimator]\nfrom = "robots/robot"\n',
    )

    robot_filter = load_profile_toml(str(child), tmp_path)["robot_filter"]

    assert robot_filter == {"type": "F", "motion_estimator": {"type": "K", "max_coast_s": 0.2}}


def test_inline_overlay_merges_onto_loaded_section(tmp_path: Path) -> None:
    _write(tmp_path, "robots/robot.toml", "[plant]\nk_fwd = 5.0\nk_ang = 30.0\n")
    _write(tmp_path, "base.toml", '[plant]\nfrom = "robots/robot"\n')
    child = _write(tmp_path, "child.toml", 'extends = "base"\n\n[plant]\nk_fwd = 9.0\n')

    assert load_profile_toml(str(child), tmp_path)["plant"] == {"k_fwd": 9.0, "k_ang": 30.0}


@pytest.mark.parametrize(
    ("files", "child", "message"),
    [
        (
            {"robots/robot.toml": "[plant]\nk_fwd = 5.0\n"},
            '[plant]\nfrom = "robots/robot"\nk_ang = 30.0\n',
            "only key",
        ),
        (
            {"robots/robot.toml": "[plant]\nk_fwd = 5.0\n"},
            '[plant]\nfrom = "robots/robot"\n\n[plant.process_noise]\nq_along = 1.0\n',
            "only key",
        ),
        ({}, '[plant]\nfrom = "robots/absent"\n', "no file"),
        (
            {"robots/robot.toml": '[navigation]\ntype = "N"\n'},
            '[plant]\nfrom = "robots/robot"\n',
            "has no [plant]",
        ),
        (
            {"base.toml": "", "robots/robot.toml": 'extends = "base"\n\n[plant]\nk_fwd = 5.0\n'},
            '[plant]\nfrom = "robots/robot"\n',
            "`extends`",
        ),
        (
            {
                "other.toml": "[plant]\nk_fwd = 5.0\n",
                "robots/robot.toml": '[plant]\nfrom = "other"\n',
            },
            '[plant]\nfrom = "robots/robot"\n',
            "uses `from`",
        ),
        ({}, 'from = "robots/robot"\n', "only valid inside a table"),
        ({}, "[plant]\nfrom = 3\n", "must be a string"),
    ],
)
def test_rejects(tmp_path: Path, files: dict[str, str], child: str, message: str) -> None:
    for rel, text in files.items():
        _write(tmp_path, rel, text)
    path = _write(tmp_path, "child.toml", child)

    with pytest.raises(ValueError, match=re.escape(message)):
        load_profile_toml(str(path), tmp_path)
