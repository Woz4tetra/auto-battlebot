"""Hazard file loading fails loudly instead of dropping hazards."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "simulation"))

import hazards  # noqa: E402


def test_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        hazards.load_hazards(tmp_path / "typo.toml")


def test_empty_hazard_file_is_an_empty_list(tmp_path: Path) -> None:
    path = tmp_path / "empty.toml"
    path.write_text("")
    assert hazards.load_hazards(path) == []
