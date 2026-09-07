"""The JSON schema texts the Python writer emits must be the bytes the C++ writer emits."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from auto_battlebot import mcap_write

HEADER = Path(__file__).resolve().parents[2] / "include" / "foxglove_adapters" / "json_schemas.hpp"


def _cpp_constant(name: str) -> str:
    text = HEADER.read_text(encoding="utf-8")
    match = re.search(r"constexpr const char \*" + name + r' =\s*R"\((.*?)\)";', text, re.S)
    assert match is not None, f"{name} not found in {HEADER}"
    return match.group(1)


@pytest.mark.parametrize(
    ("python_name", "cpp_name"),
    [
        ("FRAME_META_JSON_SCHEMA", "kFrameMetaSchema"),
        ("DETECTIONS_JSON_SCHEMA", "kDetectionsSchema"),
        ("DIAGNOSTICS_JSON_SCHEMA", "kDiagnosticsSchema"),
    ],
)
def test_schema_text_matches_header(python_name: str, cpp_name: str) -> None:
    assert getattr(mcap_write, python_name) == _cpp_constant(cpp_name)


@pytest.mark.parametrize(
    ("python_name", "cpp_name"),
    [
        ("FRAME_META_SCHEMA_NAME", "kFrameMetaSchemaName"),
        ("DETECTIONS_SCHEMA_NAME", "kDetectionsSchemaName"),
        ("DIAGNOSTICS_SCHEMA_NAME", "kDiagnosticsSchemaName"),
    ],
)
def test_schema_name_matches_header(python_name: str, cpp_name: str) -> None:
    text = HEADER.read_text(encoding="utf-8")
    match = re.search(r"constexpr const char \*" + cpp_name + r' =\s*"([^"]*)";', text)
    assert match is not None, cpp_name
    assert getattr(mcap_write, python_name) == match.group(1)
