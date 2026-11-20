"""convert_one reports a torn or unreadable input as a failed status instead of raising."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "convert_ros1_mcap.py"
SPEC = importlib.util.spec_from_file_location("convert_ros1_mcap", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
convert = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = convert
SPEC.loader.exec_module(convert)

MCAP_MAGIC = b"\x89MCAP0\r\n"


def test_torn_header_fails_the_file_without_raising(tmp_path: Path) -> None:
    src = tmp_path / "torn.mcap"
    src.write_bytes(MCAP_MAGIC + b"\x01\xff")
    status, detail = convert.convert_one(src, tmp_path / "out.mcap", check=False, overwrite=False)
    assert status == "failed"
    assert detail.startswith("unreadable header")


def test_non_mcap_bytes_fail_the_file_without_raising(tmp_path: Path) -> None:
    src = tmp_path / "junk.mcap"
    src.write_bytes(b"not an mcap file at all")
    status, _ = convert.convert_one(src, tmp_path / "out.mcap", check=False, overwrite=False)
    assert status == "failed"


def test_empty_file_is_skipped(tmp_path: Path) -> None:
    src = tmp_path / "empty.mcap"
    src.write_bytes(b"")
    status, _ = convert.convert_one(src, tmp_path / "out.mcap", check=False, overwrite=False)
    assert status == "skipped"
