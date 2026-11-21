from __future__ import annotations

import importlib.util
import io
import select
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from auto_battlebot.paths import REPO_ROOT

SCRIPT = REPO_ROOT / "firmware" / "velocity_jig" / "scripts" / "console.py"


@pytest.fixture(scope="module")
def console() -> ModuleType:
    spec = importlib.util.spec_from_file_location("jig_console_under_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeSerial:
    def __init__(self, incoming: bytes) -> None:
        self.incoming = bytearray(incoming)
        self.writes: list[bytes] = []

    def read(self, size: int = 1) -> bytes:
        chunk = bytes(self.incoming[:size])
        del self.incoming[:size]
        return chunk

    def write(self, data: bytes) -> None:
        self.writes.append(data)
        if data == b"\n":
            self.incoming += b"END\n"

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass

    def fileno(self) -> int:
        return 99


def test_download_consumes_blank_and_end_trailer(console: ModuleType, tmp_path: Path) -> None:
    ser = FakeSerial(b"SIZE 4\nABCD\nEND\nNEXT\n")
    console.do_get(ser, "a.bin", str(tmp_path))
    assert (tmp_path / "a.bin").read_bytes() == b"ABCD"
    assert console.read_line(ser, 0.2) == "NEXT"


def test_newline_stops_live_stream(console: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    ser = FakeSerial(b"OK\n")
    monkeypatch.setattr(sys, "stdin", io.StringIO("\n"))
    ready: list[Any] = [[sys.stdin]]
    monkeypatch.setattr(select, "select", lambda *_: (ready.pop() if ready else [], [], []))
    console.do_stream(ser)
    assert ser.writes[-1] == b"\n"
