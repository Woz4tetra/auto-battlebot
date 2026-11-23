"""Which first-party source files a gcov-instrumented run executed, per function."""

from __future__ import annotations

import json
import re
import shutil
import struct
import subprocess
from pathlib import Path

GCDA_MAGIC = 0x67636461
FUNCTION_TAG = 0x01000000
ARC_COUNTER_TAG = 0x01A10000
HEADER_WORDS = 4
WORD = 4
GCOV_DUMP_NAMES = ("gcov-dump-13", "gcov-dump")
GCNO_FUNCTION = re.compile(r"FUNCTION ident=(\d+),.*?' (\S+?):\d+:\d+-\d+:\d+\s*$", re.M)
EXCLUDED_PARTS = ("third_party/", "build")

FunctionBlocks = dict[int, bytes]


def function_blocks(data: bytes) -> FunctionBlocks:
    """Arc counters per function ident; an all-zero block reads as empty bytes."""
    if len(data) < HEADER_WORDS * WORD:
        return {}
    (magic,) = struct.unpack_from("<I", data, 0)
    if magic != GCDA_MAGIC:
        raise ValueError("not a little-endian gcda file")
    blocks: FunctionBlocks = {}
    ident: int | None = None
    offset = HEADER_WORDS * WORD
    end = len(data)
    while offset + 2 * WORD <= end:
        tag, length = struct.unpack_from("<Ii", data, offset)
        offset += 2 * WORD
        if tag == FUNCTION_TAG:
            ident = struct.unpack_from("<I", data, offset)[0] if length > 0 else None
        elif tag == ARC_COUNTER_TAG and ident is not None:
            blocks[ident] = b"" if length < 0 else bytes(data[offset : offset + length])
        if length > 0:
            offset += length
    return blocks


def gcov_dump() -> str | None:
    for name in GCOV_DUMP_NAMES:
        path = shutil.which(name)
        if path:
            return path
    return None


def first_party(path: str, root: Path) -> str | None:
    """Repo-relative path of a source the repo owns, or None for system and vendored files."""
    prefix = str(root.resolve()) + "/"
    if not path.startswith(prefix):
        return None
    rel = path[len(prefix) :]
    if rel.startswith(EXCLUDED_PARTS):
        return None
    return rel


class FunctionIndex:
    """Function ident to source file for each object, read from its gcno once and cached."""

    def __init__(self, root: Path, cache_file: Path) -> None:
        self.root = root
        self.cache_file = cache_file
        self.tool = gcov_dump()
        self.entries: dict[str, dict[str, object]] = {}
        if cache_file.exists():
            self.entries = json.loads(cache_file.read_text())
        self.dirty = False

    def functions(self, gcno: Path) -> dict[int, str]:
        key = str(gcno)
        if not gcno.exists():
            return {}
        mtime = gcno.stat().st_mtime
        cached = self.entries.get(key)
        if cached is not None and cached.get("mtime") == mtime:
            files = cached["functions"]
            assert isinstance(files, dict)
            return {int(k): v for k, v in files.items()}
        if self.tool is None:
            raise RuntimeError("gcov-dump is not installed")
        text = subprocess.run(
            [self.tool, "-l", str(gcno)], capture_output=True, text=True, check=False
        ).stdout
        functions: dict[int, str] = {}
        for match in GCNO_FUNCTION.finditer(text):
            rel = first_party(match.group(2), self.root)
            if rel is not None:
                functions[int(match.group(1))] = rel
        self.entries[key] = {"mtime": mtime, "functions": functions}
        self.dirty = True
        return functions

    def save(self) -> None:
        if self.dirty:
            self.cache_file.parent.mkdir(parents=True, exist_ok=True)
            self.cache_file.write_text(json.dumps(self.entries))
            self.dirty = False


def read_blocks(path: Path) -> FunctionBlocks:
    try:
        return function_blocks(path.read_bytes())
    except (OSError, ValueError):
        return {}


def object_path(gcda: Path, prefix: Path) -> Path:
    """The gcda's original location, which is where its gcno sits."""
    return Path("/") / gcda.relative_to(prefix)


def snapshot(prefix: Path) -> dict[Path, FunctionBlocks]:
    """Every gcda a run wrote under GCOV_PREFIX=`prefix`, keyed by original object path."""
    return {object_path(g, prefix): read_blocks(g) for g in prefix.rglob("*.gcda")}


def executed_files(
    run: dict[Path, FunctionBlocks],
    baseline: dict[Path, FunctionBlocks],
    index: FunctionIndex,
) -> set[str]:
    """First-party files holding a function whose counters moved past the baseline run."""
    found: set[str] = set()
    for gcda, blocks in run.items():
        before = baseline.get(gcda, {})
        moved = [ident for ident, counts in blocks.items() if counts != before.get(ident, b"")]
        if not moved:
            continue
        functions = index.functions(gcda.with_suffix(".gcno"))
        found |= {functions[ident] for ident in moved if ident in functions}
    return found
