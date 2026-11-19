"""Pytest setup for the synthgen tests."""

import sys
from collections.abc import Iterator
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

SYNTHETIC_DIR = Path(__file__).resolve().parents[1]
BLENDER_MODULES = (
    "bpy",
    "bmesh",
    "bpy_extras",
    "bpy_extras.object_utils",
    "mathutils",
    "blenderproc",
)


@pytest.fixture
def blender_stubs(monkeypatch: pytest.MonkeyPatch) -> Iterator[SimpleNamespace]:
    """Stub Blender modules so a Blender-side synthgen module imports and runs its pure logic."""
    stubs = {name: MagicMock(name=name) for name in BLENDER_MODULES}
    stubs["bpy_extras"].object_utils = stubs["bpy_extras.object_utils"]
    for name, stub in stubs.items():
        monkeypatch.setitem(sys.modules, name, stub)
    before = set(sys.modules)
    yield SimpleNamespace(**{name.replace(".", "_"): stub for name, stub in stubs.items()})
    package = sys.modules.get("synthgen")
    for name in [n for n in sys.modules if n not in before]:
        origin = getattr(sys.modules[name], "__file__", None)
        if origin is None or not Path(origin).resolve().is_relative_to(SYNTHETIC_DIR):
            continue
        del sys.modules[name]
        if package is not None and name.startswith("synthgen."):
            package.__dict__.pop(name.removeprefix("synthgen."), None)
