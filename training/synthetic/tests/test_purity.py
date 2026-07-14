"""Guard: every pure synthgen module must import without Blender.

If someone adds a ``bpy``/``blenderproc``/``mathutils`` import to one of these
modules, this test fails immediately in the plain venv instead of breaking
only inside the Docker/Blender environment.
"""

import importlib

import pytest

PURE_MODULES = [
    "synthgen",
    "synthgen.constants",
    "synthgen.configuration",
    "synthgen.logsetup",
    "synthgen.reporting",
    "synthgen.gating",
    "synthgen.annotations",
    "synthgen.imaging",
    "synthgen.geometry",
    "synthgen.colorspec",
    "synthgen.asset_index",
]


@pytest.mark.parametrize("module_name", PURE_MODULES)
def test_pure_module_imports_without_blender(module_name: str) -> None:
    importlib.import_module(module_name)
