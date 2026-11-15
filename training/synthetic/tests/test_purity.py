"""Guard: every pure synthgen module must import without Blender."""

import ast
import importlib
from pathlib import Path

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
    "synthgen.cage_spec",
    "synthgen.cage_mount",
    "synthgen.damage",
    "synthgen.lens",
    "synthgen.shards",
    "synthgen.freefly",
    "synthgen.preview_server",
]


@pytest.mark.parametrize("module_name", PURE_MODULES)
def test_pure_module_imports_without_blender(module_name: str) -> None:
    importlib.import_module(module_name)


@pytest.mark.parametrize("module_name", PURE_MODULES)
def test_pure_module_source_never_names_blender(module_name: str) -> None:
    path = Path(__file__).resolve().parents[1].joinpath(*module_name.split("."))
    source = path / "__init__.py" if path.is_dir() else path.with_suffix(".py")
    imported = set()
    for node in ast.walk(ast.parse(source.read_text())):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    assert not imported & {"bpy", "blenderproc", "bmesh", "mathutils"}
