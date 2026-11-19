"""Static checks on hardware/mr_stabs_mk2_pcb/pcba/design.py, which needs SKiDL to execute."""

import ast
from pathlib import Path

DESIGN = Path(__file__).resolve().parents[2] / "hardware/mr_stabs_mk2_pcb/pcba/design.py"
TAG_POSITION = {"r": 2, "c": 2, "pad": 2}
HELPERS = {"part", "r", "c", "pad"}


def _tag_expressions() -> list[tuple[int, str, ast.expr | None]]:
    tree = ast.parse(DESIGN.read_text())
    helper_bodies = {n for f in tree.body if isinstance(f, ast.FunctionDef) for n in ast.walk(f)}
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node in helper_bodies:
            continue
        name = node.func.id
        if name not in HELPERS | {"Part"}:
            continue
        tag = next((k.value for k in node.keywords if k.arg == "tag"), None)
        if tag is None and name in TAG_POSITION and len(node.args) > TAG_POSITION[name]:
            tag = node.args[TAG_POSITION[name]]
        found.append((node.lineno, name, tag))
    return found


def test_every_part_has_a_tag() -> None:
    calls = _tag_expressions()
    assert len(calls) > 50
    missing = [(line, name) for line, name, tag in calls if tag is None]
    assert missing == []


def test_part_tags_are_unique() -> None:
    tags = [ast.unparse(tag) for _, _, tag in _tag_expressions() if isinstance(tag, ast.Constant)]
    duplicates = sorted({t for t in tags if tags.count(t) > 1})
    assert duplicates == []
