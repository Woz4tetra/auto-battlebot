"""Discover every reach point a person can trigger or observe, from the source tree."""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from pathlib import Path

PROTOCOL_HEADER = "include/remote/protocol.hpp"
VIZ_TOPICS_HEADER = "include/publisher/viz_topics.hpp"
CHANNEL_CONVENTION = "firmware/config/CHANNEL_CONVENTION.md"
WEB_COMPONENTS = "web/src/components"
SERVICE_DIR = "service"
CONFIG_DIR = "config"
SCRIPTS_DIR = "scripts"
PACKAGE_DIR = "auto_battlebot"

KINDS = ("command", "status", "type", "cli", "channel", "readout", "profile", "unit", "topic")

STATUS_ROW = re.compile(r'StatusTopic<(\w+)>\{"(/status/[\w/]+)"')
COMMAND_ROW = re.compile(r'CommandTopic<(\w+)>\{"(/command/[\w/]+)"\}')
VIZ_ROW = re.compile(
    r'VizTopic\{VizTopicId::(\w+),\s*"([^"]+)",\s*"(\w+)",\s*(true|false),\s*'
    r"TopicRate::(\w+),\s*(true|false)\}"
)
REGISTER_ROW = re.compile(r'REGISTER_CONFIG\(\s*(\w+),\s*(\w+),\s*"(\w+)"\s*\)')
READOUT_MARKERS = ("status.svelte", "subscribe", "diagnostics.svelte")
SCRIPT_SKIP = frozenset({"README.md"})
WRAPPER_DIRS = ("training", "simulation", "logo")
WRAPPER_IMPORT = re.compile(r"^from (auto_battlebot\.[\w.]+) import main\b", re.M)
MAIN_GUARD = re.compile(r"""^if __name__ == ["']__main__["']:""", re.M)


@dataclass(frozen=True)
class ReachPoint:
    """One place a person reaches a feature: a control, readout, CLI, profile, or unit."""

    id: str
    kind: str
    path: str
    line: int
    detail: str = ""


@dataclass(frozen=True)
class VizTopicRow:
    """A row of the Foxglove topic table."""

    id: str
    topic: str
    encoding: str
    latch: bool
    rate: str
    recorded: bool


@dataclass(frozen=True)
class FactoryType:
    """A REGISTER_CONFIG type and the base configuration it registers under."""

    name: str
    base: str
    path: str
    line: int


def line_of(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def remote_topics(root: Path) -> list[ReachPoint]:
    text = (root / PROTOCOL_HEADER).read_text()
    points = [
        ReachPoint(
            f"status:{m.group(2)}", "status", PROTOCOL_HEADER, line_of(text, m.start()), m.group(1)
        )
        for m in STATUS_ROW.finditer(text)
    ]
    points += [
        ReachPoint(
            f"command:{m.group(2)}",
            "command",
            PROTOCOL_HEADER,
            line_of(text, m.start()),
            m.group(1),
        )
        for m in COMMAND_ROW.finditer(text)
    ]
    return points


def viz_topic_rows(root: Path) -> list[VizTopicRow]:
    text = (root / VIZ_TOPICS_HEADER).read_text()
    return [
        VizTopicRow(
            m.group(1),
            m.group(2),
            m.group(3),
            m.group(4) == "true",
            m.group(5),
            m.group(6) == "true",
        )
        for m in VIZ_ROW.finditer(text)
    ]


def viz_topics(root: Path) -> list[ReachPoint]:
    text = (root / VIZ_TOPICS_HEADER).read_text()
    return [
        ReachPoint(
            f"topic:{m.group(2)}", "topic", VIZ_TOPICS_HEADER, line_of(text, m.start()), m.group(5)
        )
        for m in VIZ_ROW.finditer(text)
    ]


def factory_types(root: Path) -> list[FactoryType]:
    found = []
    for path in sorted((root / "src").rglob("*.cpp")):
        text = path.read_text(errors="replace")
        for m in REGISTER_ROW.finditer(text):
            rel = str(path.relative_to(root))
            found.append(FactoryType(m.group(3), m.group(1), rel, line_of(text, m.start())))
    return found


def type_points(root: Path) -> list[ReachPoint]:
    return [
        ReachPoint(f"type:{t.name}", "type", t.path, t.line, t.base) for t in factory_types(root)
    ]


def has_main(path: Path) -> int:
    """Line of a module-level `def main`, or 0."""
    try:
        tree = ast.parse(path.read_text(errors="replace"))
    except SyntaxError:
        return 0
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "main":
            return node.lineno
    return 0


def module_name(root: Path, path: Path) -> str:
    rel = path.relative_to(root).with_suffix("")
    parts = list(rel.parts)
    if parts[-1] in ("__main__", "__init__"):
        parts = parts[:-1]
    return ".".join(parts)


def wrappers(root: Path) -> dict[str, list[str]]:
    """Module name to the scripts that import its `main` and run it."""
    found: dict[str, list[str]] = {}
    for top in WRAPPER_DIRS:
        for path in sorted((root / top).rglob("*.py")):
            if "tests" in path.relative_to(root).parts:
                continue
            for match in WRAPPER_IMPORT.finditer(path.read_text(errors="replace")):
                found.setdefault(match.group(1), []).append(str(path.relative_to(root)))
    return found


def cli_modules(root: Path) -> list[ReachPoint]:
    """Runnable modules; a module whose `main` only a wrapper runs is reached at the wrapper."""
    points = []
    wrapped = wrappers(root)
    for path in sorted((root / PACKAGE_DIR).rglob("*.py")):
        if "tests" in path.relative_to(root).parts:
            continue
        line = has_main(path)
        if not line or path.name == "__init__.py":
            continue
        name = module_name(root, path)
        rel = str(path.relative_to(root))
        if MAIN_GUARD.search(path.read_text(errors="replace")) or name not in wrapped:
            points.append(ReachPoint(f"cli:{name}", "cli", rel, line))
        else:
            points += [ReachPoint(f"cli:{w}", "cli", w, 1, name) for w in wrapped[name]]
    unique = {p.id: p for p in points}
    return list(unique.values())


def script_entry_points(root: Path) -> list[ReachPoint]:
    points = []
    for path in sorted((root / SCRIPTS_DIR).iterdir()):
        if path.is_file() and path.name not in SCRIPT_SKIP:
            rel = str(path.relative_to(root))
            points.append(ReachPoint(f"cli:{rel}", "cli", rel, 1))
    return points


def markdown_table_after(text: str, heading: str) -> list[tuple[int, list[str]]]:
    """Rows of the first pipe table after a line equal to `heading`, with their offsets."""
    start = text.find("\n" + heading + "\n")
    if start < 0:
        return []
    rows: list[tuple[int, list[str]]] = []
    offset = start + len(heading) + 2
    in_table = False
    for raw in text[offset:].split("\n"):
        line = raw.strip()
        if line.startswith("|"):
            in_table = True
            cells = [c.strip() for c in line.strip("|").split("|")]
            if not all(set(c) <= set("-: ") for c in cells):
                rows.append((offset, cells))
        elif in_table:
            break
        offset += len(raw) + 1
    return rows[1:]


def transmitter_controls(root: Path) -> list[ReachPoint]:
    text = (root / CHANNEL_CONVENTION).read_text()
    points = []
    for heading in ("Sticks:", "Switches:"):
        for offset, cells in markdown_table_after(text, heading):
            name = cells[0]
            if name:
                points.append(
                    ReachPoint(
                        f"channel:{name}",
                        "channel",
                        CHANNEL_CONVENTION,
                        line_of(text, offset),
                        cells[2] if len(cells) > 2 else "",
                    )
                )
    return points


def dashboard_readouts(root: Path) -> list[ReachPoint]:
    points = []
    for path in sorted((root / WEB_COMPONENTS).glob("*.svelte")):
        text = path.read_text(errors="replace")
        if any(marker in text for marker in READOUT_MARKERS):
            rel = str(path.relative_to(root))
            points.append(ReachPoint(f"readout:{path.stem}", "readout", rel, 1))
    return points


def profiles(root: Path) -> list[ReachPoint]:
    return [
        ReachPoint(f"profile:{path.stem}", "profile", str(path.relative_to(root)), 1)
        for path in sorted((root / CONFIG_DIR).glob("*.toml"))
        if not path.stem.startswith("_") and path.stem != "profiles"
    ]


def units(root: Path) -> list[ReachPoint]:
    return [
        ReachPoint(f"unit:{path.stem}", "unit", str(path.relative_to(root)), 1)
        for path in sorted((root / SERVICE_DIR).glob("*.service"))
    ]


SOURCES = (
    remote_topics,
    type_points,
    cli_modules,
    script_entry_points,
    transmitter_controls,
    dashboard_readouts,
    profiles,
    units,
    viz_topics,
)


def discover(root: Path) -> list[ReachPoint]:
    """Every reach point in the tree, in source order."""
    points: list[ReachPoint] = []
    for source in SOURCES:
        points += source(root)
    return points
