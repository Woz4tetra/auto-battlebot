"""Append a draft entry to the registry for every unclaimed reach point."""

from __future__ import annotations

import re
from pathlib import Path

from auto_battlebot.features import reach
from auto_battlebot.features.registry import FEATURES_FILE, Feature, Registry, stub_text

KIND_LABEL = {
    "command": "dashboard command",
    "status": "dashboard status",
    "type": "factory type",
    "cli": "CLI",
    "channel": "transmitter control",
    "readout": "dashboard readout",
    "profile": "profile",
    "unit": "systemd unit",
    "topic": "Foxglove topic",
}
SOURCE_GLOBS = ("src/**/*.cpp", "src/**/*.hpp")
SKIP_SOURCES = frozenset({reach.PROTOCOL_HEADER, reach.VIZ_TOPICS_HEADER})


def slug(rid: str) -> str:
    kind, _, rest = rid.partition(":")
    words = re.sub(r"[^0-9a-zA-Z]+", "_", rest).strip("_").lower()
    return f"{kind}_{words}" if words else kind


def title(point: reach.ReachPoint) -> str:
    _, _, rest = point.id.partition(":")
    last = rest.rstrip("/").split("/")[-1].split(".")[-1]
    words = re.sub(r"(?<=[a-z])(?=[A-Z])", " ", last).replace("_", " ").strip()
    return f"{words[:1].upper()}{words[1:]} ({KIND_LABEL[point.kind]})"


def first_user(root: Path, needle: str) -> str | None:
    for pattern in SOURCE_GLOBS:
        for path in sorted(root.glob(pattern)):
            rel = str(path.relative_to(root))
            if rel in SKIP_SOURCES:
                continue
            if needle in path.read_text(errors="replace"):
                return rel
    return None


def guess_code(root: Path, point: reach.ReachPoint) -> list[str]:
    if point.kind == "type":
        return [str(Path(point.path).parent) + "/"]
    if point.kind in ("command", "status") and point.detail:
        user = first_user(root, point.detail)
        return [user] if user else []
    if point.kind == "topic" and point.detail:
        user = first_user(root, point.id.partition(":")[2])
        return [user] if user else []
    if point.kind == "channel":
        return []
    return [point.path]


def stub(root: Path, point: reach.ReachPoint, taken: set[str]) -> Feature:
    key = slug(point.id)
    while key in taken:
        key += "_"
    taken.add(key)
    feature = Feature(key=key, name=title(point), code=guess_code(root, point))
    if point.kind == "type":
        feature.variants = {point.id.partition(":")[2]: []}
    else:
        feature.reach = [point.id]
    return feature


def variant_stub_text(feature: Feature) -> str:
    text = stub_text(feature)
    if not feature.variants:
        return text
    lines = text.splitlines()
    variant = next(iter(feature.variants))
    lines.insert(2, f"variants = {{ {variant} = [] }}")
    return "\n".join(lines) + "\n"


def draft(root: Path, registry: Registry, unclaimed: list[reach.ReachPoint]) -> int:
    """Append stubs for the unclaimed reach points; returns how many it wrote."""
    if not unclaimed:
        return 0
    taken = set(registry.features)
    blocks = [variant_stub_text(stub(root, point, taken)) for point in unclaimed]
    path = root / FEATURES_FILE
    existing = path.read_text() if path.exists() else "enforce = false\n"
    separator = "" if existing.endswith("\n\n") else "\n"
    path.write_text(existing + separator + "\n".join(blocks))
    return len(blocks)
