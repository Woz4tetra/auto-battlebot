"""Load and write `docs/features.toml`, the hand-kept feature registry."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from auto_battlebot.compat import tomllib

FEATURES_FILE = "docs/features.toml"
STATUSES = ("draft", "reviewed")
LIST_FIELDS = ("reach", "code", "proofs", "supporting", "links", "docs")
FEATURE_FIELDS = frozenset({"name", "variants", "status", *LIST_FIELDS})
TOP_LEVEL = frozenset({"enforce", "features", "ignored"})


@dataclass
class Feature:
    """One user-facing behavior with its reach points, code, and proofs."""

    key: str
    name: str
    reach: list[str] = field(default_factory=list)
    variants: dict[str, list[str]] = field(default_factory=dict)
    code: list[str] = field(default_factory=list)
    proofs: list[str] = field(default_factory=list)
    supporting: list[str] = field(default_factory=list)
    links: list[str] = field(default_factory=list)
    docs: list[str] = field(default_factory=list)
    status: str = "draft"

    def claims(self) -> list[str]:
        """Reach ids this feature claims, counting each variant as `type:<name>`."""
        return self.reach + [f"type:{name}" for name in self.variants]

    def all_proofs(self) -> list[str]:
        ids = list(self.proofs)
        for variant_proofs in self.variants.values():
            ids += [p for p in variant_proofs if p not in ids]
        return ids


@dataclass
class Registry:
    """Every feature, the deliberately ignored reach points, and whether lint enforces."""

    features: dict[str, Feature]
    ignored: dict[str, str]
    enforce: bool
    errors: list[str]


def string_list(value: Any, where: str, errors: list[str]) -> list[str]:
    if not isinstance(value, list) or not all(isinstance(v, str) for v in value):
        errors.append(f"{where} must be a list of strings")
        return []
    return list(value)


def parse_feature(key: str, raw: Any, errors: list[str]) -> Feature | None:
    where = f"features.{key}"
    if not isinstance(raw, dict):
        errors.append(f"{where} must be a table")
        return None
    unknown = sorted(set(raw) - FEATURE_FIELDS)
    if unknown:
        errors.append(f"{where} has unknown fields {unknown}")
    name = raw.get("name")
    if not isinstance(name, str) or not name:
        errors.append(f"{where}.name must be a non-empty string")
        name = key
    status = raw.get("status", "draft")
    if status not in STATUSES:
        errors.append(f"{where}.status must be one of {list(STATUSES)}, got {status!r}")
    variants: dict[str, list[str]] = {}
    raw_variants = raw.get("variants", {})
    if not isinstance(raw_variants, dict):
        errors.append(f"{where}.variants must be a table of type name to proof ids")
    else:
        for variant, proofs in raw_variants.items():
            variants[variant] = string_list(proofs, f"{where}.variants.{variant}", errors)
    lists = {
        name_: string_list(raw.get(name_, []), f"{where}.{name_}", errors) for name_ in LIST_FIELDS
    }
    return Feature(key=key, name=name, variants=variants, status=status, **lists)


def parse(text: str) -> Registry:
    """A registry from TOML text; problems with its shape go in `errors`."""
    errors: list[str] = []
    try:
        data = tomllib.loads(text)
    except tomllib.TOMLDecodeError as error:
        return Registry({}, {}, False, [f"{FEATURES_FILE} does not parse: {error}"])
    unknown = sorted(set(data) - TOP_LEVEL)
    if unknown:
        errors.append(f"{FEATURES_FILE} has unknown top-level keys {unknown}")
    enforce = data.get("enforce", False)
    if not isinstance(enforce, bool):
        errors.append("enforce must be true or false")
        enforce = False
    features = {}
    raw_features = data.get("features", {})
    if not isinstance(raw_features, dict):
        errors.append("features must be a table")
        raw_features = {}
    for key, raw in raw_features.items():
        feature = parse_feature(key, raw, errors)
        if feature is not None:
            features[key] = feature
    ignored = data.get("ignored", {})
    if not isinstance(ignored, dict) or not all(isinstance(v, str) and v for v in ignored.values()):
        errors.append("ignored must map reach ids to a non-empty reason")
        ignored = {}
    return Registry(features, dict(ignored), enforce, errors)


def load(root: Path) -> Registry:
    path = root / FEATURES_FILE
    if not path.exists():
        return Registry({}, {}, False, [])
    return parse(path.read_text())


def toml_string(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def toml_list(values: list[str]) -> str:
    return "[" + ", ".join(toml_string(v) for v in values) + "]"


def stub_text(feature: Feature) -> str:
    """The TOML block `draft` appends for one feature."""
    lines = [
        f"[features.{feature.key}]",
        f"name = {toml_string(feature.name)}",
        f"reach = {toml_list(feature.reach)}",
        f"code = {toml_list(feature.code)}",
        "proofs = []",
        f'status = "{feature.status}"',
    ]
    return "\n".join(lines) + "\n"
