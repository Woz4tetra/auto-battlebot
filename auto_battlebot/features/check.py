"""Lint rules for the feature registry against the reach points and proofs in the tree."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

from auto_battlebot.features import proofs as proof_ids
from auto_battlebot.features import reach as reach_points
from auto_battlebot.features.registry import FEATURES_FILE, Feature, Registry


@dataclass
class Report:
    """What lint found: errors always fail, gaps fail once the registry sets `enforce`."""

    errors: list[str] = field(default_factory=list)
    gaps: list[str] = field(default_factory=list)
    unclaimed: list[reach_points.ReachPoint] = field(default_factory=list)
    enforce: bool = False

    def failed(self) -> bool:
        return bool(self.errors) or (self.enforce and bool(self.gaps))

    def lines(self) -> list[str]:
        out = [f"error: {e}" for e in self.errors]
        label = "error" if self.enforce else "gap"
        out += [f"{label}: {g}" for g in self.gaps]
        return out


def feature_errors(
    root: Path,
    key: str,
    feature: Feature,
    context: tuple[set[str], set[str], set[str], proof_ids.Resolver],
) -> list[str]:
    """One feature's listed reach ids, variants, proofs, links, code, and docs that do not exist."""
    points, types, links, resolver = context
    where = f"features.{key}"
    errors = []
    for rid in feature.reach:
        if rid not in points:
            errors.append(f"{where}.reach: {rid} is not a reach point in the tree")
        elif rid.startswith("type:"):
            errors.append(f"{where}.reach: list {rid} under variants instead")
    errors += [
        f"{where}.variants: {v} is not a REGISTER_CONFIG type"
        for v in feature.variants
        if f"type:{v}" not in types
    ]
    errors += [
        f"{where}: {p}" for p in resolver.problems(feature.all_proofs() + feature.supporting)
    ]
    errors += [
        f"{where}.links: {link} is not a MonitorLink ({sorted(links)})"
        for link in feature.links
        if link not in links
    ]
    errors += [f"{where}.code: {c} does not exist" for c in feature.code if not (root / c).exists()]
    errors += [
        f"{where}.docs: {d} does not exist" for d in feature.docs if not (root / d).is_file()
    ]
    return errors


def reference_errors(
    root: Path, registry: Registry, points: dict[str, reach_points.ReachPoint]
) -> list[str]:
    """Registry shape errors, plus every listed reference that does not exist."""
    types = {p.id for p in points.values() if p.kind == "type"}
    context = (set(points), types, proof_ids.link_names(root), proof_ids.Resolver(root))
    errors = list(registry.errors)
    for key, feature in registry.features.items():
        errors += feature_errors(root, key, feature, context)
    errors += [
        f"ignored: {rid} is not a reach point in the tree"
        for rid in registry.ignored
        if rid not in points
    ]
    return errors


def duplicate_claims(registry: Registry) -> list[str]:
    counts = Counter(rid for f in registry.features.values() for rid in f.claims())
    errors = []
    for rid, count in sorted(counts.items()):
        if count > 1:
            owners = [k for k, f in registry.features.items() if rid in f.claims()]
            errors.append(f"{rid} is claimed by {count} features: {owners}")
        if rid in registry.ignored:
            errors.append(f"{rid} is both claimed and ignored")
    return errors


def completeness_gaps(registry: Registry) -> list[str]:
    gaps = []
    for key, feature in registry.features.items():
        if not feature.proofs:
            gaps.append(f"features.{key} ({feature.name}) has no entry-point proof")
        for variant, ids in feature.variants.items():
            if not ids:
                gaps.append(f"features.{key}.variants.{variant} has no proof")
        if feature.status == "draft":
            gaps.append(f"features.{key} ({feature.name}) is still a draft")
    return gaps


def unclaimed_points(
    registry: Registry, points: dict[str, reach_points.ReachPoint]
) -> list[reach_points.ReachPoint]:
    claimed = {rid for f in registry.features.values() for rid in f.claims()}
    return [p for rid, p in points.items() if rid not in claimed and rid not in registry.ignored]


def check(root: Path, registry: Registry) -> Report:
    points = {p.id: p for p in reach_points.discover(root)}
    unclaimed = unclaimed_points(registry, points)
    gaps = [
        f"{p.id} ({p.path}:{p.line}) is not claimed by any feature in {FEATURES_FILE}"
        for p in unclaimed
    ]
    gaps += completeness_gaps(registry)
    errors = reference_errors(root, registry, points) + duplicate_claims(registry)
    return Report(errors, gaps, unclaimed, registry.enforce)
