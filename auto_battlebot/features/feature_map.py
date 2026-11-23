"""Join the registry, reach discovery, run store, coverage, config schema, and git into one map."""

from __future__ import annotations

import json
import subprocess
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

from auto_battlebot.features import check, proofs, reach, registry, staleness
from auto_battlebot.verify import run_store
from auto_battlebot.verify.state import changed_since

SCHEMA_DUMP = Path("build/config_schema_dump")
STAGES = (
    ("camera", "Camera", ("src/rgbd_camera/", "include/rgbd_camera/")),
    (
        "perception",
        "Perception",
        (
            "src/mask_model/",
            "src/field_filter/",
            "src/keypoint_model/",
            "src/keypoint_filter/",
            "include/mask_model/",
            "include/field_filter/",
            "include/keypoint_model/",
            "include/keypoint_filter/",
        ),
    ),
    ("filter", "Filter", ("src/robot_filter/", "include/robot_filter/")),
    ("target", "Target", ("src/target_selector/", "include/target_selector/")),
    (
        "nav",
        "Navigation",
        ("src/navigation/", "include/navigation/", "src/rl_policy/", "src/command_shaper/"),
    ),
    ("transmit", "Transmit", ("src/transmitter/", "include/transmitter/")),
    ("robot", "Robot", ("firmware/",)),
)
PLATFORM_STAGE = ("platform", "Platform")
COMMITS_SHOWN = 5
INFRASTRUCTURE_SHARE = 0.5
WIDE_PROOF_SHARE = 1 / 3
STATE_ORDER = ("failed", "stale", "unrun", "passing")


def stage_votes(feature: registry.Feature, points: dict[str, reach.ReachPoint]) -> dict[str, int]:
    paths = list(feature.code) + [points[r].path for r in feature.reach if r in points]
    votes: dict[str, int] = {}
    for path in paths:
        for key, _, prefixes in STAGES:
            if any(path.startswith(prefix) for prefix in prefixes):
                votes[key] = votes.get(key, 0) + 1
    if any(r.startswith("channel:") for r in feature.reach):
        votes["robot"] = votes.get("robot", 0) + 1
    return votes


def stage_of(feature: registry.Feature, points: dict[str, reach.ReachPoint]) -> str:
    votes = stage_votes(feature, points)
    if not votes:
        return PLATFORM_STAGE[0]
    return max(votes, key=lambda k: (votes[k], -[s[0] for s in STAGES].index(k)))


def stages_of(feature: registry.Feature, points: dict[str, reach.ReachPoint]) -> list[str]:
    """Every pipeline stage the feature's code and reach paths fall in, in pipeline order."""
    votes = stage_votes(feature, points)
    return [key for key, _, _ in STAGES if key in votes]


def config_schema(root: Path) -> dict[str, Any]:
    binary = root / SCHEMA_DUMP
    if not binary.exists():
        return {}
    result = subprocess.run(
        [str(binary), "--config-dir", str(root / "config")],
        capture_output=True,
        text=True,
        check=False,
        timeout=60,
    )
    if result.returncode != 0:
        return {}
    data: dict[str, Any] = json.loads(result.stdout)
    return data


def variant_config(schema: dict[str, Any], variant: str) -> dict[str, Any]:
    """The variant's section, its fields and defaults, and the profiles that select it."""
    for section, entry in schema.get("sections", {}).items():
        probed = entry.get("types", {}).get(variant)
        if probed is None:
            continue
        selecting = sorted(
            name
            for name, profile in schema.get("profiles", {}).items()
            if profile.get("types", {}).get(section) == variant
        )
        return {
            "section": section,
            "fields": probed.get("fields", []),
            "probe_error": probed.get("error"),
            "profiles": selecting,
        }
    return {}


def git_lines(root: Path, *args: str) -> list[str]:
    result = subprocess.run(["git", *args], cwd=root, capture_output=True, text=True, check=False)
    return [line for line in result.stdout.splitlines() if line] if result.returncode == 0 else []


def paths_for_git(root: Path, paths: list[str]) -> list[str]:
    return [p for p in dict.fromkeys(paths) if (root / p).exists()]


def recent_commits(root: Path, paths: list[str]) -> list[dict[str, str]]:
    if not paths:
        return []
    lines = git_lines(
        root, "log", f"-n{COMMITS_SHOWN}", "--format=%h%x09%ad%x09%s", "--date=short", "--", *paths
    )
    commits = []
    for line in lines:
        sha, date, subject = (line.split("\t", 2) + ["", ""])[:3]
        commits.append({"sha": sha, "date": date, "subject": subject})
    return commits


def churn_since(root: Path, commit: str | None, paths: list[str]) -> int | None:
    if not commit or not paths:
        return None
    lines = git_lines(root, "rev-list", "--count", f"{commit}..HEAD", "--", *paths)
    return int(lines[0]) if lines else None


def feature_state(entries: list[dict[str, Any]]) -> str:
    if not entries:
        return "gap"
    states = {e["state"] for e in entries}
    for state in STATE_ORDER:
        if state in states:
            return state
    return "passing"


def proof_entry(
    proof: str, kind: str, states: dict[str, staleness.ProofState], scope: list[str]
) -> dict[str, Any]:
    parsed = proofs.parse(proof)
    state = states.get(proof, staleness.ProofState(proof))
    return {
        **asdict(state),
        "kind": kind,
        "runner": parsed.runner if parsed else "unknown",
        "command": f"scripts/verify --only {proof}",
        "modules": sorted({module_of(path) for path in scope if not is_test_path(path)}),
    }


def link_grades(links: list[str], states: dict[str, staleness.ProofState]) -> list[dict[str, Any]]:
    out = []
    for link in links:
        state = states.get(f"mcap:{link}_link")
        out.append(
            {
                "name": link,
                "grade": state.last_status if state else None,
                "stamp": state.last_stamp if state else None,
            }
        )
    return out


def proof_scopes(
    features: dict[str, registry.Feature], coverage: dict[str, list[str]]
) -> dict[str, list[str]]:
    """Files a proof depends on: its coverage `touches`, else the code of features listing it."""
    scopes: dict[str, list[str]] = {}
    for feature in features.values():
        for proof in feature.all_proofs() + feature.supporting:
            scopes.setdefault(proof, [])
            for path in coverage.get(proof) or feature.code:
                if path not in scopes[proof]:
                    scopes[proof].append(path)
    return scopes


def build_features(
    root: Path,
    reg: registry.Registry,
    points: dict[str, reach.ReachPoint],
    context: dict[str, Any],
) -> list[dict[str, Any]]:
    states = context["states"]
    coverage = context["coverage"]
    schema = context["schema"]
    with_git = context["with_git"]
    diff_files = context["diff_files"]
    out = []
    for key, feature in reg.features.items():

        def entry(proof: str, kind: str, code: list[str] = feature.code) -> dict[str, Any]:
            return proof_entry(proof, kind, states, coverage.get(proof) or code)

        entries = [entry(p, "entry") for p in feature.proofs]
        variants: list[dict[str, Any]] = []
        for variant, ids in feature.variants.items():
            variant_entries = [entry(p, "variant") for p in ids]
            variants.append(
                {
                    "type": variant,
                    "proofs": variant_entries,
                    "state": feature_state(variant_entries),
                    "config": variant_config(schema, variant),
                }
            )
        supporting = [entry(p, "supporting") for p in feature.supporting]
        touches = sorted({path for p in feature.all_proofs() for path in coverage.get(p, [])})
        scope = list(feature.code) + touches
        git_paths = paths_for_git(root, scope) if with_git else []
        pass_commits = [e["last_pass_commit"] for e in entries if e["last_pass_commit"]]
        state = feature_state(entries + [e for v in variants for e in v["proofs"]])
        out.append(
            {
                "key": key,
                "name": feature.name,
                "status": feature.status,
                "stage": stage_of(feature, points),
                "stages": stages_of(feature, points),
                "reach": feature.reach,
                "variants": variants,
                "code": feature.code,
                "touches": touches,
                "proofs": entries,
                "supporting": supporting,
                "links": link_grades(feature.links, states),
                "docs": feature.docs,
                "state": state,
                "commits": recent_commits(root, git_paths) if with_git else [],
                "churn": churn_since(root, min(pass_commits) if pass_commits else None, git_paths)
                if with_git
                else None,
                "in_diff": any(staleness.matches(f, scope) for f in diff_files),
            }
        )
    return out


def is_test_path(path: str) -> bool:
    parts = Path(path).parts
    return bool(parts) and (parts[0] == "tests" or "tests" in parts[1:-1])


def module_of(path: str) -> str:
    parts = Path(path).parts
    if parts and parts[0] in ("src", "include", "auto_battlebot", "firmware", "web"):
        top = "src" if parts[0] == "include" else parts[0]
        if len(parts) >= 3 or (len(parts) == 2 and path.endswith("/")):
            return f"{top}/{parts[1]}/"
        return f"{top}/{Path(parts[-1]).stem}"
    return "/".join(parts[:1]) + ("/" if len(parts) > 1 else "")


def proof_entries(feature: dict[str, Any]) -> list[dict[str, Any]]:
    variants = [e for v in feature["variants"] for e in v["proofs"]]
    return [*feature["proofs"], *feature["supporting"], *variants]


def collapse_wide_proofs(features: list[dict[str, Any]]) -> None:
    """Replace a proof's modules in a root with `<root>/*` when it covers most of that root."""
    entries = [e for f in features for e in proof_entries(f)]
    per_root: dict[str, set[str]] = {}
    for module in {m for e in entries for m in e["modules"]}:
        per_root.setdefault(module.split("/")[0], set()).add(module)
    for entry in entries:
        modules = set(entry["modules"])
        for root, every in per_root.items():
            mine = modules & every
            if len(every) > 2 and len(mine) > WIDE_PROOF_SHARE * len(every):
                modules = (modules - mine) | {f"{root}/*"}
        entry["modules"] = sorted(modules)


def feature_modules(feature: dict[str, Any]) -> set[str]:
    code = {module_of(p) for p in feature["code"]}
    return code | {m for e in proof_entries(feature) for m in e["modules"]}


def module_index(features: list[dict[str, Any]]) -> dict[str, Any]:
    """Code modules per feature, with ones over half their root's features collapsed."""
    modules: dict[str, set[str]] = {}
    for feature in features:
        for module in feature_modules(feature):
            modules.setdefault(module, set()).add(feature["key"])
    roots: dict[str, set[str]] = {}
    for module, keys in modules.items():
        roots.setdefault(module.split("/")[0], set()).update(keys)
    infrastructure = sorted(
        m
        for m, keys in modules.items()
        if len(keys) > 1 and len(keys) > INFRASTRUCTURE_SHARE * len(roots[m.split("/")[0]])
    )
    return {
        "modules": {m: sorted(keys) for m, keys in sorted(modules.items())},
        "infrastructure": infrastructure,
    }


def overview(features: list[dict[str, Any]]) -> list[dict[str, Any]]:
    stages = []
    for key, label in [(s[0], s[1]) for s in STAGES] + [PLATFORM_STAGE]:
        members = [f for f in features if f["stage"] == key]
        stages.append(
            {
                "id": key,
                "label": label,
                "proven": sum(f["state"] == "passing" for f in members),
                "stale": sum(f["state"] in ("stale", "unrun") for f in members),
                "failed": sum(f["state"] == "failed" for f in members),
                "gap": sum(f["state"] == "gap" for f in members),
                "features": [f["key"] for f in members],
            }
        )
    return stages


def diff_overlay(
    root: Path, features: list[dict[str, Any]], files: list[str], with_selection: bool
) -> dict[str, Any]:
    from auto_battlebot.verify.cli import describe
    from auto_battlebot.verify.selection import select

    selection = select(files, root) if files and with_selection else None
    return {
        "files": files,
        "features": [f["key"] for f in features if f["in_diff"]],
        "would_run": describe(selection) if selection else [],
        "command": "scripts/verify --changed",
    }


def build_map(root: Path, with_git: bool = True, with_selection: bool = False) -> dict[str, Any]:
    """Everything the feature map UI and lint read, computed from the tree as it is now."""
    reg = registry.load(root)
    points = {p.id: p for p in reach.discover(root)}
    report = check.check(root, reg)
    runs = run_store.load_runs(root)
    coverage = staleness.load_coverage(root)
    head = run_store.head_commit(root) if with_git else ""
    dirty = run_store.dirty_hash(root) if with_git else ""
    changes = staleness.ChangedFiles(root)
    states = staleness.proof_states(
        runs, proof_scopes(reg.features, coverage), changes, (head, dirty)
    )
    diff_files = changed_since(root, "HEAD") if with_git else []
    context = {
        "states": states,
        "coverage": coverage,
        "schema": config_schema(root),
        "with_git": with_git,
        "diff_files": diff_files,
    }
    features = build_features(root, reg, points, context)
    collapse_wide_proofs(features)
    claimed = {rid: key for key, f in reg.features.items() for rid in f.claims()}
    return {
        "generated": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "head": head,
        "dirty": dirty,
        "enforce": reg.enforce,
        "stages": overview(features),
        "features": features,
        "reach": [
            {**asdict(p), "feature": claimed.get(rid), "ignored": reg.ignored.get(rid)}
            for rid, p in points.items()
        ],
        "impact": module_index(features),
        "diff": diff_overlay(root, features, diff_files, with_selection),
        "check": {"errors": report.errors, "gaps": report.gaps},
        "runs": [
            {"stamp": r.stamp, "commit": r.commit, "tier": r.tier, "results": len(r.results)}
            for r in runs[-20:]
        ],
    }
