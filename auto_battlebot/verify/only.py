"""Turn a list of proof ids into the suites, test filters, and playback clips that run them."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from auto_battlebot.features import proofs
from auto_battlebot.verify.selection import Selection

NOT_RUNNABLE = {
    "hil": "hil: proofs run in a guided session (scripts/verify --hil, phase 6)",
    "sim": "sim: proofs need the nav plan 3.5 smoke tier (scripts/verify --sim)",
}


@dataclass
class OnlyPlan:
    """What `--only` runs: a suite selection, playback tiers and clips, and ids it cannot run."""

    selection: Selection
    playback: tuple[str, ...] = ()
    clips: list[str] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)
    every_clip: bool = False


def add_vitest(plan: OnlyPlan, locator: str) -> None:
    if locator.startswith("tools/"):
        plan.selection.scopes["tools"].add(locator.split("/")[1])
    else:
        plan.selection.scopes["web"].add_everything()


def add_clip(plan: OnlyPlan, locator: str) -> None:
    clip = locator.partition(":")[0]
    if clip not in plan.clips:
        plan.clips.append(clip)


def add_every_clip(plan: OnlyPlan, locator: str) -> None:
    del locator
    plan.every_clip = True


HANDLERS: dict[str, Callable[[OnlyPlan, str], None]] = {
    "gtest": lambda plan, loc: plan.selection.scopes["gtest"].add(loc),
    "pytest": lambda plan, loc: plan.selection.scopes["pytest"].add(loc),
    "native": lambda plan, loc: plan.selection.scopes["firmware"].add(loc.partition("::")[0]),
    "vitest": add_vitest,
    "playback": add_clip,
    "mcap": add_every_clip,
}


def plan_only(ids: list[str]) -> OnlyPlan:
    plan = OnlyPlan(Selection())
    for proof in ids:
        parsed = proofs.parse(proof)
        if parsed is None:
            plan.problems.append(f"{proof!r} is not a proof id")
        elif parsed.runner in HANDLERS:
            HANDLERS[parsed.runner](plan, parsed.locator)
        else:
            plan.problems.append(NOT_RUNNABLE[parsed.runner])
    if plan.clips or plan.every_clip:
        plan.playback = ("lockstep",)
        plan.clips = [] if plan.every_clip else sorted(plan.clips)
    return plan
