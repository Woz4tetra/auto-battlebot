"""Parse a floor-mask filename into (field type, scene, source frame).

The floor-mask corpus is a flat pile of `.jpg` + `_mask.png` pairs whose only
provenance is the filename. Three facts have to come back out of it:

    field   the physical arena the frame was shot in -- the unit of diversity for
            the DeepLab data question. Cage-1..7 are NHRL's cages; MobileCam and
            SkyCam are NHRL broadcast rigs whose cage is not named in the file;
            zed2023 / own_recent are our own recordings; mini_bot is the test rig.
    scene   the recording. Splits are drawn scene-disjoint, so this is what must
            not cross the train/eval line.
    source  the labelled frame identity. Offline `_augment-N` copies share it with
            their original, which is how the leak in the shipped split is found.

Field is inferred, never verified -- see the "Risks" section of
docs/experiments/perception_performance/deeplab_field_data_plan.md.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]

OVERRIDES_PATH = Path(__file__).with_name("field_overrides.toml")

# Frames whose arena cannot be read off the filename. Kept and reported, never a probe.
UNCLASSIFIED = "unclassified"

# A merge artifact: merge_segmask_datasets.py re-imported a tree that was already
# present, prefixing every name. The unprefixed twin is the original.
DUPLICATE_PREFIX = "floor_dataset__"

_AUGMENT_RE = re.compile(r"_augment-(\d+)$")

# The camera token appears either as a bare filename prefix (`Cage-6-Red-2024-...`)
# or inside a BrettZone export (`..._tournamentID-nhrl_oct24_3lb_Cage-6-Overhead-High_...`,
# `BZ-nhrl_feb26_3lb-cole-anubis-W-53-Cage-2-Overhead-High_...`). Matching anywhere in
# the stem covers all three without enumerating the export formats.
_CAGE_RE = re.compile(r"(?:^|[_-])Cage-(\d)(?:[_-]|$)")
_MOBILECAM_RE = re.compile(r"(?:^|[_-])MobileCam-(\d)(?:[_-]|$)")
_SKYCAM_RE = re.compile(r"(?:^|[_-])SkyCam-([A-Z])(?:[_-]|$)")

_ZED_RE = re.compile(r"^zed_(\d{4})-")
_MINI_BOT_RE = re.compile(r"^mini_bot_")
# Our own recordings are named by capture time and nothing else.
_OWN_RECENT_RE = re.compile(r"^(\d{4})-(\d{2})-\d{2}[T_]\d{2}-\d{2}-\d{2}")

# Frame-index tails, in the two conventions the corpus uses. Stripping them collapses
# every frame of a recording onto one scene key.
_YOLO_SEG_TAIL_RE = re.compile(r"_yolo_seg__frame_\d+$")
# Roboflow exports: `<scene>-<frame>_jpg.rf.<hash>` or `<scene>_repaired_<frame>_jpg.rf.<hash>`.
_ROBOFLOW_TAIL_RE = re.compile(r"(?:_repaired_\d+)?_jpg\.rf\.[0-9a-f]+$")
_TRAILING_FRAME_RE = re.compile(r"-\d{4,}$")


@dataclass(frozen=True)
class FrameLabel:
    """What a filename says about the frame behind it."""

    canonical: str
    """Stem with the duplicate prefix and any `_augment-N` suffix removed."""

    source: str
    """Identity of the labelled frame. Augment siblings share one source."""

    scene: str
    """The recording. Never crosses the train/eval line."""

    field: str
    """Physical arena. The unit of diversity for the data question."""

    augment_index: int | None
    """N of the `_augment-N` copy, or None for an original."""

    is_prefixed: bool
    """Carries the `floor_dataset__` merge prefix, i.e. a duplicate candidate."""


def parse_field(stem: str) -> str:
    """Field type for a canonical stem.

    Cage tokens win over camera-rig tokens: a BrettZone export names both the
    tournament and the cage, and the cage is the arena.
    """
    cage = _CAGE_RE.search(stem)
    if cage:
        return f"cage{cage.group(1)}"
    if _MOBILECAM_RE.search(stem):
        # MobileCam-1/3/4 are movable rigs; which cage they point at is not in the name,
        # so they group as one "field" and are reported as such.
        return "mobilecam"
    if _SKYCAM_RE.search(stem):
        return "skycam"
    if _MINI_BOT_RE.match(stem):
        return "mini_bot"
    zed = _ZED_RE.match(stem)
    if zed:
        return f"zed{zed.group(1)}"
    own = _OWN_RECENT_RE.match(stem)
    if own:
        return "own_recent"
    return UNCLASSIFIED


def parse_scene(stem: str) -> str:
    """Recording key for a canonical stem: the stem with its frame index removed."""
    scene = _YOLO_SEG_TAIL_RE.sub("", stem)
    if scene != stem:
        return scene
    scene = _ROBOFLOW_TAIL_RE.sub("", stem)
    return _TRAILING_FRAME_RE.sub("", scene)


@lru_cache(maxsize=1)
def scene_overrides() -> dict[str, str]:
    """Scene -> field corrections found by the Phase-0 spot-check.

    The parser only sees the filename, and for a handful of recordings the filename is
    wrong about the arena. See field_overrides.toml for what was corrected and why.
    """
    if not OVERRIDES_PATH.exists():
        return {}
    return dict(tomllib.loads(OVERRIDES_PATH.read_text()).get("scenes", {}))


def parse_name(stem: str) -> FrameLabel:
    """Full label for an image stem (no extension, no `_mask` suffix)."""
    is_prefixed = stem.startswith(DUPLICATE_PREFIX)
    canonical = stem[len(DUPLICATE_PREFIX) :] if is_prefixed else stem

    augment_match = _AUGMENT_RE.search(canonical)
    augment_index = int(augment_match.group(1)) if augment_match else None
    source = _AUGMENT_RE.sub("", canonical)

    scene = parse_scene(source)
    return FrameLabel(
        canonical=canonical,
        source=source,
        scene=scene,
        field=scene_overrides().get(scene, parse_field(source)),
        augment_index=augment_index,
        is_prefixed=is_prefixed,
    )
