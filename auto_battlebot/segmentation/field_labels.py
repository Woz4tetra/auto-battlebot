"""Parse a floor-mask filename into (field type, scene, source frame)."""

from __future__ import annotations

import re
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from auto_battlebot.compat import tomllib

OVERRIDES_PATH = Path(__file__).with_name("field_overrides.toml")

UNCLASSIFIED = "unclassified"

DUPLICATE_PREFIX = "floor_dataset__"

_AUGMENT_RE = re.compile(r"_augment-(\d+)$")

_CAGE_RE = re.compile(r"(?:^|[_-])Cage-(\d)(?:[_-]|$)")
_MOBILECAM_RE = re.compile(r"(?:^|[_-])MobileCam-(\d)(?:[_-]|$)")
_SKYCAM_RE = re.compile(r"(?:^|[_-])SkyCam-([A-Z])(?:[_-]|$)")

_ZED_RE = re.compile(r"^zed_(\d{4})-")
_MINI_BOT_RE = re.compile(r"^mini_bot_")
_OWN_RECENT_RE = re.compile(r"^(\d{4})-(\d{2})-\d{2}[T_]\d{2}-\d{2}-\d{2}")

_YOLO_SEG_TAIL_RE = re.compile(r"_yolo_seg__frame_\d+$")
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
    """Field type for a canonical stem."""
    cage = _CAGE_RE.search(stem)
    if cage:
        return f"cage{cage.group(1)}"
    if _MOBILECAM_RE.search(stem):
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
    """Scene to field corrections from field_overrides.toml."""
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
