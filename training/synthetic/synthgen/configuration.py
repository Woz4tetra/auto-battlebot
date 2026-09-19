"""Typed configuration parsed from config.toml (no Blender imports).

Every default that used to live in a scattered ``dict.get(key, default)`` call
is centralized here, with the same values the original code used. The TOML
schema itself is a frozen contract: keys are neither added nor renamed, and
unknown keys (e.g. ``randomization.light_color_temp_range``,
``environment.ground_textures``) are tolerated because sibling scripts consume
them.
"""

import os
import tomllib
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

from synthgen.annotations import normalize_annotation_mode
from synthgen.cage_mount import WALLS, CageMountRanges
from synthgen.colorspec import ColorMappingEntry
from synthgen.constants import ANNOTATION_MODE_SEGMENTATION_BBOX, VIEWS
from synthgen.damage import CUTTER_SHAPES
from synthgen.geometry import model_to_blender_local

_PROJECT_ROOT = Path(__file__).resolve().parents[3]


class ConfigError(ValueError):
    """A config.toml value is missing or malformed."""


class PathResolver:
    """Resolve relative config paths the way the original script did.

    Candidates are tried in order: config directory, launch CWD (BlenderProc
    re-executes the script from a temp dir, so the real CWD arrives via the
    ``BLENDERPROC_CWD`` environment variable), then the project root. The first
    existing candidate wins; if none exist, the first candidate is returned so
    error messages show the most likely intended location.
    """

    def __init__(self, config_dir: Path, launch_cwd: Path, project_root: Path) -> None:
        self.config_dir = config_dir
        self.launch_cwd = launch_cwd
        self.project_root = project_root

    def resolve(self, path: Path) -> Path:
        """Resolve *path* against the candidate base directories."""
        if path.is_absolute():
            return path
        candidates = [
            (self.config_dir / path).resolve(),
            (self.launch_cwd / path).resolve(),
            (self.project_root / path).resolve(),
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[0]


@dataclass(frozen=True)
class OutputConfig:
    """``[output]`` section."""

    image_dir: Path
    label_dir: Path
    num_images: int
    annotation_mode: str = "keypoints_bbox"
    image_width: int = 1280
    image_height: int = 720
    images_per_scene: int = 5
    min_robot_visibility: float = 0.10
    ignore_obstructions: bool = False
    memory_cleanup_interval: int = 25
    segmentation_min_bbox_dim: int = 1

    @property
    def is_segmentation_mode(self) -> bool:
        """True when producing YOLO segmentation polygons instead of keypoints."""
        return self.annotation_mode == ANNOTATION_MODE_SEGMENTATION_BBOX


@dataclass(frozen=True, eq=False)
class KeypointPair:
    """Front/back keypoints in Blender local axes (converted from model space)."""

    front: np.ndarray
    back: np.ndarray


@dataclass(frozen=True)
class DamagePartConfig:
    """One ``[[robots.damage_parts]]`` entry: a named assembly battle damage can remove.

    Faces whose centres fall in ``boxes`` are split off into their own objects at load, so
    a draw can hide the assembly whole. Entries are matched in config order and the first
    one to claim a face keeps it.
    """

    name: str
    # Each piece is boxes, [x_min, y_min, z_min, x_max, y_max, z_max] in the model's native
    # frame like keypoints. A plain part is one piece; a subset part loses 1..n of them.
    pieces: tuple[tuple[tuple[float, float, float, float, float, float], ...], ...]
    # Only faces of these Blender objects (``.001`` suffix ignored). Empty means any object.
    objects: tuple[str, ...] = ()
    exclude_objects: tuple[str, ...] = ()
    # A draw removes 1..len(pieces) of the pieces rather than all of them (the wheels).
    subset: bool = False
    # Other parts of the same robot that go whenever this one does.
    includes: tuple[str, ...] = ()
    # Subset parts: relative odds of removing 1, 2, ... pieces. Empty means uniform.
    count_weights: tuple[float, ...] = ()
    # False for a part that only goes through another's ``includes``, such as a decal.
    selectable: bool = True


@dataclass(frozen=True)
class RobotConfig:
    """One ``[[robots]]`` entry."""

    name: str
    model_path: Path
    keypoints: KeypointPair
    color_mapping: tuple[ColorMappingEntry, ...]
    class_id: int | None = None
    scale: float = 1.0
    weight: float = 1.0
    ground_roll_upright: float = 0.0
    ground_roll_inverted: float = 0.0
    # True for Z-up GLBs (e.g. Meshy models): sit flat at identity pitch, like distractor CAD
    # robots, instead of the +/-90 deg pitch the OnShape .gltf (Y-up) robots need.
    flat_ground: bool = False
    damage_parts: tuple[DamagePartConfig, ...] = ()


@dataclass(frozen=True)
class MaterialConfig:
    """One ``[materials.<name>]`` entry."""

    metallic: float = 0.0
    roughness: float = 0.5
    cc_texture: str | None = None
    texture_dir: Path | None = None
    base_color: tuple[int, int, int] | None = None


@dataclass(frozen=True)
class DistractorSource:
    """One ``[[distractors.sources]]`` entry."""

    path: Path
    weight: float = 1.0
    kind: str = ""

    def effective_kind(self) -> str:
        """The configured kind, or a heuristic from the directory name."""
        kind = self.kind.strip().lower()
        if kind:
            return kind
        return "objaverse" if "objaverse" in self.path.name.lower() else "cad"


@dataclass(frozen=True)
class DistractorsConfig:
    """``[distractors]`` section."""

    sources: tuple[DistractorSource, ...] = ()
    min_per_scene: int = 0
    max_per_scene: int = 5
    vram_budget_mb: float | None = None
    vram_audit_csv: Path = Path("training/data/distractor_models/distractor_gpu_audit.csv")
    base_dimension_m: float = 0.25
    scale_range: tuple[float, float] = (0.5, 3.0)
    shuffle_interval: int = 100
    # None means: fall back to the [randomization] value at the use site.
    robot_air_probability: float | None = None
    robot_air_height_range: tuple[float, float] | None = None
    motion_blur_probability: float | None = None

    def has_cad_source(self) -> bool:
        """True when any source is CAD-kind (as configured, matching original)."""
        return any(s.kind.strip().lower() == "cad" for s in self.sources)


@dataclass(frozen=True)
class EnvironmentConfig:
    """``[environment]`` section."""

    hdri_dir: Path = Path("data/hdris")
    cc_textures_dir: Path | None = None


@dataclass(frozen=True)
class CameraConfig:
    """``[camera]`` section."""

    min_distance: float = 0.3
    max_distance: float = 1.5
    height_range: tuple[float, float] = (0.1, 0.8)
    look_at_noise: float = 0.05
    max_frame_fraction: float = 0.9


@dataclass(frozen=True)
class SceneConfig:
    """``[scene]`` section."""

    ground_size_range: tuple[float, float] = (2.0, 5.0)
    arena_radius_range: tuple[float, float] = (0.5, 1.5)
    ground_visibility: float = 0.8
    max_robots_per_scene: int = 1


@dataclass(frozen=True)
class CageConfig:
    """One ``[[cages]]`` entry: a real arena the scene mix renders some of its images in."""

    name: str = "nhrl_cage"
    enabled: bool = False
    probability: float = 0.5
    spec: Path = Path("cage/cage2_overhead_high.toml")
    camera_calibration: Path = Path("config/cameras/brettzone_cage_high.toml")
    # None keeps the run's --render-samples; a cage is darker and has glass, so it usually
    # wants more.
    render_samples: int | None = None
    tube_jitter: float = 0.25
    mat_margin_m: float = 0.20
    mount: CageMountRanges = CageMountRanges()
    # The frame this cage writes: `pinhole` at the rectified matrix, `distorted` as the sensor
    # sees it, or `rectified` through the C++ Rectifier's maps. See synthgen.lens.
    view: str = "pinhole"
    # getOptimalNewCameraMatrix alpha behind the rectified matrix; 1.0 keeps every sensor pixel.
    rectify_alpha: float = 1.0
    # Draw a new house bot position and heading every scene (synthgen.house_bot_pose). False
    # leaves it where the spec's [house_bot_box] put it. No effect on a cage without one.
    randomize_house_bot: bool = False
    # Gap kept between the house bot's footprint circle and each robot's, metres.
    house_bot_clearance_m: float = 0.05
    # Footprint radius assumed for a robot when keeping the house bot clear of it, metres.
    house_bot_robot_radius_m: float = 0.20

    @property
    def active(self) -> bool:
        return self.enabled and self.probability > 0.0

    def is_behind(self, images_written: int, own_images: int) -> bool:
        """Whether this cage owes the run images at its configured share."""
        if not self.active:
            return False
        return own_images <= self.probability * images_written


def choose_cage(
    cages: Sequence[CageConfig], images_written: int, images_per_cage: Sequence[int]
) -> int | None:
    """Index of the cage the next scene belongs to, or None for the HDRI arena.

    The split is tracked rather than coin-flipped: a scene goes to whichever cage is furthest
    behind its share of the images written so far, so even a 100-image run lands on the
    configured ratios instead of somewhere in their binomial spread. When no cage is behind,
    the scene goes to the arena, which is what is left over.
    """
    behind = [
        # Ties go to the larger share, which matters on the first scene of a run where every
        # cage is equally (and entirely) behind.
        (cage.probability * images_written - own, cage.probability, index)
        for index, (cage, own) in enumerate(zip(cages, images_per_cage))
        if cage.is_behind(images_written, own)
    ]
    return max(behind)[2] if behind else None


@dataclass(frozen=True)
class RandomizationConfig:
    """``[randomization]`` section."""

    roughness_jitter: float = 0.1
    hue_jitter_degrees: float = 5.0
    light_count_range: tuple[int, int] = (1, 3)
    light_intensity_range: tuple[float, float] = (100.0, 500.0)
    air_probability: float = 0.15
    air_height_range: tuple[float, float] = (0.02, 0.15)
    motion_blur_probability: float = 0.0
    motion_blur_strength_range: tuple[int, int] = (5, 25)


@dataclass(frozen=True)
class DamageConfig:
    """``[damage]`` section: battle damage drawn per robot instance per scene.

    Applied by ``synthgen.damage_scene`` and reverted when the scene's frames are written,
    then recorded per instance in ``manifest.jsonl`` so damage-on and damage-off arms are
    two filters over one render rather than two renders.
    """

    enabled: bool = False
    # Chance a scene is a damaged scene at all. Damage is drawn per scene, because one
    # render call covers all of a scene's camera poses, so this is what sets the size of
    # the fully clean pool: it is exactly 1 - scene_probability, whatever a scene holds.
    scene_probability: float = 0.5
    # Chance each robot instance inside a damaged scene is damaged.
    probability: float = 0.35
    # Fraction of a part robot's removable parts to hide.
    part_severity: tuple[float, float] = (0.05, 0.30)
    # Fraction of a fused mesh's bounding volume the cutter is sized to take.
    chunk_volume_fraction: tuple[float, float] = (0.03, 0.20)
    # Parts holding more than this share of the summed part bounding volumes never go.
    max_part_volume_fraction: float = 0.45
    # Keypoint anchors are protected with this much clearance.
    keypoint_clearance_m: float = 0.02
    # Lowercase substrings of Blender object names that must survive.
    protected_name_patterns: tuple[str, ...] = ()
    # Named parts ([[robots.damage_parts]]) this batch may remove. Empty allows every part.
    removable_parts: tuple[str, ...] = ()
    # How many distinct named parts one damaged instance loses, clamped to what is allowed.
    part_count: tuple[int, int] = (1, 2)
    cutter_shapes: tuple[str, ...] = CUTTER_SHAPES
    # Cutters pre-built at startup. One scene can need one per robot-like instance.
    cutter_pool_size: int = 16


@dataclass(frozen=True)
class RenderConfig:
    """The full parsed config plus the path resolver used to load it."""

    output: OutputConfig
    robots: tuple[RobotConfig, ...]
    materials: dict[str, MaterialConfig] = field(default_factory=dict)
    distractors: DistractorsConfig = DistractorsConfig()
    environment: EnvironmentConfig = EnvironmentConfig()
    camera: CameraConfig = CameraConfig()
    scene: SceneConfig = SceneConfig()
    randomization: RandomizationConfig = RandomizationConfig()
    cages: tuple[CageConfig, ...] = ()
    damage: DamageConfig = DamageConfig()
    resolver: PathResolver = PathResolver(Path("."), Path("."), _PROJECT_ROOT)


def _require(section: dict[str, Any], key: str, context: str) -> Any:
    if key not in section:
        raise ConfigError(f"{context}: missing required key '{key}'")
    return section[key]


def _as_float(value: Any, context: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as e:
        raise ConfigError(f"{context}: expected a number, got {value!r}") from e


def _as_int(value: Any, context: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as e:
        raise ConfigError(f"{context}: expected an integer, got {value!r}") from e


def _as_pair(value: Any, context: str) -> tuple[float, float]:
    try:
        lo, hi = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError, KeyError) as e:
        raise ConfigError(f"{context}: expected a [lo, hi] pair, got {value!r}") from e
    return (lo, hi)


def _as_int_pair(value: Any, context: str) -> tuple[int, int]:
    lo, hi = _as_pair(value, context)
    return (int(lo), int(hi))


def _as_rgb(value: Any, context: str) -> tuple[int, int, int]:
    try:
        r, g, b = int(value[0]), int(value[1]), int(value[2])
    except (TypeError, ValueError, IndexError, KeyError) as e:
        raise ConfigError(f"{context}: expected an [r, g, b] color, got {value!r}") from e
    return (r, g, b)


def _parse_output(section: dict[str, Any]) -> OutputConfig:
    context = "[output]"
    try:
        annotation_mode = normalize_annotation_mode(
            str(section.get("annotation_mode", "keypoints_bbox"))
        )
    except ValueError as e:
        raise ConfigError(str(e)) from e
    return OutputConfig(
        image_dir=Path(str(_require(section, "image_dir", context))),
        label_dir=Path(str(_require(section, "label_dir", context))),
        num_images=_as_int(_require(section, "num_images", context), f"{context}.num_images"),
        annotation_mode=annotation_mode,
        image_width=_as_int(section.get("image_width", 1280), f"{context}.image_width"),
        image_height=_as_int(section.get("image_height", 720), f"{context}.image_height"),
        images_per_scene=_as_int(section.get("images_per_scene", 5), f"{context}.images_per_scene"),
        min_robot_visibility=_as_float(
            section.get("min_robot_visibility", 0.10), f"{context}.min_robot_visibility"
        ),
        ignore_obstructions=bool(section.get("ignore_obstructions", False)),
        memory_cleanup_interval=_as_int(
            section.get("memory_cleanup_interval", 25), f"{context}.memory_cleanup_interval"
        ),
        segmentation_min_bbox_dim=_as_int(
            section.get("segmentation_min_bbox_dim", 1), f"{context}.segmentation_min_bbox_dim"
        ),
    )


def _parse_keypoints(section: dict[str, Any], context: str) -> KeypointPair:
    front = _require(section, "front", context)
    back = _require(section, "back", context)
    try:
        return KeypointPair(front=model_to_blender_local(front), back=model_to_blender_local(back))
    except (TypeError, ValueError) as e:
        raise ConfigError(f"{context}: keypoints must be [x, y, z] positions") from e


def _parse_color_mapping(
    entries: list[dict[str, Any]], context: str
) -> tuple[ColorMappingEntry, ...]:
    mapping = []
    for i, entry in enumerate(entries):
        entry_context = f"{context}.color_mapping[{i}]"
        mapping.append(
            ColorMappingEntry(
                color=_as_rgb(_require(entry, "color", entry_context), entry_context),
                tolerance=_as_float(
                    _require(entry, "tolerance", entry_context), f"{entry_context}.tolerance"
                ),
                material=str(_require(entry, "material", entry_context)),
            )
        )
    return tuple(mapping)


def _parse_robot(section: dict[str, Any], index: int) -> RobotConfig:
    context = f"robots[{index}]"
    model_path = Path(str(_require(section, "model_path", context)))
    class_id = section.get("class_id")
    return RobotConfig(
        name=str(section.get("name", model_path.stem)),
        model_path=model_path,
        keypoints=_parse_keypoints(_require(section, "keypoints", context), f"{context}.keypoints"),
        color_mapping=_parse_color_mapping(_require(section, "color_mapping", context), context),
        class_id=None if class_id is None else _as_int(class_id, f"{context}.class_id"),
        scale=_as_float(section.get("scale", 1.0), f"{context}.scale"),
        weight=_as_float(section.get("weight", 1.0), f"{context}.weight"),
        ground_roll_upright=_as_float(
            section.get("ground_roll_upright", 0.0), f"{context}.ground_roll_upright"
        ),
        ground_roll_inverted=_as_float(
            section.get("ground_roll_inverted", 0.0), f"{context}.ground_roll_inverted"
        ),
        flat_ground=bool(section.get("flat_ground", False)),
        damage_parts=_parse_damage_parts(section.get("damage_parts", []), context),
    )


def _as_box(value: Any, context: str) -> tuple[float, float, float, float, float, float]:
    try:
        x0, y0, z0, x1, y1, z1 = (float(v) for v in value)
    except (TypeError, ValueError) as e:
        raise ConfigError(
            f"{context}: expected [x_min, y_min, z_min, x_max, y_max, z_max], got {value!r}"
        ) from e
    if x0 > x1 or y0 > y1 or z0 > z1:
        raise ConfigError(f"{context}: a min bound exceeds its max in {value!r}")
    return (x0, y0, z0, x1, y1, z1)


def _as_count_weights(entry: dict[str, Any], piece_count: int, context: str) -> tuple[float, ...]:
    raw = entry.get("count_weights", [])
    weights = tuple(_as_float(w, f"{context}.count_weights[{i}]") for i, w in enumerate(raw))
    if not weights:
        return weights
    if not entry.get("subset", False):
        raise ConfigError(f"{context}.count_weights: only a subset part draws a piece count")
    if len(weights) > piece_count or any(w < 0.0 for w in weights) or sum(weights) <= 0.0:
        raise ConfigError(
            f"{context}.count_weights: expected at most {piece_count} non-negative weights with"
            f" a positive sum, got {list(weights)}"
        )
    return weights


def _parse_part_pieces(
    entry: dict[str, Any], context: str
) -> tuple[tuple[tuple[float, float, float, float, float, float], ...], ...]:
    """A part's pieces: ``pieces`` as given, or ``boxes`` as one piece or one per box.

    ``boxes`` on a subset part makes each box its own piece, which suits a wheel. A piece
    that needs several boxes, such as a guard with a side wall and a front arm, uses
    ``pieces``, which only a subset part may.
    """
    boxes = entry.get("boxes")
    pieces = entry.get("pieces")
    if (boxes is None) == (pieces is None):
        raise ConfigError(f"{context}: give exactly one of 'boxes' or 'pieces'")
    subset = bool(entry.get("subset", False))
    if pieces is not None:
        if not subset:
            raise ConfigError(f"{context}.pieces: only a subset part has separate pieces")
        parsed = tuple(
            tuple(_as_box(box, f"{context}.pieces[{j}][{k}]") for k, box in enumerate(piece))
            for j, piece in enumerate(pieces)
        )
        if not parsed or any(not piece for piece in parsed):
            raise ConfigError(f"{context}.pieces: every piece needs at least one box")
        return parsed
    flat = tuple(_as_box(box, f"{context}.boxes[{j}]") for j, box in enumerate(boxes))
    if not flat:
        raise ConfigError(f"{context}.boxes: at least one box is required")
    return tuple((box,) for box in flat) if subset else (flat,)


def _parse_damage_parts(
    entries: list[dict[str, Any]], context: str
) -> tuple[DamagePartConfig, ...]:
    parts = []
    for i, entry in enumerate(entries):
        part_context = f"{context}.damage_parts[{i}]"
        pieces = _parse_part_pieces(entry, part_context)
        parts.append(
            DamagePartConfig(
                name=str(_require(entry, "name", part_context)),
                pieces=pieces,
                objects=tuple(str(o) for o in entry.get("objects", [])),
                exclude_objects=tuple(str(o) for o in entry.get("exclude_objects", [])),
                subset=bool(entry.get("subset", False)),
                includes=tuple(str(n) for n in entry.get("includes", [])),
                count_weights=_as_count_weights(entry, len(pieces), part_context),
                selectable=bool(entry.get("selectable", True)),
            )
        )
    names = [part.name for part in parts]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ConfigError(f"{context}.damage_parts: duplicate names {duplicates}")
    for part in parts:
        unknown = [name for name in part.includes if name not in names]
        if unknown:
            raise ConfigError(
                f"{context}.damage_parts {part.name!r}.includes: unknown {unknown};"
                f" valid are {names}"
            )
    return tuple(parts)


def _parse_material(name: str, section: dict[str, Any]) -> MaterialConfig:
    context = f"[materials.{name}]"
    texture_dir = section.get("texture_dir")
    base_color = section.get("base_color")
    return MaterialConfig(
        metallic=_as_float(section.get("metallic", 0.0), f"{context}.metallic"),
        roughness=_as_float(section.get("roughness", 0.5), f"{context}.roughness"),
        cc_texture=(None if section.get("cc_texture") is None else str(section["cc_texture"])),
        texture_dir=None if texture_dir is None else Path(str(texture_dir)),
        base_color=None if base_color is None else _as_rgb(base_color, f"{context}.base_color"),
    )


def _parse_distractors(section: dict[str, Any]) -> DistractorsConfig:
    context = "[distractors]"
    sources = []
    for i, src in enumerate(section.get("sources", [])):
        src_context = f"{context}.sources[{i}]"
        sources.append(
            DistractorSource(
                path=Path(str(_require(src, "path", src_context))),
                weight=_as_float(src.get("weight", 1.0), f"{src_context}.weight"),
                kind=str(src.get("kind", "")),
            )
        )
    vram_budget = section.get("vram_budget_mb")
    robot_air_probability = section.get("robot_air_probability")
    robot_air_height_range = section.get("robot_air_height_range")
    motion_blur_probability = section.get("motion_blur_probability")
    return DistractorsConfig(
        sources=tuple(sources),
        min_per_scene=_as_int(section.get("min_per_scene", 0), f"{context}.min_per_scene"),
        max_per_scene=_as_int(section.get("max_per_scene", 5), f"{context}.max_per_scene"),
        vram_budget_mb=(
            None if vram_budget is None else _as_float(vram_budget, f"{context}.vram_budget_mb")
        ),
        vram_audit_csv=Path(
            str(
                section.get(
                    "vram_audit_csv", "training/data/distractor_models/distractor_gpu_audit.csv"
                )
            )
        ),
        base_dimension_m=_as_float(
            section.get("base_dimension_m", 0.25), f"{context}.base_dimension_m"
        ),
        scale_range=_as_pair(section.get("scale_range", [0.5, 3.0]), f"{context}.scale_range"),
        shuffle_interval=_as_int(
            section.get("shuffle_interval", 100), f"{context}.shuffle_interval"
        ),
        robot_air_probability=(
            None
            if robot_air_probability is None
            else _as_float(robot_air_probability, f"{context}.robot_air_probability")
        ),
        robot_air_height_range=(
            None
            if robot_air_height_range is None
            else _as_pair(robot_air_height_range, f"{context}.robot_air_height_range")
        ),
        motion_blur_probability=(
            None
            if motion_blur_probability is None
            else _as_float(motion_blur_probability, f"{context}.motion_blur_probability")
        ),
    )


def _parse_environment(section: dict[str, Any]) -> EnvironmentConfig:
    cc_dir = section.get("cc_textures_dir")
    return EnvironmentConfig(
        hdri_dir=Path(str(section.get("hdri_dir", "data/hdris"))),
        cc_textures_dir=None if cc_dir is None else Path(str(cc_dir)),
    )


def _parse_camera(section: dict[str, Any]) -> CameraConfig:
    context = "[camera]"
    return CameraConfig(
        min_distance=_as_float(section.get("min_distance", 0.3), f"{context}.min_distance"),
        max_distance=_as_float(section.get("max_distance", 1.5), f"{context}.max_distance"),
        height_range=_as_pair(section.get("height_range", [0.1, 0.8]), f"{context}.height_range"),
        look_at_noise=_as_float(section.get("look_at_noise", 0.05), f"{context}.look_at_noise"),
        max_frame_fraction=_as_float(
            section.get("max_frame_fraction", 0.9), f"{context}.max_frame_fraction"
        ),
    )


def _parse_scene(section: dict[str, Any]) -> SceneConfig:
    context = "[scene]"
    return SceneConfig(
        ground_size_range=_as_pair(
            section.get("ground_size_range", [2.0, 5.0]), f"{context}.ground_size_range"
        ),
        arena_radius_range=_as_pair(
            section.get("arena_radius_range", [0.5, 1.5]), f"{context}.arena_radius_range"
        ),
        ground_visibility=_as_float(
            section.get("ground_visibility", 0.8), f"{context}.ground_visibility"
        ),
        max_robots_per_scene=_as_int(
            section.get("max_robots_per_scene", 1), f"{context}.max_robots_per_scene"
        ),
    )


def _parse_mount(section: dict[str, Any], parent_context: str = "[[cages]]") -> CageMountRanges:
    context = f"{parent_context}.mount"
    defaults = CageMountRanges()
    walls = tuple(str(w) for w in section.get("walls", defaults.walls))
    if not walls:
        raise ConfigError(f"{context}.walls: at least one wall is required")
    unknown = [w for w in walls if w not in WALLS]
    if unknown:
        raise ConfigError(f"{context}.walls: unknown {unknown}; valid walls are {list(WALLS)}")
    aim = str(section.get("aim", defaults.aim))
    if aim not in ("fixed", "centre"):
        raise ConfigError(f"{context}.aim: expected 'fixed' or 'centre', got {aim!r}")
    return CageMountRanges(
        walls=walls,
        aim=aim,
        tilt_offset_deg=_as_pair(
            section.get("tilt_offset_deg", defaults.tilt_offset_deg), f"{context}.tilt_offset_deg"
        ),
        along_m=_as_pair(section.get("along_m", defaults.along_m), f"{context}.along_m"),
        height_m=_as_pair(section.get("height_m", defaults.height_m), f"{context}.height_m"),
        inset_m=_as_pair(section.get("inset_m", defaults.inset_m), f"{context}.inset_m"),
        tilt_deg=_as_pair(section.get("tilt_deg", defaults.tilt_deg), f"{context}.tilt_deg"),
        yaw_deg=_as_pair(section.get("yaw_deg", defaults.yaw_deg), f"{context}.yaw_deg"),
        roll_deg=_as_pair(section.get("roll_deg", defaults.roll_deg), f"{context}.roll_deg"),
    )


def _parse_cages(entries: Any) -> tuple[CageConfig, ...]:
    """``[[cages]]``: one entry per real arena the run can render in."""
    if not isinstance(entries, list):
        raise ConfigError(f"[[cages]]: expected an array of tables, got {type(entries).__name__}")
    cages = tuple(_parse_cage(entry) for entry in entries)
    names = [cage.name for cage in cages]
    if len(set(names)) != len(names):
        raise ConfigError(f"[[cages]]: names must be unique, got {names}")
    total = sum(cage.probability for cage in cages if cage.active)
    if total > 1.0 + 1e-9:
        raise ConfigError(
            f"[[cages]]: enabled probabilities sum to {total:.2f}; they share the run with the "
            "HDRI arena, so they cannot exceed 1.0"
        )
    return cages


def _parse_cage(section: dict[str, Any]) -> CageConfig:
    defaults = CageConfig()
    name = str(section.get("name", defaults.name))
    context = f"[[cages]] {name}"
    probability = _as_float(
        section.get("probability", defaults.probability), f"{context}.probability"
    )
    if not 0.0 <= probability <= 1.0:
        raise ConfigError(f"{context}.probability: expected 0.0 to 1.0, got {probability}")
    render_samples = section.get("render_samples")
    view = str(section.get("view", defaults.view))
    if view not in VIEWS:
        raise ConfigError(f"{context}.view: unknown {view!r}; valid are {list(VIEWS)}")
    rectify_alpha = _as_float(
        section.get("rectify_alpha", defaults.rectify_alpha), f"{context}.rectify_alpha"
    )
    if not 0.0 <= rectify_alpha <= 1.0:
        raise ConfigError(f"{context}.rectify_alpha: expected 0.0 to 1.0, got {rectify_alpha}")
    return CageConfig(
        name=name,
        enabled=bool(section.get("enabled", defaults.enabled)),
        probability=probability,
        spec=Path(str(section.get("spec", defaults.spec))),
        camera_calibration=Path(
            str(section.get("camera_calibration", defaults.camera_calibration))
        ),
        render_samples=(
            None if render_samples is None else _as_int(render_samples, f"{context}.render_samples")
        ),
        tube_jitter=_as_float(
            section.get("tube_jitter", defaults.tube_jitter), f"{context}.tube_jitter"
        ),
        mat_margin_m=_as_float(
            section.get("mat_margin_m", defaults.mat_margin_m), f"{context}.mat_margin_m"
        ),
        mount=_parse_mount(section.get("mount", {}), context),
        view=view,
        rectify_alpha=rectify_alpha,
        randomize_house_bot=bool(section.get("randomize_house_bot", defaults.randomize_house_bot)),
        house_bot_clearance_m=_as_float(
            section.get("house_bot_clearance_m", defaults.house_bot_clearance_m),
            f"{context}.house_bot_clearance_m",
        ),
        house_bot_robot_radius_m=_as_float(
            section.get("house_bot_robot_radius_m", defaults.house_bot_robot_radius_m),
            f"{context}.house_bot_robot_radius_m",
        ),
    )


def _parse_randomization(section: dict[str, Any]) -> RandomizationConfig:
    context = "[randomization]"
    return RandomizationConfig(
        roughness_jitter=_as_float(
            section.get("roughness_jitter", 0.1), f"{context}.roughness_jitter"
        ),
        hue_jitter_degrees=_as_float(
            section.get("hue_jitter_degrees", 5), f"{context}.hue_jitter_degrees"
        ),
        light_count_range=_as_int_pair(
            section.get("light_count_range", [1, 3]), f"{context}.light_count_range"
        ),
        light_intensity_range=_as_pair(
            section.get("light_intensity_range", [100, 500]), f"{context}.light_intensity_range"
        ),
        air_probability=_as_float(
            section.get("air_probability", 0.15), f"{context}.air_probability"
        ),
        air_height_range=_as_pair(
            section.get("air_height_range", [0.02, 0.15]), f"{context}.air_height_range"
        ),
        motion_blur_probability=_as_float(
            section.get("motion_blur_probability", 0.0), f"{context}.motion_blur_probability"
        ),
        motion_blur_strength_range=_as_int_pair(
            section.get("motion_blur_strength_range", [5, 25]),
            f"{context}.motion_blur_strength_range",
        ),
    )


def _parse_damage(section: dict[str, Any]) -> DamageConfig:
    context = "[damage]"
    defaults = DamageConfig()
    shapes = tuple(str(s) for s in section.get("cutter_shapes", defaults.cutter_shapes))
    unknown = [s for s in shapes if s not in CUTTER_SHAPES]
    if unknown:
        raise ConfigError(
            f"{context}.cutter_shapes: unknown {unknown}; valid are {list(CUTTER_SHAPES)}"
        )
    if not shapes:
        raise ConfigError(f"{context}.cutter_shapes: at least one shape is required")
    part_count = _as_int_pair(
        section.get("part_count", defaults.part_count), f"{context}.part_count"
    )
    if part_count[0] < 1 or part_count[0] > part_count[1]:
        raise ConfigError(f"{context}.part_count: expected 1 <= lo <= hi, got {list(part_count)}")
    return DamageConfig(
        enabled=bool(section.get("enabled", defaults.enabled)),
        scene_probability=_as_float(
            section.get("scene_probability", defaults.scene_probability),
            f"{context}.scene_probability",
        ),
        probability=_as_float(
            section.get("probability", defaults.probability), f"{context}.probability"
        ),
        part_severity=_as_pair(
            section.get("part_severity", defaults.part_severity), f"{context}.part_severity"
        ),
        chunk_volume_fraction=_as_pair(
            section.get("chunk_volume_fraction", defaults.chunk_volume_fraction),
            f"{context}.chunk_volume_fraction",
        ),
        max_part_volume_fraction=_as_float(
            section.get("max_part_volume_fraction", defaults.max_part_volume_fraction),
            f"{context}.max_part_volume_fraction",
        ),
        keypoint_clearance_m=_as_float(
            section.get("keypoint_clearance_m", defaults.keypoint_clearance_m),
            f"{context}.keypoint_clearance_m",
        ),
        protected_name_patterns=tuple(
            str(pattern).lower()
            for pattern in section.get("protected_name_patterns", defaults.protected_name_patterns)
        ),
        cutter_shapes=shapes,
        cutter_pool_size=_as_int(
            section.get("cutter_pool_size", defaults.cutter_pool_size),
            f"{context}.cutter_pool_size",
        ),
        removable_parts=tuple(
            str(name) for name in section.get("removable_parts", defaults.removable_parts)
        ),
        part_count=part_count,
    )


def _check_removable_parts(
    robots: tuple[RobotConfig, ...], names: tuple[str, ...], context: str
) -> None:
    """Raise unless every name in *names* is a selectable damage part some robot defines."""
    valid = sorted(
        {part.name for robot in robots for part in robot.damage_parts if part.selectable}
    )
    unknown = [name for name in names if name not in valid]
    if unknown:
        raise ConfigError(f"{context}: unknown damage parts {unknown}; valid are {valid}")


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """*override* laid over *base*. Tables merge key by key; anything else replaces."""
    merged = dict(base)
    for key, value in override.items():
        current = merged.get(key)
        if isinstance(current, dict) and isinstance(value, dict):
            merged[key] = _deep_merge(current, value)
        else:
            merged[key] = value
    return merged


def _load_raw(config_path: Path, seen: tuple[Path, ...] = ()) -> dict[str, Any]:
    """Parse a config file, following ``extends`` so a variant need not copy the whole thing.

    A per-venue config is three keys different from the shared one, so it says what differs
    and inherits the rest. Tables merge key by key, so a variant can override
    ``[output].num_images`` without restating ``[output]``; arrays, including arrays of
    tables like ``[[robots]]``, replace wholesale, because half-merging a robot list by
    index would be a trap.

    Raises:
        ConfigError: On a cycle, a missing parent, or a parent in another directory.
    """
    resolved = config_path.resolve()
    if resolved in seen:
        chain = " -> ".join(p.name for p in (*seen, resolved))
        raise ConfigError(f"extends cycle: {chain}")
    with open(resolved, "rb") as handle:
        raw = tomllib.load(handle)
    parent_name = raw.pop("extends", None)
    if parent_name is None:
        return raw
    parent = resolved.parent / str(parent_name)
    if Path(str(parent_name)).parent != Path("."):
        raise ConfigError(
            f"extends must name a file in the same directory, got {parent_name!r}: relative"
            " paths inside the inherited config resolve against the loaded file's directory"
        )
    if not parent.exists():
        raise ConfigError(f"extends target not found: {parent}")
    return _deep_merge(_load_raw(parent, (*seen, resolved)), raw)


def _apply_only_cage(cages: tuple[CageConfig, ...], name: Any) -> tuple[CageConfig, ...]:
    """Narrow the scene mix to one cage, which then takes every scene.

    This is how a per-venue render is pinned in a file rather than on the command line:
    the named cage goes to probability 1.0 and every other cage is disabled, so no scene
    lands in the HDRI arena or the other venue.

    Raises:
        ConfigError: When no ``[[cages]]`` entry carries *name*.
    """
    if name is None:
        return cages
    wanted = str(name)
    if wanted not in [cage.name for cage in cages]:
        raise ConfigError(
            f"only_cage is {wanted!r}; no [[cages]] entry has that name"
            f" (have {[cage.name for cage in cages]})"
        )
    return tuple(
        replace(cage, enabled=cage.name == wanted, probability=1.0 if cage.name == wanted else 0.0)
        for cage in cages
    )


ARENA_VENUE = "arena"
DAMAGE_MODES = ("config", "off", "all")


def apply_venue(cfg: RenderConfig, venue: str | None) -> RenderConfig:
    """Pin every scene to one venue from the command line, the way ``only_cage`` does in a file.

    ``venue`` is a ``[[cages]]`` name, or ``"arena"`` for the HDRI arena alone, which drops
    every cage. None leaves the config's own mix.

    Raises:
        ConfigError: When *venue* names neither a cage nor the arena.
    """
    if venue is None:
        return cfg
    if venue == ARENA_VENUE:
        return replace(cfg, cages=())
    return replace(cfg, cages=_apply_only_cage(cfg.cages, venue))


def apply_damage_mode(cfg: RenderConfig, mode: str) -> RenderConfig:
    """Override the ``[damage]`` split from the command line.

    ``"config"`` keeps the file's values. ``"off"`` disables damage. ``"all"`` damages every
    scene and rolls every instance, for a set that is meant to show damage rather than
    the run's tracked mix.

    Raises:
        ConfigError: When *mode* is not one of ``DAMAGE_MODES``.
    """
    if mode == "config":
        return cfg
    if mode == "off":
        return replace(cfg, damage=replace(cfg.damage, enabled=False))
    if mode == "all":
        return replace(
            cfg, damage=replace(cfg.damage, enabled=True, scene_probability=1.0, probability=1.0)
        )
    raise ConfigError(f"damage mode must be one of {DAMAGE_MODES}, got {mode!r}")


def apply_view(cfg: RenderConfig, view: str | None) -> RenderConfig:
    """Write every cage's frames in *view* from the command line. None keeps each cage's own.

    The HDRI arena half has no lens model, so it stays pinhole whatever the view.

    Raises:
        ConfigError: When *view* is not one of ``VIEWS``.
    """
    if view is None:
        return cfg
    if view not in VIEWS:
        raise ConfigError(f"--view: unknown {view!r}; valid are {list(VIEWS)}")
    return replace(cfg, cages=tuple(replace(cage, view=view) for cage in cfg.cages))


def apply_damage_parts(cfg: RenderConfig, names: list[str] | None) -> RenderConfig:
    """Pin the named parts this batch may remove from the command line.

    None keeps ``[damage].removable_parts``.

    Raises:
        ConfigError: When a name is not a ``[[robots.damage_parts]]`` entry of any robot.
    """
    if names is None:
        return cfg
    _check_removable_parts(cfg.robots, tuple(names), "--damage-parts")
    return replace(cfg, damage=replace(cfg.damage, removable_parts=tuple(names)))


def load_render_config(
    config_path: Path,
    launch_cwd: Path | None = None,
    project_root: Path | None = None,
) -> RenderConfig:
    """Load and validate config.toml into typed dataclasses.

    Args:
        config_path: Path to config.toml (resolved against launch CWD and
            project root when relative).
        launch_cwd: Override for the launch working directory (defaults to
            ``BLENDERPROC_CWD`` or the process CWD).
        project_root: Override for the repository root.

    Returns:
        The fully parsed configuration.

    Raises:
        ConfigError: When a required key is missing or a value is malformed.
    """
    if launch_cwd is None:
        launch_cwd = Path(os.environ.get("BLENDERPROC_CWD", os.getcwd()))
    if project_root is None:
        project_root = _PROJECT_ROOT

    bootstrap = PathResolver(Path.cwd(), launch_cwd, project_root)
    resolved_config = bootstrap.resolve(config_path)
    if not resolved_config.exists():
        raise ConfigError(f"Config file not found: {config_path}")
    resolver = PathResolver(resolved_config.parent, launch_cwd, project_root)

    raw = _load_raw(resolved_config)

    if "output" not in raw:
        raise ConfigError("missing required section [output]")
    robots_raw = raw.get("robots", [])
    if not robots_raw:
        raise ConfigError("No [[robots]] entries found in config.")

    robots = tuple(_parse_robot(r, i) for i, r in enumerate(robots_raw))
    damage = _parse_damage(raw.get("damage", {}))
    _check_removable_parts(robots, damage.removable_parts, "[damage].removable_parts")

    return RenderConfig(
        output=_parse_output(raw["output"]),
        robots=robots,
        materials={
            name: _parse_material(name, section)
            for name, section in raw.get("materials", {}).items()
        },
        distractors=_parse_distractors(raw.get("distractors", {})),
        environment=_parse_environment(raw.get("environment", {})),
        camera=_parse_camera(raw.get("camera", {})),
        scene=_parse_scene(raw.get("scene", {})),
        randomization=_parse_randomization(raw.get("randomization", {})),
        cages=_apply_only_cage(_parse_cages(raw.get("cages", [])), raw.get("only_cage")),
        damage=damage,
        resolver=resolver,
    )
