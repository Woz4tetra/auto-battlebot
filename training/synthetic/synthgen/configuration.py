"""Typed configuration parsed from config.toml (no Blender imports).

Every default that used to live in a scattered ``dict.get(key, default)`` call
is centralized here, with the same values the original code used. The TOML
schema itself is a frozen contract: keys are neither added nor renamed, and
unknown keys (e.g. ``randomization.light_color_temp_range``,
``environment.ground_textures``) are tolerated because sibling scripts consume
them.
"""

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import tomllib

from synthgen.annotations import normalize_annotation_mode
from synthgen.colorspec import ColorMappingEntry
from synthgen.constants import ANNOTATION_MODE_SEGMENTATION_BBOX
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
    )


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

    with open(resolved_config, "rb") as f:
        raw = tomllib.load(f)

    if "output" not in raw:
        raise ConfigError("missing required section [output]")
    robots_raw = raw.get("robots", [])
    if not robots_raw:
        raise ConfigError("No [[robots]] entries found in config.")

    return RenderConfig(
        output=_parse_output(raw["output"]),
        robots=tuple(_parse_robot(r, i) for i, r in enumerate(robots_raw)),
        materials={
            name: _parse_material(name, section)
            for name, section in raw.get("materials", {}).items()
        },
        distractors=_parse_distractors(raw.get("distractors", {})),
        environment=_parse_environment(raw.get("environment", {})),
        camera=_parse_camera(raw.get("camera", {})),
        scene=_parse_scene(raw.get("scene", {})),
        randomization=_parse_randomization(raw.get("randomization", {})),
        resolver=resolver,
    )
