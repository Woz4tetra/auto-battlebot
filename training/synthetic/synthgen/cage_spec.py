"""Cage geometry spec: the TOML schema, dotted overrides, and the primitives it expands to.

Pure module (no Blender). `cage.py` turns the boxes and cylinders returned here into mesh
objects; tests check the geometry without a renderer. All lengths are metres in the W frame:
mat centre at the origin, z up, +y away from the fixed camera, +x to its right. The mat top
surface is z = 0.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field, fields, is_dataclass, replace
from pathlib import Path
from typing import Any, get_args, get_origin, get_type_hints

if sys.version_info >= (3, 11):
    import tomllib
else:  # pragma: no cover - Blender ships 3.11, the dev venv 3.12
    import tomli as tomllib

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Box:
    name: str
    center: Vec3
    size: Vec3
    material: str
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class Cylinder:
    name: str
    center: Vec3
    radius: float
    length: float
    material: str
    axis: str = "z"


@dataclass(frozen=True)
class MatSpec:
    size: float = 2.35
    thickness: float = 0.012
    albedo: str = ""
    # The albedo image is a photo of the lit mat, brighter than the paint's real reflectance.
    # This linear multiplier darkens it so the lights, fitted by auto exposure to the mat, carry
    # the brightness and every true-albedo object (the robots) is lit to match.
    albedo_gain: float = 1.0
    roughness: float = 0.7
    specular: float = 0.3


@dataclass(frozen=True)
class CageSpec:
    interior: float = 2.4384
    wall_height: float = 1.22
    post_size: float = 0.05
    # Extra vertical posts along each side, as fractions of the side length from its centre
    # (0.0 is a mid-side post). Corner posts are always present.
    side_posts: tuple[float, ...] = ()


@dataclass(frozen=True)
class FrameSpec:
    height_above_mat: float = 0.05
    bolt_pitch: float = 0.20
    bolt_head_diameter: float = 0.019
    bolt_head_height: float = 0.008
    # "top" stands the bolt heads on the rail's upper face, which is what a low steel rail
    # shows. "inner" drives them horizontally through the face looking at the mat, which is
    # what a tall wooden kick rail shows; `bolt_height_above_mat` then says how far up.
    bolt_face: str = "top"
    bolt_height_above_mat: float = 0.05
    cc_texture: str = "Metal030"
    color: Vec3 = (0.02, 0.02, 0.022)
    roughness: float = 0.55


@dataclass(frozen=True)
class PanelSpec:
    thickness: float = 0.012
    ior: float = 1.586
    roughness: float = 0.03
    tint: Vec3 = (0.97, 0.97, 0.97)
    # Which walls to glaze. BlenderProc's segmentation stops at glass, so a wall that sits
    # between the camera and the mat costs every label behind it: leave it out when the
    # camera is outside the cage looking in.
    walls: tuple[str, ...] = ("near", "far", "left", "right")


@dataclass(frozen=True)
class PitSpec:
    """A rectangular hole through the mat: the mat is cut around it, walls and floor below.

    `center` and `size` are the rim in the W frame, the opening measured at mat level.
    The four walls sit just outside the rim so their inner faces land on the cut edge, and
    the floor closes the box off at `depth` so nothing under the arena shows through.
    """

    center: tuple[float, float] = (0.0, 0.0)
    size: tuple[float, float] = (0.43, 0.43)
    depth: float = 0.13
    wall_thickness: float = 0.02
    wall_cc_texture: str = "none"
    wall_color: Vec3 = (0.30, 0.22, 0.14)
    wall_roughness: float = 0.85
    floor_color: Vec3 = (0.015, 0.015, 0.015)
    floor_roughness: float = 0.9


@dataclass(frozen=True)
class VenueSpec:
    floor_drop: float = 0.45
    floor_extent: float = 12.0
    floor_cc_texture: str = "Concrete035"
    floor_color: Vec3 = (0.25, 0.24, 0.26)
    floor_roughness: float = 0.8


@dataclass(frozen=True)
class HouseBotBoxSpec:
    enabled: bool = True
    # The GLB is scaled so its footprint's longer side equals size[0] (0 keeps its own size);
    # size[2] is unused then.
    size: Vec3 = (0.45, 0.45, 0.30)
    position: tuple[float, float] = (-0.95, -0.95)
    yaw_deg: float = 30.0
    # Rotation about the model's own x axis (the LED-face normal) applied before yaw. The GLB
    # lies on its side: -90 stands it up with the eyes side by side and the mouth below.
    roll_deg: float = -90.0
    # A narrow spot on the camera side aimed at the box, so its vertical LED face is lit the
    # way the venue lights light the real one; 0 disables it.
    front_fill: float = 40.0
    # Textured model, repo-relative. Empty string falls back to a flat-coloured box.
    model: str = "training/data/environments/nhrl_3lb_cage/house_bot/HOUSE BOT.glb"
    texture_dir: str = "training/data/environments/nhrl_3lb_cage/house_bot"
    # Linear scale on the texture's grey body; 1.0 matches the mat brightness, as the real box does.
    albedo_gain: float = 1.0
    # The LED eyes and mouth in the texture glow: saturated texels emit at this strength.
    led_emission: float = 1.3
    color: Vec3 = (0.30, 0.55, 0.20)
    roughness: float = 0.6


@dataclass(frozen=True)
class WorldSpec:
    background_color: Vec3 = (0.01, 0.008, 0.015)
    ambient_strength: float = 1.0


@dataclass(frozen=True)
class TubeLight:
    position: Vec3 = (0.0, 0.0, 3.0)
    length: float = 1.2
    radius: float = 0.015
    color: Vec3 = (1.0, 1.0, 1.0)
    strength: float = 40.0
    axis: str = "x"


@dataclass(frozen=True)
class TubeGrid:
    """A rows x cols array of identical LED tubes, the ceiling rig as one knob.

    Shadow softness follows the rig's extent and tube size: more, longer, thicker tubes
    spread further apart give softer shadow edges. rows run along y, cols along x.
    """

    enabled: bool = True
    rows: int = 3
    cols: int = 4
    spacing_x: float = 0.9
    spacing_y: float = 1.0
    height: float = 2.6
    length: float = 2.2
    radius: float = 0.03
    color: Vec3 = (1.0, 1.0, 1.0)
    strength: float = 12.0
    axis: str = "x"


@dataclass(frozen=True)
class WashLight:
    position: Vec3 = (-4.0, 0.0, 2.5)
    look_at: Vec3 = (0.0, 0.0, 0.0)
    size: float = 1.5
    color: Vec3 = (0.8, 0.2, 1.0)
    strength: float = 300.0


@dataclass(frozen=True)
class BackdropSpec:
    radius: float = 6.0
    height: float = 4.0
    color: Vec3 = (0.03, 0.02, 0.04)
    strip_height: float = 2.6
    strip_thickness: float = 0.08
    strip_color: Vec3 = (0.7, 0.3, 1.0)
    strip_strength: float = 4.0


@dataclass(frozen=True)
class ExposureSpec:
    gain: float = 1.0
    color_gain: Vec3 = (1.0, 1.0, 1.0)


@dataclass(frozen=True)
class RenderSpec:
    samples: int = 256
    denoiser: str = "OPTIX"
    transmission_bounces: int = 8
    glossy_bounces: int = 4


@dataclass(frozen=True)
class LightsSpec:
    grid: TubeGrid = field(default_factory=TubeGrid)
    tubes: tuple[TubeLight, ...] = ()  # extra tubes beside the grid
    wash: tuple[WashLight, ...] = ()


@dataclass(frozen=True)
class CageSceneSpec:
    mat: MatSpec = field(default_factory=MatSpec)
    cage: CageSpec = field(default_factory=CageSpec)
    frame: FrameSpec = field(default_factory=FrameSpec)
    panel: PanelSpec = field(default_factory=PanelSpec)
    venue: VenueSpec = field(default_factory=VenueSpec)
    pits: tuple[PitSpec, ...] = ()
    house_bot_box: HouseBotBoxSpec = field(default_factory=HouseBotBoxSpec)
    world: WorldSpec = field(default_factory=WorldSpec)
    lights: LightsSpec = field(default_factory=LightsSpec)
    backdrop: BackdropSpec = field(default_factory=BackdropSpec)
    exposure: ExposureSpec = field(default_factory=ExposureSpec)
    render: RenderSpec = field(default_factory=RenderSpec)


# ----------------------------------------------------------------------------- loading


def _build(cls: Any, data: Any) -> Any:
    """Recursively build a frozen dataclass from a TOML table, rejecting unknown keys."""
    if not (isinstance(cls, type) and is_dataclass(cls)):
        raise TypeError(f"{cls} is not a dataclass")
    if not isinstance(data, dict):
        raise TypeError(f"expected a table for {cls.__name__}, got {type(data).__name__}")
    hints = get_type_hints(cls)
    known = {f.name for f in fields(cls)}
    unknown = set(data) - known
    if unknown:
        raise KeyError(f"{cls.__name__}: unknown keys {sorted(unknown)}; known {sorted(known)}")
    kwargs: dict[str, Any] = {}
    for f in fields(cls):
        if f.name not in data:
            continue
        value = data[f.name]
        hint = hints[f.name]
        if is_dataclass(hint):
            kwargs[f.name] = _build(hint, value)
        elif get_origin(hint) is tuple and get_args(hint) and is_dataclass(get_args(hint)[0]):
            kwargs[f.name] = tuple(_build(get_args(hint)[0], item) for item in value)
        elif get_origin(hint) is tuple:
            kwargs[f.name] = tuple(value)
        else:
            kwargs[f.name] = value
    return cls(**kwargs)


def apply_overrides(data: dict[str, Any], overrides: list[str]) -> dict[str, Any]:
    """Apply `section.key=value` overrides; values are parsed as TOML literals."""
    for item in overrides:
        if "=" not in item:
            raise ValueError(f"override {item!r} is not key=value")
        key, raw = item.split("=", 1)
        parsed = tomllib.loads(f"v = {raw.strip()}")["v"]
        parts = key.strip().split(".")
        node: Any = data
        for part in parts[:-1]:
            if part.isdigit():
                node = node[int(part)]
            else:
                node = node.setdefault(part, {})
        last = parts[-1]
        if last.isdigit():
            node[int(last)] = parsed
        else:
            node[last] = parsed
    return data


def load_cage_spec(path: Path, overrides: list[str] | None = None) -> CageSceneSpec:
    with path.open("rb") as handle:
        data = tomllib.load(handle)
    data = apply_overrides(data, list(overrides or []))
    spec: CageSceneSpec = _build(CageSceneSpec, data)
    return spec


def spec_to_dict(spec: Any) -> Any:
    """Plain nested dicts and lists, for render_meta.json."""
    if is_dataclass(spec) and not isinstance(spec, type):
        return {f.name: spec_to_dict(getattr(spec, f.name)) for f in fields(spec)}
    if isinstance(spec, tuple):
        return [spec_to_dict(v) for v in spec]
    return spec


def with_albedo(spec: CageSceneSpec, albedo: str) -> CageSceneSpec:
    return replace(spec, mat=replace(spec.mat, albedo=albedo))


# ---------------------------------------------------------------------------- geometry


def all_tubes(spec: CageSceneSpec) -> list[TubeLight]:
    """The tube grid expanded to individual tubes, plus any explicit `[[lights.tubes]]`."""
    grid = spec.lights.grid
    tubes: list[TubeLight] = []
    if grid.enabled and grid.rows > 0 and grid.cols > 0:
        for row in range(grid.rows):
            y = (row - (grid.rows - 1) / 2) * grid.spacing_y
            for col in range(grid.cols):
                x = (col - (grid.cols - 1) / 2) * grid.spacing_x
                tubes.append(
                    TubeLight(
                        (x, y, grid.height),
                        grid.length,
                        grid.radius,
                        grid.color,
                        grid.strength,
                        grid.axis,
                    )
                )
    tubes.extend(spec.lights.tubes)
    return tubes


Rect = tuple[float, float, float, float]  # x0, x1, y0, y1


def subtract_rects(rects: list[Rect], hole: Rect) -> list[Rect]:
    """Guillotine `hole` out of every rectangle, keeping the pieces axis-aligned.

    A rectangle that overlaps the hole becomes up to four pieces (below, above, left,
    right of it). Rectangles that miss the hole are passed through untouched.
    """
    out: list[Rect] = []
    hx0, hx1, hy0, hy1 = hole
    for x0, x1, y0, y1 in rects:
        if hx1 <= x0 or hx0 >= x1 or hy1 <= y0 or hy0 >= y1:
            out.append((x0, x1, y0, y1))
            continue
        cx0, cx1 = max(x0, hx0), min(x1, hx1)
        if y0 < hy0:
            out.append((x0, x1, y0, hy0))
        if hy1 < y1:
            out.append((x0, x1, hy1, y1))
        if x0 < cx0:
            out.append((x0, cx0, max(y0, hy0), min(y1, hy1)))
        if cx1 < x1:
            out.append((cx1, x1, max(y0, hy0), min(y1, hy1)))
    return [r for r in out if r[1] - r[0] > 1e-6 and r[3] - r[2] > 1e-6]


def pit_rects(spec: CageSceneSpec) -> list[Rect]:
    """Every pit opening as an (x0, x1, y0, y1) rectangle in W."""
    return [
        (
            pit.center[0] - pit.size[0] / 2,
            pit.center[0] + pit.size[0] / 2,
            pit.center[1] - pit.size[1] / 2,
            pit.center[1] + pit.size[1] / 2,
        )
        for pit in spec.pits
    ]


def _slab(name: str, rect: Rect, z_top: float, thickness: float, material: str) -> Box:
    x0, x1, y0, y1 = rect
    return Box(
        name,
        ((x0 + x1) / 2, (y0 + y1) / 2, z_top - thickness / 2),
        (x1 - x0, y1 - y0, thickness),
        material,
    )


def mat_boxes(spec: CageSceneSpec) -> list[Box]:
    """The mat, cut into pieces around any pits. One box when there are none."""
    m = spec.mat
    half = m.size / 2
    rects: list[Rect] = [(-half, half, -half, half)]
    for hole in pit_rects(spec):
        rects = subtract_rects(rects, hole)
    if len(rects) == 1:
        return [_slab("mat", rects[0], 0.0, m.thickness, "mat")]
    return [_slab(f"mat_{i}", rect, 0.0, m.thickness, "mat") for i, rect in enumerate(rects)]


def pit_boxes(spec: CageSceneSpec) -> list[Box]:
    """Four walls and a floor per pit, closing the hole off below the mat.

    The walls hang from the mat's underside, not from z = 0: a wall top flush with the floor
    surface is coplanar with the mat slab it sits under and z-fights in a ring around the
    opening. What shows above each wall is the mat's own cut edge, which is what the real
    plywood rim looks like. The walls run past the floor slab for the same reason.
    """
    out: list[Box] = []
    for i, (pit, rect) in enumerate(zip(spec.pits, pit_rects(spec))):
        x0, x1, y0, y1 = rect
        t = pit.wall_thickness
        top = -spec.mat.thickness
        height = pit.depth + 2 * t - spec.mat.thickness
        out += [
            _slab(f"pit{i}_wall_near", (x0 - t, x1 + t, y0 - t, y0), top, height, f"pit{i}_wall"),
            _slab(f"pit{i}_wall_far", (x0 - t, x1 + t, y1, y1 + t), top, height, f"pit{i}_wall"),
            _slab(f"pit{i}_wall_left", (x0 - t, x0, y0, y1), top, height, f"pit{i}_wall"),
            _slab(f"pit{i}_wall_right", (x1, x1 + t, y0, y1), top, height, f"pit{i}_wall"),
            _slab(
                f"pit{i}_floor",
                (x0 - t / 2, x1 + t / 2, y0 - t / 2, y1 + t / 2),
                -pit.depth,
                t,
                f"pit{i}_floor",
            ),
        ]
    return out


def rail_width(spec: CageSceneSpec) -> float:
    """The steel band between the mat edge and the wall plane."""
    return max(spec.cage.interior / 2 - spec.mat.size / 2, 0.02)


def frame_rails(spec: CageSceneSpec) -> list[Box]:
    """Four floor rails filling the gap between the mat and the wall plane."""
    width = rail_width(spec)
    inner = spec.mat.size / 2
    outer = inner + width
    height = spec.frame.height_above_mat + spec.mat.thickness
    z = (spec.frame.height_above_mat - spec.mat.thickness) / 2
    mid = (inner + outer) / 2
    length = 2 * outer
    return [
        Box("rail_near", (0.0, -mid, z), (length, width, height), "frame"),
        Box("rail_far", (0.0, mid, z), (length, width, height), "frame"),
        Box("rail_left", (-mid, 0.0, z), (width, length, height), "frame"),
        Box("rail_right", (mid, 0.0, z), (width, length, height), "frame"),
    ]


def bolt_positions(spec: CageSceneSpec) -> list[Cylinder]:
    """Bolt heads along every rail at the configured pitch, on the top or the inner face."""
    f = spec.frame
    if f.bolt_pitch <= 0:
        return []
    if f.bolt_face not in ("top", "inner"):
        raise ValueError(f"frame.bolt_face is {f.bolt_face!r}; use 'top' or 'inner'")
    half = spec.mat.size / 2
    count = int(2 * half // f.bolt_pitch)
    offsets = [(-count / 2 + 0.5 + i) * f.bolt_pitch for i in range(count)]
    if f.bolt_face == "top":
        mid = half + rail_width(spec) / 2
        z = f.height_above_mat + f.bolt_head_height / 2
        sides = [
            ("near", lambda a: (a, -mid, z), "z"),
            ("far", lambda a: (a, mid, z), "z"),
            ("left", lambda a: (-mid, a, z), "z"),
            ("right", lambda a: (mid, a, z), "z"),
        ]
    else:
        inset = half + f.bolt_head_height / 2
        z = f.bolt_height_above_mat
        sides = [
            ("near", lambda a: (a, -inset, z), "y"),
            ("far", lambda a: (a, inset, z), "y"),
            ("left", lambda a: (-inset, a, z), "x"),
            ("right", lambda a: (inset, a, z), "x"),
        ]
    bolts = []
    for i, along in enumerate(offsets):
        for side, place, axis in sides:
            bolts.append(
                Cylinder(
                    f"bolt_{side}_{i}",
                    place(along),
                    f.bolt_head_diameter / 2,
                    f.bolt_head_height,
                    "frame",
                    axis,
                )
            )
    return bolts


def posts(spec: CageSceneSpec) -> list[Box]:
    """Corner posts on the wall plane plus any configured mid-side posts."""
    c = spec.cage
    half = c.interior / 2 + c.post_size / 2
    z = c.wall_height / 2 - spec.mat.thickness
    size = (c.post_size, c.post_size, c.wall_height)
    out = [
        Box("post_corner_0", (-half, -half, z), size, "frame"),
        Box("post_corner_1", (half, -half, z), size, "frame"),
        Box("post_corner_2", (half, half, z), size, "frame"),
        Box("post_corner_3", (-half, half, z), size, "frame"),
    ]
    for i, fraction in enumerate(c.side_posts):
        along = fraction * c.interior
        out += [
            Box(f"post_near_{i}", (along, -half, z), size, "frame"),
            Box(f"post_far_{i}", (along, half, z), size, "frame"),
            Box(f"post_left_{i}", (-half, along, z), size, "frame"),
            Box(f"post_right_{i}", (half, along, z), size, "frame"),
        ]
    return out


def panels(spec: CageSceneSpec) -> list[Box]:
    """Polycarbonate walls on the interior plane, from the rail top to the wall height."""
    c, p = spec.cage, spec.panel
    z0 = spec.frame.height_above_mat
    z1 = c.wall_height
    height = z1 - z0
    z = (z0 + z1) / 2
    half = c.interior / 2 + p.thickness / 2
    length = c.interior
    built = {
        "near": Box("panel_near", (0.0, -half, z), (length, p.thickness, height), "panel"),
        "far": Box("panel_far", (0.0, half, z), (length, p.thickness, height), "panel"),
        "left": Box("panel_left", (-half, 0.0, z), (p.thickness, length, height), "panel"),
        "right": Box("panel_right", (half, 0.0, z), (p.thickness, length, height), "panel"),
    }
    unknown = set(p.walls) - set(PANEL_WALLS)
    if unknown:
        raise ValueError(f"panel.walls has unknown walls {sorted(unknown)}; use {sorted(built)}")
    return [built[name] for name in built if name in p.walls]


PANEL_WALLS: tuple[str, ...] = ("near", "far", "left", "right")


def panels_outside_camera(spec: CageSceneSpec, camera_xy: tuple[float, float]) -> tuple[str, ...]:
    """Which walls the camera stands outside of, so their glass is between it and the mat.

    One-way glass: a pane the camera looks through from outside the cage is hidden, because
    BlenderProc's segmentation stops at glass and everything behind it loses its label. Panes
    the camera sees from inside, across the arena, are untouched: they are what the real
    picture shows on the far side.

    The test is the wall plane, not the line of sight. A camera outside a wall but aimed away
    from it does not see that pane, and hiding it changes nothing.
    """
    half = spec.cage.interior / 2
    x, y = camera_xy
    outside = []
    if y < -half:
        outside.append("near")
    if y > half:
        outside.append("far")
    if x < -half:
        outside.append("left")
    if x > half:
        outside.append("right")
    return tuple(name for name in PANEL_WALLS if name in outside)


def venue_floor(spec: CageSceneSpec) -> Box:
    v = spec.venue
    return Box(
        "venue_floor",
        (0.0, 0.0, -v.floor_drop - 0.01),
        (v.floor_extent, v.floor_extent, 0.02),
        "venue_floor",
    )


def stage_riser_boxes(spec: CageSceneSpec) -> list[Box]:
    """The riser, cut around the pits so a hole in the mat is not filled in from below."""
    height = spec.venue.floor_drop - spec.mat.thickness
    outer = spec.cage.interior + 2 * spec.panel.thickness + 2 * spec.cage.post_size
    half = outer / 2
    rects: list[Rect] = [(-half, half, -half, half)]
    for pit, hole in zip(spec.pits, pit_rects(spec)):
        # Clear of the pit walls, not flush with them: coincident faces z-fight.
        margin = pit.wall_thickness * 1.5
        rects = subtract_rects(
            rects, (hole[0] - margin, hole[1] + margin, hole[2] - margin, hole[3] + margin)
        )
    if len(rects) == 1:
        return [_slab("riser", rects[0], -spec.mat.thickness, height, "frame")]
    return [
        _slab(f"riser_{i}", rect, -spec.mat.thickness, height, "frame")
        for i, rect in enumerate(rects)
    ]


def house_bot_box(spec: CageSceneSpec) -> Box | None:
    h = spec.house_bot_box
    if not h.enabled:
        return None
    return Box(
        "house_bot_box",
        (h.position[0], h.position[1], h.size[2] / 2),
        h.size,
        "house_bot",
        h.yaw_deg,
    )


def mat_image_uv(x_w: float, y_w: float, mat_size: float) -> tuple[float, float]:
    """Image UV (Blender convention, v up) of a W-frame point on the mat, for the albedo built by
    training/synthetic/build_cage_floor_texture.py.

    That texture is laid out in the hfield frame: image column along hfield +x, image row along
    hfield +y, origin at the (-half, -half) corner. With hfield x = -W y and hfield y = W x, the
    column fraction is 0.5 - y/size and the row fraction is 0.5 + x/size; Blender's v runs up
    the image, so v = 1 - row.
    """
    u = 0.5 - y_w / mat_size
    v = 1.0 - (0.5 + x_w / mat_size)
    return u, v
