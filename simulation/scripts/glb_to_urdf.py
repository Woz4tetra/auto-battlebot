#!/usr/bin/env python3
"""Convert a robot GLB model + physical properties into a URDF package.

Reads a TOML config describing the source GLB, physical properties, and
wheel extraction patterns.  Outputs a URDF file together with OBJ mesh
files for chassis and wheels.

Usage:
    python glb_to_urdf.py robot_config.toml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from xml.dom.minidom import parseString
from xml.etree.ElementTree import Element, SubElement, tostring

import numpy as np
import trimesh

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]


# ---------------------------------------------------------------------------
# Inertia helpers
# ---------------------------------------------------------------------------


def _parse_inertia(flat: list[float]) -> np.ndarray:
    """Convert a flat 9-element list (row-major) to a 3x3 matrix."""
    return np.array(flat, dtype=np.float64).reshape(3, 3)


def _rot_z(angle_deg: float) -> np.ndarray:
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rot_x(angle_deg: float) -> np.ndarray:
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rotate_inertia(I: np.ndarray, R: np.ndarray) -> np.ndarray:
    return R @ I @ R.T


def _inertia_g_mm2_to_kg_m2(I: np.ndarray) -> np.ndarray:
    return I * 1e-9


# ---------------------------------------------------------------------------
# Mesh extraction
# ---------------------------------------------------------------------------


def _export_obj(mesh: trimesh.Trimesh | trimesh.Scene, obj_path: Path) -> int:
    """Export a mesh or scene to OBJ with a unique MTL filename.

    Returns the total face count.
    """
    mesh.export(str(obj_path))
    mtl_generic = obj_path.parent / "material.mtl"
    if mtl_generic.exists():
        mtl_unique = obj_path.with_suffix(".mtl")
        mtl_generic.rename(mtl_unique)
        obj_text = obj_path.read_text()
        obj_path.write_text(obj_text.replace("material.mtl", mtl_unique.name, 1))

    if isinstance(mesh, trimesh.Scene):
        return sum(len(g.faces) for g in mesh.geometry.values())
    return len(mesh.faces)


def _matches(name: str, patterns: list[str]) -> bool:
    return any(p in name for p in patterns)


def _collect_meshes(
    scene: trimesh.Scene,
    predicate,
) -> list[trimesh.Trimesh]:
    """Return world-space copies of geometry nodes that satisfy *predicate*."""
    parts: list[trimesh.Trimesh] = []
    for node_name in scene.graph.nodes_geometry:
        if not predicate(node_name):
            continue
        transform, geom_name = scene.graph[node_name]
        geom = scene.geometry[geom_name]
        if not isinstance(geom, trimesh.Trimesh):
            continue
        m = geom.copy()
        m.apply_transform(transform)
        parts.append(m)
    return parts


def _extract_one_wheel(
    scene: trimesh.Scene,
    patterns: list[str],
    target_y: float,
    tol: float = 0.02,
) -> trimesh.Trimesh | None:
    """Extract wheel meshes near *target_y*, centered at origin."""

    def pred(name: str) -> bool:
        if not _matches(name, patterns):
            return False
        transform, _ = scene.graph[name]
        return abs(transform[1, 3] - target_y) < tol

    parts = _collect_meshes(scene, pred)
    if not parts:
        return None
    merged = trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]
    merged.apply_translation([0, -target_y, 0])
    return merged


def _extract_chassis(
    scene: trimesh.Scene,
    wheel_patterns: list[str],
    delete_patterns: list[str],
) -> trimesh.Scene | None:
    """Extract chassis parts as a Scene, preserving per-part materials/textures."""
    result = trimesh.Scene()
    idx = 0
    for node_name in scene.graph.nodes_geometry:
        if wheel_patterns and _matches(node_name, wheel_patterns):
            continue
        if delete_patterns and _matches(node_name, delete_patterns):
            continue
        transform, geom_name = scene.graph[node_name]
        geom = scene.geometry[geom_name]
        if not isinstance(geom, trimesh.Trimesh):
            continue
        m = geom.copy()
        m.apply_transform(transform)
        result.add_geometry(m, geom_name=f"chassis_{idx}")
        idx += 1
    if idx == 0:
        return None
    return result


# ---------------------------------------------------------------------------
# URDF generation
# ---------------------------------------------------------------------------


def _fmt(v: float) -> str:
    return f"{v:.10e}"


def _xyz(vals: list[float] | tuple[float, ...]) -> str:
    return " ".join(_fmt(v) for v in vals)


def _add_inertial(
    parent: Element,
    mass: float,
    com: list[float],
    I: np.ndarray,
) -> None:
    inertial = SubElement(parent, "inertial")
    SubElement(inertial, "origin", xyz=_xyz(com), rpy="0 0 0")
    SubElement(inertial, "mass", value=_fmt(mass))
    SubElement(
        inertial,
        "inertia",
        **{
            "ixx": _fmt(I[0, 0]),
            "ixy": _fmt(I[0, 1]),
            "ixz": _fmt(I[0, 2]),
            "iyy": _fmt(I[1, 1]),
            "iyz": _fmt(I[1, 2]),
            "izz": _fmt(I[2, 2]),
        },
    )


def _add_visual(parent: Element, mesh_path: str) -> None:
    visual = SubElement(parent, "visual")
    geom = SubElement(visual, "geometry")
    SubElement(geom, "mesh", filename=mesh_path)


def _add_collision_mesh(parent: Element, mesh_path: str) -> None:
    collision = SubElement(parent, "collision")
    geom = SubElement(collision, "geometry")
    SubElement(geom, "mesh", filename=mesh_path)


def _add_collision_cylinder(
    parent: Element,
    radius: float,
    length: float,
) -> None:
    collision = SubElement(parent, "collision")
    origin = SubElement(collision, "origin", xyz="0 0 0", rpy=f"{np.pi / 2} 0 0")
    geom = SubElement(collision, "geometry")
    SubElement(geom, "cylinder", radius=_fmt(radius), length=_fmt(length))


def _add_wheel_link(
    robot: Element,
    link_name: str,
    mass: float,
    com: list[float],
    I: np.ndarray,
    wheel_radius: float,
    wheel_width: float,
    visual_mesh: str | None,
) -> None:
    link = SubElement(robot, "link", name=link_name)
    _add_inertial(link, mass, com, I)
    if visual_mesh:
        _add_visual(link, visual_mesh)
    _add_collision_cylinder(link, wheel_radius, wheel_width)


def _add_wheel_joint(
    robot: Element,
    joint_name: str,
    parent_link: str,
    child_link: str,
    origin_xyz: list[float],
) -> None:
    joint = SubElement(robot, "joint", name=joint_name, type="continuous")
    SubElement(joint, "parent", link=parent_link)
    SubElement(joint, "child", link=child_link)
    SubElement(joint, "origin", xyz=_xyz(origin_xyz), rpy="0 0 0")
    SubElement(joint, "axis", xyz="0 1 0")
    SubElement(joint, "dynamics", damping="0.01", friction="0.0")


def _pretty_xml(root: Element) -> str:
    raw = tostring(root, encoding="unicode")
    return parseString(raw).toprettyxml(indent="  ", encoding=None)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="Robot TOML config file")
    args = parser.parse_args()

    cfg_path: Path = args.config.resolve()
    if not cfg_path.exists():
        print(f"Error: {cfg_path} not found", file=sys.stderr)
        sys.exit(1)

    with open(cfg_path, "rb") as f:
        cfg = tomllib.load(f)

    cfg_dir = cfg_path.parent

    # -- resolve paths -------------------------------------------------------
    robot_cfg = cfg["robot"]
    source_path = (cfg_dir / robot_cfg["source"]).resolve()
    output_dir = (cfg_dir / robot_cfg["output_dir"]).resolve()
    mesh_dir = output_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    robot_name = robot_cfg["name"]
    mesh_cfg = cfg.get("mesh", {})
    chassis_cfg = cfg["chassis"]
    wheels_cfg = cfg["wheels"]

    delete_patterns: list[str] = mesh_cfg.get("delete_patterns", [])
    wheel_patterns: list[str] = wheels_cfg.get("extract_patterns", [])
    layout: str = wheels_cfg.get("layout", "differential")
    add_cylinder: bool = wheels_cfg.get("add_cylinder", False)

    wheel_radius = wheels_cfg["diameter"] / 2.0
    track_width = wheels_cfg["track_width"]
    half_track = track_width / 2.0
    wheel_width_cfg = wheels_cfg.get("cylinder_width", 0.012)
    wheelbase = wheels_cfg.get("wheelbase", 0.0)
    half_wheelbase = wheelbase / 2.0

    # -- load source model ---------------------------------------------------
    print(f"Loading {source_path}...")
    scene = trimesh.load(str(source_path))
    if not isinstance(scene, trimesh.Scene):
        if isinstance(scene, trimesh.Trimesh):
            s = trimesh.Scene()
            s.add_geometry(scene, geom_name=robot_name)
            scene = s
        else:
            print("Error: could not load as Scene or Trimesh", file=sys.stderr)
            sys.exit(1)

    # -- chassis mesh --------------------------------------------------------
    print("Extracting chassis mesh...")
    chassis_mesh = _extract_chassis(scene, wheel_patterns, delete_patterns)
    if chassis_mesh is None:
        print("Error: no chassis geometry found", file=sys.stderr)
        sys.exit(1)

    chassis_obj = mesh_dir / "chassis.obj"
    n_faces = _export_obj(chassis_mesh, chassis_obj)
    print(f"  Saved {chassis_obj} ({n_faces:,} faces)")

    # Convex hull for collision (physics engines need convex geometry)
    all_verts = np.vstack([g.vertices for g in chassis_mesh.geometry.values()])
    chassis_hull = trimesh.convex.convex_hull(trimesh.Trimesh(vertices=all_verts))
    collision_obj = mesh_dir / "chassis_collision.obj"
    _export_obj(chassis_hull, collision_obj)
    print(f"  Saved {collision_obj} ({len(chassis_hull.faces):,} faces, convex hull)")

    # -- wheel mesh(es) ------------------------------------------------------
    wheel_visual_path: str | None = None

    if wheel_patterns and not add_cylinder:
        print("Extracting wheel mesh...")
        left_y = half_track
        wheel_mesh = _extract_one_wheel(scene, wheel_patterns, left_y)
        if wheel_mesh is None:
            right_y = -half_track
            wheel_mesh = _extract_one_wheel(scene, wheel_patterns, right_y)
        if wheel_mesh is None:
            print("Warning: could not extract wheel mesh; using cylinder only")
        else:
            wheel_obj = mesh_dir / "wheel.obj"
            wn_faces = _export_obj(wheel_mesh, wheel_obj)
            wheel_visual_path = "meshes/wheel.obj"
            print(f"  Saved {wheel_obj} ({wn_faces:,} faces)")

            # Derive wheel width from mesh extents along Y
            wheel_width_cfg = wheel_mesh.extents[1]
            print(f"  Wheel width from mesh: {wheel_width_cfg:.4f} m")
    else:
        print("No wheel extraction patterns; wheels will be cylinders only.")

    # -- compute inertia tensors ---------------------------------------------
    # Chassis
    chassis_I_raw = _parse_inertia(chassis_cfg["inertia"])
    chassis_I = _inertia_g_mm2_to_kg_m2(chassis_I_raw)
    chassis_rot_deg = chassis_cfg.get("inertia_rotation_deg", 0.0)
    if chassis_rot_deg != 0.0:
        R = _rot_z(chassis_rot_deg)
        chassis_I = _rotate_inertia(chassis_I, R)
    chassis_com = chassis_cfg["com"]
    chassis_mass = chassis_cfg["mass"]

    # Wheels — rotate so Z (spin axis) maps to Y (URDF joint axis)
    wheel_I_raw = _parse_inertia(wheels_cfg["inertia"])
    wheel_I = _inertia_g_mm2_to_kg_m2(wheel_I_raw)
    wheel_I = _rotate_inertia(wheel_I, _rot_x(90.0))
    wheel_com = wheels_cfg.get("com_offset", [0.0, 0.0, 0.0])
    wheel_mass = wheels_cfg["mass"]

    print(f"\nChassis inertia (kg m^2):\n{chassis_I}")
    print(f"Wheel inertia (kg m^2):\n{wheel_I}")

    # -- build URDF ----------------------------------------------------------
    robot = Element("robot", name=robot_name)

    # Base link
    base = SubElement(robot, "link", name="base_link")
    _add_inertial(base, chassis_mass, chassis_com, chassis_I)
    _add_visual(base, "meshes/chassis.obj")
    _add_collision_mesh(base, "meshes/chassis_collision.obj")

    # Wheel definitions
    if layout == "differential":
        wheel_defs = [
            ("left_wheel", "left_wheel_joint", [0.0, half_track, 0.0]),
            ("right_wheel", "right_wheel_joint", [0.0, -half_track, 0.0]),
        ]
    elif layout == "skid_steer":
        wheel_defs = [
            (
                "front_left_wheel",
                "front_left_wheel_joint",
                [half_wheelbase, half_track, 0.0],
            ),
            (
                "front_right_wheel",
                "front_right_wheel_joint",
                [half_wheelbase, -half_track, 0.0],
            ),
            (
                "rear_left_wheel",
                "rear_left_wheel_joint",
                [-half_wheelbase, half_track, 0.0],
            ),
            (
                "rear_right_wheel",
                "rear_right_wheel_joint",
                [-half_wheelbase, -half_track, 0.0],
            ),
        ]
    else:
        print(f"Error: unknown layout '{layout}'", file=sys.stderr)
        sys.exit(1)

    for link_name, joint_name, origin in wheel_defs:
        _add_wheel_link(
            robot,
            link_name,
            wheel_mass,
            wheel_com,
            wheel_I,
            wheel_radius,
            wheel_width_cfg,
            wheel_visual_path,
        )
        _add_wheel_joint(robot, joint_name, "base_link", link_name, origin)

    # Write URDF
    urdf_path = output_dir / "robot.urdf"
    xml_str = _pretty_xml(robot)
    urdf_path.write_text(xml_str)
    print(f"\nWrote {urdf_path}")

    # Summary
    print(f"\nURDF package: {output_dir}/")
    print("  robot.urdf")
    print("  meshes/chassis.obj (visual)")
    print("  meshes/chassis_collision.obj (collision, convex hull)")
    if wheel_visual_path:
        print("  meshes/wheel.obj")
    print(f"\nLayout: {layout}, wheels: {len(wheel_defs)}")
    print(f"  track_width={track_width}, wheel_radius={wheel_radius}")
    if layout == "skid_steer":
        print(f"  wheelbase={wheelbase}")


if __name__ == "__main__":
    main()
