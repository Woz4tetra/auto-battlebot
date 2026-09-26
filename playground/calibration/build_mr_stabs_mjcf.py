"""Build the Mr Stabs Mk2 MJCF: CoACD collision pieces, the nose skid, and the model XML.

Reads the Onshape export and mass_properties.toml (from lump_mr_stabs_mass_properties.py) and
writes, under simulation/assets/robots/mr_stabs_mk2/:
    collision/hull_*.obj       convex pieces of the chassis and plates, body frame
    collision/collision.toml   piece list, nose skid point, rest pitch
    mr_stabs_mk2.xml           the model at the nominal PlantParams

    python playground/calibration/build_mr_stabs_mjcf.py
"""

from __future__ import annotations

import argparse
from pathlib import Path

import coacd
import numpy as np
import tomli_w
import trimesh

from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.mjcf import COLLISION_DIR, CollisionSet, build_mjcf
from auto_battlebot.mujoco_sim.onshape_export import (
    STRUCTURAL_LINKS,
    UrdfAssembly,
    body_from_root,
    load_mesh_in_root,
)


def structural_mesh(export_dir: Path) -> trimesh.Trimesh:
    assembly = UrdfAssembly(export_dir / "mr_stabs_mk2.urdf")
    rot, trans = body_from_root(assembly)
    to_body = np.eye(4)
    to_body[:3, :3], to_body[:3, 3] = rot, trans
    parts = []
    for link in STRUCTURAL_LINKS:
        for mesh, _ in load_mesh_in_root(assembly, export_dir / "meshes", link):
            parts.append(mesh.apply_transform(to_body))
    merged = trimesh.util.concatenate(parts)
    merged.merge_vertices()
    return merged


def rest_contact(vertices: np.ndarray, wheel_radius: float) -> tuple[np.ndarray, float]:
    """First vertex to touch the floor as the robot pitches nose-down about the axle.

    Axle at height `wheel_radius`, floor at z = -wheel_radius in the body frame. A positive
    rotation about +y lowers points ahead of the axle.
    """
    front = vertices[vertices[:, 0] > 0.0]
    best_theta, best_vertex = np.inf, front[0]
    for x, _, z in front:
        # -x sin(t) + z cos(t) = -r  ->  solve for the smallest t in [0, pi/2)
        amp = np.hypot(x, z)
        if amp < wheel_radius:
            continue
        # -x sin t + z cos t = amp cos(t - alpha) with alpha = atan2(-x, z)
        alpha = np.arctan2(-x, z)
        candidates = alpha + np.array([1.0, -1.0]) * np.arccos(-wheel_radius / amp)
        candidates = np.mod(candidates + np.pi, 2 * np.pi) - np.pi
        candidates = candidates[(candidates >= 0.0) & (candidates < np.pi / 2)]
        if len(candidates) and candidates.min() < best_theta:
            best_theta = float(candidates.min())
            best_vertex = np.array([x, 0.0, z])
    return best_vertex, best_theta


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--export-dir", type=Path, default=mass_properties.ASSET_DIR / "onshape_export"
    )
    parser.add_argument("--threshold", type=float, default=0.05, help="CoACD concavity")
    parser.add_argument("--max-pieces", type=int, default=16)
    args = parser.parse_args()

    mp = mass_properties.load()
    mesh = structural_mesh(args.export_dir)
    print(f"structural mesh: {len(mesh.vertices)} vertices, bounds {mesh.bounds.round(4).tolist()}")
    parts = coacd.run_coacd(
        coacd.Mesh(mesh.vertices, mesh.faces),
        threshold=args.threshold,
        max_convex_hull=args.max_pieces,
    )
    COLLISION_DIR.mkdir(parents=True, exist_ok=True)
    for old in COLLISION_DIR.glob("hull_*.obj"):
        old.unlink()
    names = []
    for i, (verts, faces) in enumerate(parts):
        piece = trimesh.Trimesh(np.asarray(verts), np.asarray(faces)).convex_hull
        name = f"hull_{i}.obj"
        piece.export(COLLISION_DIR / name)
        names.append(name)
    skid, pitch = rest_contact(np.asarray(mesh.vertices), mp.wheel_radius)
    print(
        f"{len(names)} convex pieces; nose skid at {skid.round(4)}, "
        f"rest pitch {np.degrees(pitch):.2f} deg"
    )
    meta = {
        "pieces": names,
        "skid_point_m": [round(float(v), 6) for v in skid],
        "rest_pitch_rad": round(pitch, 6),
        "coacd_threshold": args.threshold,
        "generator": "playground/calibration/build_mr_stabs_mjcf.py",
    }
    (COLLISION_DIR / "collision.toml").write_text(tomli_w.dumps(meta))

    collision = CollisionSet.load()
    # Mesh paths relative to the XML so the file is portable.
    xml = build_mjcf(mp, PlantParams(), collision).replace(str(COLLISION_DIR) + "/", "collision/")
    out = mass_properties.ASSET_DIR / "mr_stabs_mk2.xml"
    out.write_text(xml)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
