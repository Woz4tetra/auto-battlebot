"""Build the Mrs Buff Mk3 model: mass properties, collision pieces, and the MJCF.

Writes, under simulation/assets/robots/mrs_buff_mk3/:
    mass_properties.toml       lumped bodies, wheel geometry, drivetrain, weapon sweep
    collision/hull_*.obj       convex pieces of the chassis structure, body frame
    collision/disk_*.obj       convex pieces of the weapon disk, body frame
    collision/collision.toml   piece list
    mrs_buff_mk3.xml           the model at the nominal PlantParams

The body frame is FLU at the footprint centre, at axle height: 15.75 mm ahead of the midpoint
between the axles. The Onshape assembly has +X to the robot's left and -Y forward, so
x = -Y_asm, y = +X_asm, z = +Z_asm, the same rotation as Mr Stabs Mk2's export.

Mass properties come from Onshape's Mass and section properties panel, not the URDF: the URDF
export sums to 708 g against the assembly's 1,332 g, because the ESCs, batteries, wheel cores and
weapon motor export at zero mass and the drive motors are not in it at all. The panel reports the
inertia tensor about the reference mate connector's origin, not about the COM, so each tensor is
shifted to its COM here (parallel axis). The wheel was measured against a connector on its own
axle, which keeps that shift small; the screenshots are in onshape_export/.

The weapon does not spin in this model: the disk and its rotating parts are lumped into the
chassis, and its CoACD pieces collide as chassis geometry. Its sweep (the cylinder the disk fills
when spinning) is recorded for the strike reward.

The drive motors (Blitz Pro 24 mm, 650 KV, 5.2:1) turn the back wheels; 120XL belts turn the front
wheels 1:1. No rotor inertia is published for the 2306 motor: 3.0e-6 kg m^2 is Mr Stabs Mk2's
CAD-integrated 1806 rotor (9.3e-7) scaled by bell mass and radius, and the fit's armature scale
absorbs the error.

    python playground/calibration/build_mrs_buff_mk3_model.py            # reuse collision pieces
    python playground/calibration/build_mrs_buff_mk3_model.py --split    # rerun CoACD
"""

from __future__ import annotations

import argparse

import numpy as np
import tomli_w
import trimesh

from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.mjcf import CollisionSet, build_mjcf
from auto_battlebot.mujoco_sim.onshape_export import (
    MassBody,
    UrdfAssembly,
    load_mesh_in_root,
    subtract,
)

ASSET_DIR = mass_properties.ROBOTS_DIR / "mrs_buff_mk3"
EXPORT_DIR = ASSET_DIR / "onshape_export"
COLLISION_DIR = ASSET_DIR / "collision"

STRUCTURAL_LINKS = (
    "mbmk3_chassis",
    "mbmk3_top_plate",
    "mbmk3_bottom_plate",
    "mbmk3_left_wheel_guard",
    "mbmk3_right_wheel_guard",
    "weapon_module_left_plate",
    "weapon_module_right_plate",
    "weapon_module_motor_mount",
    "mbmk3_weapon_module_hat",
    "weapon_belt_guard",
)
DISK_LINK = "mbmk3_single_point_weapon_disk"
TREAD_LINKS = tuple(f"mbmk3_integrated_pulley_wheel_tread{s}" for s in ("", "_1", "_2", "_3"))
COACD_THRESHOLD = 0.05
COACD_MAX_PIECES = {"hull": 24, "disk": 6}

# Assembly -> body. The footprint centre sits at Y_asm = 80.29 mm, at the axle height Z_asm = -30.5.
R_BODY_ASM = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
ORIGIN_ASM_M = np.array([0.0, 0.08029, -0.0305])

MM = 1e-3
KG_MM2 = 1e-6


def _shift(mass: float, r: np.ndarray) -> np.ndarray:
    """Parallel-axis term: the tensor of a point mass at r, about the origin."""
    return mass * (float(r @ r) * np.eye(3) - np.outer(r, r))


def _about_com(mass: float, com_mm: list[float], tensor_kg_mm2: list[list[float]]) -> MassBody:
    """An Onshape panel reading (tensor about the connector origin) as a body about its COM."""
    com = np.asarray(com_mm) * MM
    tensor = np.asarray(tensor_kg_mm2) * KG_MM2
    return MassBody(mass, com, tensor - _shift(mass, com))


# Onshape panel readings against the footprint-centre connector (x forward, y left, z up).
# Screenshot 2026-10-25 10:24: the whole assembly.
FULL = _about_com(
    1.332,
    [11.515, -0.228, 9.067],
    [[4566.754, -28.422, -752.176], [-28.422, 5043.084, -2.353], [-752.176, -2.353, 8475.613]],
)
# Screenshot 2026-10-25 10:23: the disk and every part that turns with it.
WEAPON = _about_com(
    0.249,
    [80.292, 0.152, 30.5],
    [[307.865, -3.025, -570.038], [-3.025, 1973.767, -1.149], [-570.038, -1.149, 1684.82]],
)
# Screenshot 2026-10-25 10:35: one wheel (tread, core, hub plate, clamp hub, two M3 screws)
# against a connector on its axle, z along the axle; the four wheels are identical. The front
# wheels' 6 mm shoulder screws are axles that do not turn, so they stay in the chassis.
WHEEL_MASS = 0.023
WHEEL_CONNECTOR_COM_MM = np.array([-0.082, -0.028, -14.009])
WHEEL_CONNECTOR_TENSOR = np.array(
    [[8.466, 0.003, -0.006], [0.003, 8.373, -0.002], [-0.006, -0.002, 5.793]]
)
# A wheel COM in the footprint frame (screenshot 2026-10-25 10:21, back right wheel, same parts).
WHEEL_COM_BACK_RIGHT_MM = np.array([-71.918, -97.009, 0.028])


def wheel_bodies(axle_x: float, follower_x: float) -> dict[str, MassBody]:
    tensor = WHEEL_CONNECTOR_TENSOR * KG_MM2 - _shift(WHEEL_MASS, WHEEL_CONNECTOR_COM_MM * MM)
    # Connector axes (x, y, axle) to body axes (x, axle = body y, z): swap the last two.
    swap = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
    inertia = swap @ tensor @ swap.T
    lateral = abs(WHEEL_COM_BACK_RIGHT_MM[1]) * MM
    out = {}
    for name, x, y in (
        ("back_left", axle_x, lateral),
        ("back_right", axle_x, -lateral),
        ("front_left", follower_x, lateral),
        ("front_right", follower_x, -lateral),
    ):
        out[name] = MassBody(WHEEL_MASS, np.array([x, y, 0.0]), inertia.copy())
    return out


def _full_inertia(t: np.ndarray) -> list[float]:
    """MJCF fullinertia order: Ixx Iyy Izz Ixy Ixz Iyz."""
    return [float(v) for v in (t[0, 0], t[1, 1], t[2, 2], t[0, 1], t[0, 2], t[1, 2])]


def _table(body: MassBody) -> dict[str, object]:
    return {
        "mass_kg": round(float(body.mass), 7),
        "com_m": [round(float(v), 7) for v in body.com],
        "fullinertia_kg_m2": [float(f"{v:.6g}") for v in _full_inertia(body.inertia)],
    }


def to_body(assembly: UrdfAssembly, links: tuple[str, ...]) -> trimesh.Trimesh:
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = R_BODY_ASM, -R_BODY_ASM @ ORIGIN_ASM_M
    parts = []
    for link in links:
        for mesh, _ in load_mesh_in_root(assembly, EXPORT_DIR / "meshes", link):
            parts.append(mesh.apply_transform(transform))
    merged = trimesh.util.concatenate(parts)
    merged.merge_vertices()
    return merged


def geometry(assembly: UrdfAssembly) -> dict[str, float]:
    """Wheel and axle geometry from the tread meshes, in the body frame."""
    treads = [to_body(assembly, (t,)) for t in TREAD_LINKS]
    centres = np.array([t.bounds.mean(axis=0) for t in treads])
    sizes = np.array([np.ptp(t.bounds, axis=0) for t in treads])
    xs = np.unique(np.round(centres[:, 0], 4))
    if len(xs) != 2:
        raise ValueError(f"expected two axles, found tread centres at x = {xs}")
    return {
        "axle_x_m": round(float(xs.min()), 5),  # driven: the back pair
        "follower_axle_x_m": round(float(xs.max()), 5),
        "track_half_width_m": round(float(np.abs(centres[:, 1]).mean()), 5),
        "wheel_radius_m": round(float(sizes[:, 2].mean() / 2), 5),
        "tread_width_m": round(float(sizes[:, 1].mean()), 5),
    }


def weapon_sweep(assembly: UrdfAssembly) -> dict[str, list[float] | float]:
    """The disk's spin axis from the URDF joint, and the cylinder its vertices sweep about it."""
    pose = assembly.link_pose("mbmk3_weapon_rotor_pulley")
    axis_point = R_BODY_ASM @ (pose[:3, 3] - ORIGIN_ASM_M)
    axis = R_BODY_ASM @ pose[:3, :3] @ np.array([1.0, 0.0, 0.0])
    if abs(abs(axis[1]) - 1.0) > 1e-6:
        raise ValueError(f"weapon axis {axis} is not along body y")
    disk = to_body(assembly, (DISK_LINK,))
    v = np.asarray(disk.vertices)
    radius = float(np.hypot(v[:, 0] - axis_point[0], v[:, 2] - axis_point[2]).max())
    y_lo, y_hi = float(v[:, 1].min()), float(v[:, 1].max())
    return {
        "sweep_center_m": [
            round(float(axis_point[0]), 5),
            round(0.5 * (y_lo + y_hi), 5),
            round(float(axis_point[2]), 5),
        ],
        "sweep_radius_m": round(radius, 5),
        "sweep_half_width_m": round(0.5 * (y_hi - y_lo), 5),
    }


def trim_to_cad_floor(assembly: UrdfAssembly) -> float:
    """Cut every piece off at the structure's lowest CAD point.

    CoACD hulls run up to a millimetre outside the mesh. The fork tips sit 0.35 mm above the
    floor in CAD, so an untrimmed hull reaches under it and the robot rests on its forks with the
    front wheels in the air. Idempotent.
    """
    floor = float(to_body(assembly, STRUCTURAL_LINKS).bounds[0, 2])
    for path in COLLISION_DIR.glob("*.obj"):
        piece = trimesh.load(path, force="mesh")
        v = np.asarray(piece.vertices).copy()
        if v[:, 2].min() < floor:
            v[:, 2] = np.maximum(v[:, 2], floor)
            trimesh.Trimesh(v).convex_hull.export(path)
    return floor


def split(assembly: UrdfAssembly) -> list[str]:
    import coacd

    COLLISION_DIR.mkdir(parents=True, exist_ok=True)
    for old in COLLISION_DIR.glob("*.obj"):
        old.unlink()
    names = []
    for prefix, links in (("hull", STRUCTURAL_LINKS), ("disk", (DISK_LINK,))):
        mesh = to_body(assembly, links)
        parts = coacd.run_coacd(
            coacd.Mesh(mesh.vertices, mesh.faces),
            threshold=COACD_THRESHOLD,
            max_convex_hull=COACD_MAX_PIECES[prefix],
        )
        for i, (verts, faces) in enumerate(parts):
            name = f"{prefix}_{i}.obj"
            trimesh.Trimesh(np.asarray(verts), np.asarray(faces)).convex_hull.export(
                COLLISION_DIR / name
            )
            names.append(name)
    return names


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--split", action="store_true", help="rerun CoACD on the structure")
    args = parser.parse_args()

    assembly = UrdfAssembly(EXPORT_DIR / "mrs_buff_mk3.urdf")
    geo = geometry(assembly)
    sweep = weapon_sweep(assembly)
    wheels = wheel_bodies(geo["axle_x_m"], geo["follower_axle_x_m"])
    chassis = subtract(FULL, list(wheels.values()))  # the weapon stays in: it does not spin
    print(f"geometry {geo}")
    print(f"weapon sweep {sweep}")
    print(
        f"chassis {chassis.mass:.4f} kg (weapon {WEAPON.mass:.3f} kg lumped in), "
        f"COM {np.round(chassis.com / MM, 2)} mm"
    )

    if args.split:
        pieces = split(assembly)
    else:
        pieces = sorted(
            (p.name for p in COLLISION_DIR.glob("*.obj")),
            key=lambda n: (n.split("_")[0], int(n.split("_")[1].split(".")[0])),
        )
    floor = trim_to_cad_floor(assembly)
    print(f"pieces trimmed at the CAD floor, z = {floor * 1000:.2f} mm")
    meta = {
        "pieces": pieces,
        "rest_pitch_rad": 0.0,
        "coacd_threshold": COACD_THRESHOLD,
        "coacd_max_pieces": COACD_MAX_PIECES,
        "note": "hull_* are the chassis structure, disk_* the weapon disk (approved 2026-10-25)",
        "trimmed_at_z_m": round(floor, 6),
        "generator": "playground/calibration/build_mrs_buff_mk3_model.py",
    }
    (COLLISION_DIR / "collision.toml").write_text(tomli_w.dumps(meta))

    kv = 650.0
    doc = {
        "provenance": {
            "onshape_assembly": (
                "Onshape Mass and section properties against the footprint-centre connector, "
                "screenshots 2026-10-25 (onshape_export/*.png); tensors shifted to each COM"
            ),
            "urdf_export": (
                "onshape_export/mrs_buff_mk3.urdf (Onshape URDF exporter 1.222); geometry, "
                "weapon axis and collision meshes only, it carries no mass overrides"
            ),
            "chassis": "whole assembly minus four wheels; the weapon is lumped in (not spinning)",
            "scale_mass_kg": "not weighed; the CAD total stands",
            "motor": "Blitz Pro 24 mm 650 KV 5.2:1, justcuzrobotics.com product page and datasheet",
            "rotor": "estimate: Mr Stabs Mk2's 1806 rotor scaled to a 2306 bell; fit absorbs it",
            "generator": "playground/calibration/build_mrs_buff_mk3_model.py",
        },
        "frame": {
            "description": (
                "FLU at the footprint centre, at axle height: x = -Y_asm, y = +X_asm, z = +Z_asm"
            ),
        },
        "chassis": _table(chassis),
        "wheel_left": _table(wheels["back_left"]),
        "wheel_right": _table(wheels["back_right"]),
        "weapon": {**sweep, **{f"body_{k}": v for k, v in _table(WEAPON).items()}},
        "geometry": geo,
        "drivetrain": {
            "gear_ratio": 5.2,
            "motor_kv_rpm_per_v": kv,
            "motor_kt_nm_per_a": round(60.0 / (2.0 * np.pi * kv), 7),
            "rotor_inertia_kg_m2": 3.0e-6,
        },
    }
    (ASSET_DIR / "mass_properties.toml").write_text(
        "# Mrs Buff Mk3 mass properties. Generated; do not edit by hand.\n"
        "# Regenerate: python playground/calibration/build_mrs_buff_mk3_model.py\n"
        + tomli_w.dumps(doc)
    )

    mp = mass_properties.load(ASSET_DIR / "mass_properties.toml")
    collision = CollisionSet.load(COLLISION_DIR)
    xml = build_mjcf(mp, PlantParams(), collision, model_name="mrs_buff_mk3")
    (ASSET_DIR / "mrs_buff_mk3.xml").write_text(xml.replace(str(COLLISION_DIR) + "/", "collision/"))
    print(f"wrote {ASSET_DIR}")


if __name__ == "__main__":
    main()
