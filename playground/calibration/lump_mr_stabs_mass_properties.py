"""Lump the Mr Stabs Mk2 Onshape export into three bodies and write mass_properties.toml.

Chassis, left wheel and right wheel, in the FLU body frame at the axle midpoint, plus the
tag-to-body transforms for tags 41 and 76 and the drivetrain constants. The MJCF builder and
the tag pose smoother both read the output.

    python playground/calibration/lump_mr_stabs_mass_properties.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np
import tomli_w
import trimesh

from auto_battlebot.mujoco_sim.onshape_export import (
    STRUCTURAL_LINKS,
    WHEEL_JOINTS,
    MassBody,
    UrdfAssembly,
    body_from_root,
    chassis_from_assembly,
    combine,
    extract_tag_frame,
    load_mesh_in_root,
    onshape_inertia_to_tensor,
)

ASSET_DIR = Path(__file__).resolve().parents[2] / "simulation/assets/robots/mr_stabs_mk2"

# Onshape "Mass and section properties" on the whole Mr Stabs Mk2 assembly, screenshot
# 2026-09-25 23:10 (onshape_export/onshape_mass_properties_2026-09-25.png). Assembly frame, mm
# and g mm^2, inertia about the COM. Unlike the URDF export, this includes the Onshape mass
# overrides for the motors, ESCs, flight controller, batteries and switch.
ONSHAPE_MASS_G = 494.22
ONSHAPE_COM_MM = (0.004, -29.548, 0.798)
ONSHAPE_INERTIA_G_MM2 = {
    "lxx": 589754.02,
    "lyy": 938677.976,
    "lzz": 1.449e6,
    "lxy": -88.895,
    "lxz": 248.066,
    "lyz": 3285.15,
}
SCALE_MASS_G = 495.9

# Repeat Robotics Compact 1806 gearmotor (repeat-robotics.com/products/repeat-compact-1806).
GEAR_RATIO = 22.6
MOTOR_KV_RPM_PER_V = 2300.0
# CAD integration of the rotor: steel flux ring, 14 N52 magnets, aluminum end cap, steel shaft.
ROTOR_INERTIA_KG_M2 = 9.3e-7
GEARMOTOR_MASS_KG = 0.0445
WHEEL_RADIUS_M = 0.025

TAG_OUTWARD = {41: np.array([0.0, 0.0, -1.0]), 76: np.array([0.0, 0.0, 1.0])}


def _body(b: MassBody) -> dict[str, Any]:
    i = b.inertia
    return {
        "mass_kg": round(b.mass, 7),
        "com_m": [round(float(v), 7) for v in b.com],
        # MuJoCo fullinertia order: Ixx Iyy Izz Ixy Ixz Iyz, about the COM, body axes.
        "fullinertia_kg_m2": [
            float(f"{v:.5e}") for v in (i[0, 0], i[1, 1], i[2, 2], i[0, 1], i[0, 2], i[1, 2])
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--export-dir", type=Path, default=ASSET_DIR / "onshape_export")
    parser.add_argument("--out", type=Path, default=ASSET_DIR / "mass_properties.toml")
    args = parser.parse_args()

    assembly = UrdfAssembly(args.export_dir / "mr_stabs_mk2.urdf")
    rot, trans = body_from_root(assembly)
    to_body = np.eye(4)
    to_body[:3, :3], to_body[:3, 3] = rot, trans

    wheels = [
        assembly.lumped(assembly.wheel_links(j)).transformed(rot, trans) for j in WHEEL_JOINTS
    ]
    wheels.sort(key=lambda w: -w.com[1])  # left (+y) first
    total_asm = MassBody(
        mass=ONSHAPE_MASS_G * 1e-3,
        com=np.array(ONSHAPE_COM_MM) * 1e-3,
        inertia=onshape_inertia_to_tensor(**ONSHAPE_INERTIA_G_MM2),
    )
    total = total_asm.transformed(rot, trans)
    chassis_cad, chassis = chassis_from_assembly(total, wheels, SCALE_MASS_G * 1e-3)
    whole = combine([chassis, *wheels])

    tread = trimesh.util.concatenate(
        [
            m
            for m, _ in load_mesh_in_root(
                assembly, args.export_dir / "meshes", "mr_stabs_gum_rubber_wheel_tread"
            )
        ]
    )
    tread.apply_transform(to_body)
    tread_width = float(tread.bounds[1, 1] - tread.bounds[0, 1])
    tread_radius = float(tread.bounds[1, 2] - tread.bounds[0, 2]) / 2.0

    tags: dict[str, Any] = {}
    for tag_id, outward in TAG_OUTWARD.items():
        meshes = [
            (m.copy().apply_transform(to_body), c)
            for m, c in load_mesh_in_root(
                assembly, args.export_dir / "meshes", f"apriltag_36h11_{tag_id}"
            )
        ]
        frame = extract_tag_frame(meshes, tag_id, outward)
        if frame.mirrored:
            raise SystemExit(f"tag {tag_id}: CAD face only decodes mirrored; fix the model first")
        tags[str(tag_id)] = {
            "upside_down": bool(frame.rotation[2, 2] < 0.0),
            "translation_m": [round(float(v), 6) for v in frame.translation],
            "rotation": [[round(float(v), 6) for v in row] for row in frame.rotation],
            "size_m": round(frame.size_m, 5),
            "tilt_deg": round(frame.tilt_deg, 2),
        }

    reflected = GEAR_RATIO**2 * ROTOR_INERTIA_KG_M2
    data: dict[str, Any] = {
        "provenance": {
            "onshape_assembly": "Onshape Mass and section properties, whole assembly, "
            "screenshot 2026-09-25 23:10 (onshape_export/onshape_mass_properties_2026-09-25.png)",
            "urdf_export": "onshape_export/mr_stabs_mk2.urdf (Onshape URDF exporter 1.221); "
            "used for the wheels and tag frames only, it carries no mass overrides",
            "chassis": "whole assembly minus both wheels, topped up to the scale mass at its COM "
            "with inertia scaled by the mass ratio",
            "scale_mass_kg": SCALE_MASS_G * 1e-3,
            "cad_mass_kg": ONSHAPE_MASS_G * 1e-3,
            "chassis_cad_mass_kg": round(chassis_cad.mass, 7),
            "rotor": "CAD integration of the 1806 rotor: steel flux ring, 14 magnets, aluminum "
            "end cap (a steel cap would overshoot the listed 22.4 g motor mass), steel shaft. "
            "Gearbox internals are missing from the CAD; bounding them adds at most 4%",
            "generator": "playground/calibration/lump_mr_stabs_mass_properties.py",
        },
        "frame": {
            "description": "FLU at the axle midpoint: x forward, y left, z up. "
            "x = -Y_asm, y = +X_asm, z = +Z_asm",
        },
        "chassis": _body(chassis),
        "wheel_left": _body(wheels[0]),
        "wheel_right": _body(wheels[1]),
        "whole": _body(whole),
        "geometry": {
            "track_half_width_m": round(float(0.5 * (wheels[0].com[1] - wheels[1].com[1])), 6),
            "wheel_radius_m": WHEEL_RADIUS_M,
            "tread_radius_cad_m": round(tread_radius, 5),
            "tread_width_m": round(tread_width, 5),
        },
        "drivetrain": {
            "gear_ratio": GEAR_RATIO,
            "motor_kv_rpm_per_v": MOTOR_KV_RPM_PER_V,
            "motor_kt_nm_per_a": round(60.0 / (2.0 * np.pi * MOTOR_KV_RPM_PER_V), 7),
            "rotor_inertia_kg_m2": ROTOR_INERTIA_KG_M2,
            "reflected_inertia_kg_m2": float(f"{reflected:.4e}"),
            "gearmotor_mass_kg": GEARMOTOR_MASS_KG,
        },
        "tags": tags,
        "structural_links": list(STRUCTURAL_LINKS),
    }
    header = (
        "# Mr Stabs Mk2 mass properties and tag frames. Generated; do not edit by hand.\n"
        "# Regenerate: python playground/calibration/lump_mr_stabs_mass_properties.py\n"
        "# Tag frames: T_body_tag maps OpenCV marker-frame points (origin at the tag center,\n"
        "# x right and y up in the printed image, z out of the face) into the body frame.\n"
        "# rotation is R_body_tag, row-major. upside_down = true for the tag that faces the\n"
        "# floor when the robot is upright. Found by rendering each CAD face and decoding it.\n\n"
    )
    args.out.write_text(header + tomli_w.dumps(data))
    print(f"wrote {args.out}")
    for name in ("chassis", "wheel_left", "whole"):
        print(name, data[name])
    print(
        "tags", {k: (v["translation_m"], v["tilt_deg"], v["upside_down"]) for k, v in tags.items()}
    )


if __name__ == "__main__":
    main()
