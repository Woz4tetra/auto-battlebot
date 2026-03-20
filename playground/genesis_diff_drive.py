"""Minimal Genesis simulation with a differential-drive robot.

Creates a two-wheeled robot from a procedurally generated URDF (box chassis,
two cylinder wheels, rear ball caster), drops it onto a ground plane, and
drives it in a figure-8 pattern using wheel velocity control.

Usage:
    python playground/genesis_diff_drive.py
"""

from __future__ import annotations

import tempfile
import textwrap
from pathlib import Path

import numpy as np

import genesis as gs

# ── Robot geometry ────────────────────────────────────────────────────────────
CHASSIS_X, CHASSIS_Y, CHASSIS_Z = 0.20, 0.14, 0.05
WHEEL_RADIUS = 0.04
WHEEL_WIDTH = 0.02
TRACK_WIDTH = 0.16
CHASSIS_MASS = 0.5
WHEEL_MASS = 0.05

# ── Drive parameters (match main sim: sim_config.toml) ───────────────────────
MAX_WHEEL_SPEED = 40.0  # rad/s
WHEEL_KP = 0.0
WHEEL_KV = 50.0
WHEEL_FORCE_LIMIT = 200.0

URDF_TEMPLATE = textwrap.dedent("""\
<?xml version="1.0"?>
<robot name="diff_drive">

  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{chassis_mass}"/>
      <inertia ixx="{chassis_ixx}" ixy="0" ixz="0"
               iyy="{chassis_iyy}" iyz="0" izz="{chassis_izz}"/>
    </inertial>
    <visual>
      <geometry><box size="{cx} {cy} {cz}"/></geometry>
    </visual>
  </link>

  <link name="left_wheel">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{wheel_mass}"/>
      <inertia ixx="{wheel_ixx}" ixy="0" ixz="0"
               iyy="{wheel_iyy}" iyz="0" izz="{wheel_ixx}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{wr}" length="{ww}"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{wr}" length="{ww}"/></geometry>
    </collision>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 {half_track} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="right_wheel">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{wheel_mass}"/>
      <inertia ixx="{wheel_ixx}" ixy="0" ixz="0"
               iyy="{wheel_iyy}" iyz="0" izz="{wheel_ixx}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{wr}" length="{ww}"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{wr}" length="{ww}"/></geometry>
    </collision>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -{half_track} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>


</robot>
""")


def _write_urdf(path: Path) -> None:
    # Box inertia: I = m/12 * (a² + b²)
    m = CHASSIS_MASS
    chassis_ixx = m / 12 * (CHASSIS_Y**2 + CHASSIS_Z**2)
    chassis_iyy = m / 12 * (CHASSIS_X**2 + CHASSIS_Z**2)
    chassis_izz = m / 12 * (CHASSIS_X**2 + CHASSIS_Y**2)

    # Cylinder inertia (rotation axis = Y after the rpy transform)
    mw = WHEEL_MASS
    r, h = WHEEL_RADIUS, WHEEL_WIDTH
    wheel_ixx = mw / 12 * (3 * r**2 + h**2)
    wheel_iyy = mw * r**2 / 2

    path.write_text(
        URDF_TEMPLATE.format(
            chassis_mass=CHASSIS_MASS,
            cx=CHASSIS_X,
            cy=CHASSIS_Y,
            cz=CHASSIS_Z,
            chassis_ixx=f"{chassis_ixx:.6e}",
            chassis_iyy=f"{chassis_iyy:.6e}",
            chassis_izz=f"{chassis_izz:.6e}",
            wheel_mass=WHEEL_MASS,
            wr=WHEEL_RADIUS,
            ww=WHEEL_WIDTH,
            wheel_ixx=f"{wheel_ixx:.6e}",
            wheel_iyy=f"{wheel_iyy:.6e}",
            half_track=TRACK_WIDTH / 2,
        )
    )


def diff_drive(linear: float, angular: float) -> tuple[float, float]:
    """Convert (v, omega) into (left_vel, right_vel) in rad/s."""
    half = TRACK_WIDTH / 2
    v_l = (linear - angular * half) / WHEEL_RADIUS
    v_r = (linear + angular * half) / WHEEL_RADIUS
    return v_l, v_r


def main() -> None:
    urdf_path = Path(tempfile.mktemp(suffix=".urdf"))
    _write_urdf(urdf_path)

    gs.init(backend=gs.gpu)

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
            substeps=10,
        ),
        rigid_options=gs.options.RigidOptions(
            enable_self_collision=False,
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0, -1.0, 0.8),
            camera_lookat=(0.0, 0.0, 0.0),
            camera_fov=40,
            res=(1280, 720),
            max_FPS=60,
        ),
        show_viewer=True,
    )

    scene.add_entity(gs.morphs.Plane())

    robot = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_path),
            pos=(0.0, 0.0, WHEEL_RADIUS),
            fixed=False,
        ),
        gs.materials.Rigid(friction=0.8),
    )

    scene.build()

    # Resolve wheel DOFs and configure PD gains
    left_dof = robot.get_joint("left_wheel_joint").dofs_idx_local[0]
    right_dof = robot.get_joint("right_wheel_joint").dofs_idx_local[0]
    wheel_dofs = [left_dof, right_dof]

    robot.set_dofs_kp(np.array([WHEEL_KP, WHEEL_KP]), dofs_idx_local=wheel_dofs)
    robot.set_dofs_kv(np.array([WHEEL_KV, WHEEL_KV]), dofs_idx_local=wheel_dofs)
    robot.set_dofs_force_range(
        lower=np.full(2, -WHEEL_FORCE_LIMIT),
        upper=np.full(2, WHEEL_FORCE_LIMIT),
        dofs_idx_local=wheel_dofs,
    )

    # Let the robot settle onto the ground before driving
    for _ in range(100):
        scene.step()

    # ── Scripted demo: figure-8 pattern ───────────────────────────────────
    dt = 0.01  # matches SimOptions.dt
    total_steps = 4000
    period = 3.0  # seconds per half-loop

    print(f"Running figure-8 for {total_steps * dt:.0f}s …")

    for step in range(total_steps):
        t = step * dt
        linear = 0.3  # m/s forward
        angular = 3.0 if (t % (2 * period)) < period else -3.0

        v_l, v_r = diff_drive(linear, angular)
        v_l = float(np.clip(v_l, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
        v_r = float(np.clip(v_r, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))

        robot.control_dofs_velocity(np.array([v_l, v_r]), dofs_idx_local=wheel_dofs)
        scene.step()

    print("Done.")
    urdf_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
