"""Test mr_stabs_mk2 wheel drive in isolation -- tracer demo pattern.

Generates a temporary URDF with mr_stabs physical properties but NO caster
spheres and COM centered over the wheel axis.  This isolates wheel drive
behavior from caster drag.

If rotation matches the command here but not with the real URDF, the problem
is caster drag (fixed spheres share the entity's friction material and create
massive sliding resistance).

Usage:
    python playground/test_mr_stabs.py
"""

import os
import tempfile
import textwrap
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import genesis as gs

WHEEL_R = 0.025
TRACK_L = 0.128
HALF_TRACK = TRACK_L / 2.0

MESH_DIR = os.path.join(
    os.path.dirname(__file__),
    "..",
    "simulation",
    "assets",
    "robots",
    "mr_stabs_mk2",
    "meshes",
)

URDF_TEMPLATE = textwrap.dedent("""\
<?xml version="1.0"?>
<robot name="mr_stabs_mk2_test">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="4.228e-01" />
      <inertia ixx="7.503e-04" ixy="0" ixz="0"
               iyy="5.076e-04" iyz="0" izz="1.187e-03" />
    </inertial>
    <visual>
      <geometry>
        <mesh filename="{chassis_mesh}" />
      </geometry>
    </visual>
  </link>

  <!-- Pitch stabilizer casters — ~zero normal force at equilibrium (COM at origin) -->
  <link name="caster_front">
    <inertial>
      <mass value="0.001" />
      <inertia ixx="1e-8" ixy="0" ixz="0" iyy="1e-8" iyz="0" izz="1e-8" />
    </inertial>
    <collision>
      <geometry><sphere radius="0.005" /></geometry>
    </collision>
  </link>
  <joint name="caster_front_joint" type="fixed">
    <parent link="base_link" />
    <child link="caster_front" />
    <origin xyz="0.08 0.0 -0.020" rpy="0 0 0" />
  </joint>

  <link name="caster_rear">
    <inertial>
      <mass value="0.001" />
      <inertia ixx="1e-8" ixy="0" ixz="0" iyy="1e-8" iyz="0" izz="1e-8" />
    </inertial>
    <collision>
      <geometry><sphere radius="0.005" /></geometry>
    </collision>
  </link>
  <joint name="caster_rear_joint" type="fixed">
    <parent link="base_link" />
    <child link="caster_rear" />
    <origin xyz="-0.015 0.0 -0.020" rpy="0 0 0" />
  </joint>

  <link name="left_wheel">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1.5e-02" />
      <inertia ixx="1.821e-06" ixy="0" ixz="0"
               iyy="3.325e-06" iyz="0" izz="1.821e-06" />
    </inertial>
    <visual>
      <geometry>
        <mesh filename="{wheel_mesh}" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5707963267948966 0 0" />
      <geometry>
        <cylinder radius="0.025" length="0.012" />
      </geometry>
    </collision>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="left_wheel" />
    <origin xyz="0 0.064 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <dynamics damping="0.1" friction="0.0" />
  </joint>

  <link name="right_wheel">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1.5e-02" />
      <inertia ixx="1.821e-06" ixy="0" ixz="0"
               iyy="3.325e-06" iyz="0" izz="1.821e-06" />
    </inertial>
    <visual>
      <geometry>
        <mesh filename="{wheel_mesh}" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5707963267948966 0 0" />
      <geometry>
        <cylinder radius="0.025" length="0.012" />
      </geometry>
    </collision>
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="right_wheel" />
    <origin xyz="0 -0.064 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <dynamics damping="0.1" friction="0.0" />
  </joint>
</robot>
""")


class DiffDrive:
    def __init__(self, r: float, l: float):
        self._r = r
        self._l = l

    def __call__(self, v: float, w: float):
        c1 = 1.0 / (2.0 * self._r)
        c2 = 2.0 * v
        c3 = w * self._l
        w_r = c1 * (c2 + c3)
        w_l = c1 * (c2 - c3)
        return np.array([w_r, w_l])


def write_urdf(path: Path) -> None:
    chassis_mesh = os.path.abspath(os.path.join(MESH_DIR, "chassis.obj"))
    wheel_mesh = os.path.abspath(os.path.join(MESH_DIR, "wheel.obj"))
    path.write_text(
        URDF_TEMPLATE.format(
            chassis_mesh=chassis_mesh,
            wheel_mesh=wheel_mesh,
        )
    )


gs.init(backend=gs.gpu)
controller = DiffDrive(r=WHEEL_R, l=TRACK_L)

scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.01, substeps=10, requires_grad=False),
    rigid_options=gs.options.RigidOptions(enable_self_collision=False),
    show_viewer=True,
)

plane = scene.add_entity(
    gs.morphs.Plane(),
    gs.materials.Rigid(friction=1.0),
)

urdf_path = Path(tempfile.mktemp(suffix=".urdf"))
write_urdf(urdf_path)
print(f"Temp URDF: {urdf_path}")

robot = scene.add_entity(
    gs.morphs.URDF(
        file=str(urdf_path),
        pos=(0.0, 0.0, WHEEL_R),
        euler=(0, 0, 0),
        decimate_aggressiveness=0,
    ),
    gs.materials.Rigid(friction=0.8),
)

scene.build()
urdf_path.unlink(missing_ok=True)

jnt_names = ["right_wheel_joint", "left_wheel_joint"]
dofs_idx = [robot.get_joint(name).dof_idx_local for name in jnt_names]

robot.set_dofs_kp(np.array([0.0, 0.0]), dofs_idx_local=dofs_idx)
robot.set_dofs_kv(np.array([50.0, 50.0]), dofs_idx_local=dofs_idx)
robot.set_dofs_force_range(
    lower=np.array([-200.0, -200.0]),
    upper=np.array([200.0, 200.0]),
    dofs_idx_local=dofs_idx,
)

for _ in range(200):
    scene.step()

cmd_v = 0.0
cmd_w = np.deg2rad(360.0)
action = controller(v=cmd_v, w=cmd_w)

dt = 0.01
prev_quat = None
prev_pos = None

print(f"Wheel radius: {WHEEL_R}, Track width: {TRACK_L}")
print(
    f"DOF indices: right={dofs_idx[0]}, left={dofs_idx[1]}, total n_dofs={robot.n_dofs}"
)
print(f"COM at (0,0,0) over wheel axis, pitch stabilizer casters (near-zero load)")
print(f"Command: v={cmd_v:.2f} m/s, w={np.rad2deg(cmd_w):.1f} deg/s")
print(f"Wheel targets: right={action[0]:.2f} rad/s, left={action[1]:.2f} rad/s")
print(
    f"{'step':>6} | {'cmd_wr':>8} {'act_wr':>8} {'cmd_wl':>8} {'act_wl':>8} | "
    f"{'cmd_v':>7} {'meas_v':>7} {'cmd_w':>8} {'meas_w':>8} {'yaw':>7}"
)
print("-" * 105)

for i in range(10000):
    robot.control_dofs_velocity(
        np.array([action[0], action[1]]),
        dofs_idx,
    )
    scene.step()

    if i % 50 == 0:
        all_vel = robot.get_dofs_velocity().cpu().numpy().squeeze()
        act_wr = float(all_vel[dofs_idx[0]])
        act_wl = float(all_vel[dofs_idx[1]])

        pos = robot.get_pos().cpu().numpy().squeeze()
        quat_wxyz = robot.get_quat().cpu().numpy().squeeze()
        quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
        rot = Rotation.from_quat(quat_xyzw)
        yaw = rot.as_euler("ZYX")[0]

        meas_v = 0.0
        meas_w = 0.0
        if prev_quat is not None:
            elapsed = 50 * dt
            dx = pos[0] - prev_pos[0]
            dy = pos[1] - prev_pos[1]
            meas_v = np.sqrt(dx**2 + dy**2) / elapsed

            prev_rot = Rotation.from_quat(prev_quat)
            prev_yaw = prev_rot.as_euler("ZYX")[0]
            dyaw = yaw - prev_yaw
            if dyaw > np.pi:
                dyaw -= 2 * np.pi
            elif dyaw < -np.pi:
                dyaw += 2 * np.pi
            meas_w = dyaw / elapsed

        prev_quat = quat_xyzw
        prev_pos = pos.copy()

        print(
            f"{i:6d} | {action[0]:8.2f} {act_wr:8.2f} {action[1]:8.2f} {act_wl:8.2f} | "
            f"{cmd_v:7.3f} {meas_v:7.3f} {np.rad2deg(cmd_w):8.1f} {np.rad2deg(meas_w):8.1f} {np.rad2deg(yaw):7.1f}"
        )
