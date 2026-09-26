"""Rigid-body MuJoCo model of Mr Stabs Mk2 and its fit against hand-driven recordings.

onshape_export   Onshape URDF export: kinematic tree, lumped wheels, tag frames from the meshes
mass_properties  loader for simulation/assets/robots/mr_stabs_mk2/mass_properties.toml
mjcf             MJCF builder from the mass-property table
actuator         DC motor to MuJoCo `general` actuator mapping and the command-tape transform
firmware         mix_motor_outputs and the heading-hold PID, for closed-loop use only
session          loads a smooth_tag_poses.py session bundle into rollout windows
rollout          batched MuJoCo Warp rollouts with per-world parameters, plus a CPU reference
fit              parameter vector, loss, staged CMA-ES
"""
