"""Warp kernels for the batched rollout: tape to ctrl, pose recording, the step counter.

Kept in their own module, imported only when a rollout runs, so the rest of mujoco_sim loads
without warp. No `from __future__ import annotations` here: warp reads the kernel annotations
as live types.
"""

import warp as wp


@wp.kernel
def control(
    step: wp.array(dtype=int),
    tape_v: wp.array3d(dtype=float),
    tape_zero: wp.array3d(dtype=float),
    coast: wp.array(dtype=float),
    qvel: wp.array2d(dtype=float),
    wheel_dofs: wp.array(dtype=int),
    emf_per_radps: float,
    steps: int,
    ctrl: wp.array2d(dtype=float),
):
    """ctrl = tape voltage; on coast steps, the back-EMF voltage, which zeroes motor torque."""
    w = wp.tid()
    t = wp.min(step[0], steps - 1)
    for k in range(2):
        v = tape_v[w, t, k]
        if tape_zero[w, t, k] > 0.5 and coast[w] > 0.5:
            v = emf_per_radps * qvel[w, wheel_dofs[k]]
        ctrl[w, k] = v


@wp.kernel
def record(
    step: wp.array(dtype=int),
    qpos: wp.array2d(dtype=float),
    every: int,
    samples: int,
    out: wp.array3d(dtype=float),
):
    w = wp.tid()
    s = step[0]
    if s % every == 0 and s / every < samples:
        for i in range(7):
            out[w, s / every, i] = qpos[w, i]


@wp.kernel
def advance(step: wp.array(dtype=int)):
    step[0] = step[0] + 1
