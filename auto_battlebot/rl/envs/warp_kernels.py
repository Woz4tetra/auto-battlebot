"""Warp kernels for the RL navigation environment: the robot's command chain and contact flags.

One physics substep is, in order: `command_path` (transport delay, firmware mixer and yaw loop,
drivetrain delay, command shaping to wheel volts), `move_mocap` (the kinematic opponent and house
bot), `mjw.step`, `classify_contacts`, `advance`. The environment captures a run of substeps as
one CUDA graph.

The firmware is `auto_battlebot/mujoco_sim/firmware.py`'s current-firmware path (`legacy=False`,
auto steer on) ported line for line; `firmware_test` runs it alone so a test can compare it with
the scalar `FirmwareMixer`.

No `from __future__ import annotations`: warp reads the annotations as live types.
"""

import warp as wp

# yaw_control::Config and the mixer constants, as in firmware.py.
MAX_RATE = 2000.0
FEEDFORWARD = 1.0 / 27.0
KP = 0.02
KI = 0.02
I_MAX = 20.0
K_HEADING = 6.0
HOLD_RATE_MAX = 360.0
REVERSE_KP_SCALE = 4.0
REVERSE_FF_SCALE = 0.25
TURN_THRESHOLD = 1.0
CAPTURE_RATE = 45.0
CAPTURE_TIMEOUT = 0.3
IDLE_THROTTLE = 5.0
SAMPLE_PERIOD = 0.01
GYRO_RANGE_DPS = 2000.0
PI = 3.14159265358979

# Firmware state columns, one row per world.
FW_SETPOINT = 0
FW_CAPTURING = 1
FW_CAPTURE_TIME = 2
FW_INTEGRAL = 3
FW_OUTPUT = 4
FW_RATE_COMMAND = 5
FW_ACTIVE = 6
FW_WAS_UPSIDE = 7
FW_SINCE_SAMPLE = 8
FW_HELD_HEADING = 9
FW_HELD_RATE = 10
FW_HAS_HELD = 11
FW_PID_OUTPUT = 12
FW_COLUMNS = 13


@wp.func
def wrap_degrees(angle: float) -> float:
    return angle - 360.0 * wp.floor((angle + 180.0) / 360.0)


@wp.func
def yaw_reset(fw: wp.array2d(dtype=float), w: int, heading: float):
    fw[w, FW_SETPOINT] = heading
    fw[w, FW_CAPTURING] = 0.0
    fw[w, FW_CAPTURE_TIME] = 0.0
    fw[w, FW_INTEGRAL] = 0.0
    fw[w, FW_OUTPUT] = 0.0
    fw[w, FW_RATE_COMMAND] = 0.0


@wp.func
def yaw_update(
    fw: wp.array2d(dtype=float),
    w: int,
    turn: float,
    throttle: float,
    heading: float,
    rate: float,
    dt: float,
) -> float:
    turning = wp.abs(turn) > TURN_THRESHOLD
    if (not turning) and wp.abs(throttle) < IDLE_THROTTLE:
        fw[w, FW_CAPTURING] = 0.0
        fw[w, FW_SETPOINT] = heading
        fw[w, FW_RATE_COMMAND] = 0.0
        fw[w, FW_OUTPUT] = 0.0
        return 0.0
    ff_command = float(0.0)
    if turning:
        fw[w, FW_RATE_COMMAND] = turn / 100.0 * MAX_RATE
        ff_command = fw[w, FW_RATE_COMMAND]
        fw[w, FW_CAPTURING] = 1.0
        fw[w, FW_CAPTURE_TIME] = 0.0
    else:
        if fw[w, FW_CAPTURING] > 0.5:
            fw[w, FW_CAPTURE_TIME] = fw[w, FW_CAPTURE_TIME] + dt
            if wp.abs(rate) < CAPTURE_RATE or fw[w, FW_CAPTURE_TIME] >= CAPTURE_TIMEOUT:
                fw[w, FW_CAPTURING] = 0.0
                fw[w, FW_SETPOINT] = heading
        if fw[w, FW_CAPTURING] > 0.5:
            fw[w, FW_RATE_COMMAND] = 0.0
        else:
            command = K_HEADING * wrap_degrees(fw[w, FW_SETPOINT] - heading)
            fw[w, FW_RATE_COMMAND] = wp.clamp(command, -HOLD_RATE_MAX, HOLD_RATE_MAX)
    reverse = wp.clamp(throttle / 100.0, 0.0, 1.0)
    kp = KP * (1.0 + (REVERSE_KP_SCALE - 1.0) * reverse)
    feedforward = FEEDFORWARD * (1.0 + (REVERSE_FF_SCALE - 1.0) * reverse)
    error = fw[w, FW_RATE_COMMAND] - rate
    integral = fw[w, FW_INTEGRAL]
    unsaturated = feedforward * ff_command + kp * error + integral
    if wp.abs(unsaturated) < 100.0 or ((unsaturated > 0.0) != (error > 0.0)):
        integral = wp.clamp(integral + KI * error * dt, -I_MAX, I_MAX)
        fw[w, FW_INTEGRAL] = integral
    out = wp.clamp(feedforward * ff_command + kp * error + integral, -100.0, 100.0)
    fw[w, FW_OUTPUT] = out
    return out


@wp.func
def firmware_mix(
    fw: wp.array2d(dtype=float),
    w: int,
    a_percent: float,
    b_percent: float,
    heading: float,
    rate: float,
    dt: float,
    upside: int,
) -> wp.vec2:
    """main.cpp: invert throttle when upside down, run the yaw loop on fresh BNO055 samples, mix.
    `a_percent` is the CRSF value before the inversion. Returns (left, right) percent."""
    a = a_percent
    if upside == 1:
        a = -a
    since = fw[w, FW_SINCE_SAMPLE] + dt
    fresh = int(0)
    if fw[w, FW_HAS_HELD] < 0.5 or since >= SAMPLE_PERIOD - 1.0e-9:
        fw[w, FW_HELD_HEADING] = heading
        fw[w, FW_HELD_RATE] = rate
        fw[w, FW_HAS_HELD] = 1.0
        since = 0.0
        fresh = 1
    fw[w, FW_SINCE_SAMPLE] = since
    held_heading = fw[w, FW_HELD_HEADING]
    held_rate = fw[w, FW_HELD_RATE]
    tail_first = a
    if upside == 1:
        tail_first = -a
    was_upside = int(fw[w, FW_WAS_UPSIDE])
    if fw[w, FW_ACTIVE] < 0.5 or upside != was_upside:
        yaw_reset(fw, w, held_heading)
        fw[w, FW_ACTIVE] = 1.0
    if fresh == 1:
        fw[w, FW_PID_OUTPUT] = yaw_update(
            fw, w, b_percent, tail_first, held_heading, held_rate, SAMPLE_PERIOD
        )
    fw[w, FW_WAS_UPSIDE] = float(upside)
    pid = fw[w, FW_PID_OUTPUT]
    left = -a + pid
    right = -a - pid
    peak = wp.max(wp.abs(left), wp.abs(right))
    if peak > 100.0:
        left = left / peak * 100.0
        right = right / peak * 100.0
    return wp.vec2(left, right)


@wp.kernel
def firmware_test(
    a: wp.array2d(dtype=float),
    b: wp.array2d(dtype=float),
    heading: wp.array2d(dtype=float),
    rate: wp.array2d(dtype=float),
    dt: float,
    fw: wp.array2d(dtype=float),
    out: wp.array3d(dtype=float),
):
    """Run the mixer over a (W, T) sequence of inputs, one world per thread."""
    w = wp.tid()
    for t in range(a.shape[1]):
        lr = firmware_mix(fw, w, a[w, t], b[w, t], heading[w, t], rate[w, t], dt, 0)
        out[w, t, 0] = lr[0]
        out[w, t, 1] = lr[1]


@wp.func
def quat_yaw(q: wp.quat) -> float:
    # MuJoCo quats are (w, x, y, z); warp's are (x, y, z, w).
    return wp.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


@wp.func
def mj_quat(qpos: wp.array2d(dtype=float), w: int) -> wp.quat:
    return wp.quat(qpos[w, 4], qpos[w, 5], qpos[w, 6], qpos[w, 3])


@wp.kernel
def command_path(
    head: wp.array(dtype=int),
    cmd: wp.array2d(dtype=float),
    transport_ring: wp.array3d(dtype=float),
    transport_lag: wp.array(dtype=int),
    drive_ring: wp.array3d(dtype=float),
    drive_lag: wp.array(dtype=int),
    fw: wp.array2d(dtype=float),
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    wheel_dofs: wp.array(dtype=int),
    shaping: wp.array2d(dtype=float),
    emf_per_radps: float,
    dt: float,
    upside: wp.array(dtype=int),
    ctrl: wp.array2d(dtype=float),
):
    """Jetson command -> transport delay -> firmware -> drivetrain delay -> shaped wheel volts.

    `shaping` columns: deadzone left, deadzone right, curve, left gain, right gain, pack volts,
    left reverse gain, right reverse gain, coast flag.
    """
    w = wp.tid()
    cap = transport_ring.shape[1]
    h = head[0] % cap
    transport_ring[w, h, 0] = cmd[w, 0]
    transport_ring[w, h, 1] = cmd[w, 1]
    th = (h - transport_lag[w] + cap) % cap
    linear = transport_ring[w, th, 0]
    angular = transport_ring[w, th, 1]
    # CRSF percents: the firmware negates channel 0 into a_percent, the transmitter reverses
    # the angular channel (closed_loop.py).
    a_percent = -100.0 * wp.clamp(linear, -1.0, 1.0)
    b_percent = -100.0 * wp.clamp(angular, -1.0, 1.0)

    q = mj_quat(qpos, w)
    yaw = quat_yaw(q)
    heading = 0.0 - yaw * 180.0 / PI
    heading = heading - 360.0 * wp.floor(heading / 360.0)
    omega_world = wp.quat_rotate(q, wp.vec3(qvel[w, 3], qvel[w, 4], qvel[w, 5]))
    rate = wp.clamp(0.0 - omega_world[2] * 180.0 / PI, -GYRO_RANGE_DPS, GYRO_RANGE_DPS)
    up = wp.quat_rotate(q, wp.vec3(0.0, 0.0, 1.0))
    if up[2] < -0.1:
        upside[w] = 1
    elif up[2] > 0.1:
        upside[w] = 0
    lr = firmware_mix(fw, w, a_percent, b_percent, heading, rate, dt, upside[w])

    drive_ring[w, h, 0] = lr[0]
    drive_ring[w, h, 1] = lr[1]
    dh = (h - drive_lag[w] + cap) % cap
    for k in range(2):
        u = wp.clamp(drive_ring[w, dh, k] / 100.0, -1.0, 1.0)
        dz = shaping[w, k]
        mag = wp.clamp((wp.abs(u) - dz) / (1.0 - dz), 0.0, 1.0)
        bent = mag * (1.0 + shaping[w, 2] * (1.0 - mag))
        shaped = wp.sign(u) * bent
        if mag <= 0.0:
            shaped = 0.0
        gain = shaping[w, 3 + k]
        if shaped < 0.0:
            gain = gain * shaping[w, 6 + k]
        volts = shaped * shaping[w, 5] * gain
        if shaped == 0.0 and shaping[w, 8] > 0.5:
            volts = emf_per_radps * qvel[w, wheel_dofs[k]]
        ctrl[w, k] = volts


@wp.kernel
def move_mocap(
    mocap_pos: wp.array2d(dtype=wp.vec3),
    mocap_quat: wp.array2d(dtype=wp.quat),
    moving: wp.array(dtype=int),
    velocity: wp.array3d(dtype=float),
    dt: float,
):
    """Integrate each moving mocap body's planar velocity (vx, vy, yaw rate) over dt."""
    w, i = wp.tid()
    body = moving[i]
    p = mocap_pos[w, body]
    mocap_pos[w, body] = wp.vec3(p[0] + velocity[w, i, 0] * dt, p[1] + velocity[w, i, 1] * dt, p[2])
    q = mocap_quat[w, body]  # MuJoCo order (w, x, y, z) stored in warp's quat slots
    yaw = wp.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
    yaw = yaw + velocity[w, i, 2] * dt
    mocap_quat[w, body] = wp.quat(wp.cos(0.5 * yaw), 0.0, 0.0, wp.sin(0.5 * yaw))


@wp.kernel
def classify_contacts(
    nacon: wp.array(dtype=int),
    contact_geom: wp.array(dtype=wp.vec2i),
    contact_pos: wp.array(dtype=wp.vec3),
    contact_world: wp.array(dtype=int),
    geom_role: wp.array(dtype=int),
    qpos: wp.array2d(dtype=float),
    mocap_pos: wp.array2d(dtype=wp.vec3),
    mocap_quat: wp.array2d(dtype=wp.quat),
    opponent_mocap: int,
    arc_half_angle: float,
    strike_x: float,
    strike_half_width: float,
    flags: wp.array2d(dtype=float),
):
    """OR this substep's contacts into per-world flags.

    `geom_role`: 0 other, 1 our robot, 2 opponent, 3 arena wall, 4 wall block, 5 house bot.
    Flag columns: 0 any opponent contact, 1 contact inside the opponent's arc, 2 contact on our
    wedge outside the arc, 3 largest |phi| of such a wedge contact, 4 wall, 5 wall block,
    6 house bot.
    """
    c = wp.tid()
    if c >= nacon[0]:
        return
    g = contact_geom[c]
    r0 = geom_role[g[0]]
    r1 = geom_role[g[1]]
    other = int(0)
    if r0 == 1:
        other = r1
    elif r1 == 1:
        other = r0
    else:
        return
    w = contact_world[c]
    if other == 3:
        wp.atomic_max(flags, w, 4, 1.0)
        return
    if other == 4:
        wp.atomic_max(flags, w, 5, 1.0)
        return
    if other == 5:
        wp.atomic_max(flags, w, 6, 1.0)
        return
    if other != 2:
        return
    p = contact_pos[c]
    op = mocap_pos[w, opponent_mocap]
    oq = mocap_quat[w, opponent_mocap]
    oyaw = wp.atan2(
        2.0 * (oq[0] * oq[3] + oq[1] * oq[2]), 1.0 - 2.0 * (oq[2] * oq[2] + oq[3] * oq[3])
    )
    ang = wp.atan2(p[1] - op[1], p[0] - op[0]) - oyaw
    phi = ang - 2.0 * PI * wp.floor((ang + PI) / (2.0 * PI))
    wp.atomic_max(flags, w, 0, 1.0)
    in_arc = wp.abs(phi) < arc_half_angle
    if in_arc:
        wp.atomic_max(flags, w, 1, 1.0)
        return
    q = mj_quat(qpos, w)
    rel = wp.vec3(p[0] - qpos[w, 0], p[1] - qpos[w, 1], p[2] - qpos[w, 2])
    b = wp.quat_rotate_inv(q, rel)
    if b[0] > strike_x and wp.abs(b[1]) < strike_half_width:
        wp.atomic_max(flags, w, 2, 1.0)
        wp.atomic_max(flags, w, 3, wp.abs(phi))


@wp.kernel
def advance(head: wp.array(dtype=int)):
    head[0] = head[0] + 1
