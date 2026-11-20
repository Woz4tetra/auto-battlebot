"""Warp kernels for the RL navigation environment: the robot's command chain and contact flags."""

import warp as wp

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
WEAPON_WEDGE = 0
WEAPON_DISK = 1
ARC_ANGLE = 0
ARC_FRONT_FACE = 1
FRONT_FACE_MARGIN_M = 0.01
WEAPON_SWEEP_MARGIN_M = 0.005
SWEEP_ANGLES = 16
SWEEP_CENTRE_X, SWEEP_CENTRE_Z, SWEEP_RADIUS, SWEEP_HALF_WIDTH = range(4)
SHAPE_DEADZONE_LEFT = 0
SHAPE_DEADZONE_RIGHT = 1
SHAPE_CURVE = 2
SHAPE_GAIN_LEFT = 3
SHAPE_GAIN_RIGHT = 4
SHAPE_PACK_VOLTS = 5
SHAPE_REVERSE_GAIN_LEFT = 6
SHAPE_REVERSE_GAIN_RIGHT = 7
SHAPE_COAST = 8
SHAPE_COLUMNS = 9
GEOM_ROLE_OTHER = 0
GEOM_ROLE_OURS = 1
GEOM_ROLE_OPPONENT = 2
GEOM_ROLE_WALL = 3
GEOM_ROLE_WALL_BLOCK = 4
GEOM_ROLE_HOUSE_BOT = 5
FLAG_OPPONENT_CONTACT = 0
FLAG_ARC_CONTACT = 1
FLAG_WEDGE_OUTSIDE_ARC = 2
FLAG_WEDGE_MAX_PHI = 3
FLAG_WALL = 4
FLAG_WALL_BLOCK = 5
FLAG_HOUSE_BOT = 6
FLAG_COLUMNS = 7

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
    """main.cpp loop: invert when upside down, yaw loop on fresh samples, mix."""
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
def yaw_of_warp_quat(q: wp.quat) -> float:
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
    direct: int,
    ctrl: wp.array2d(dtype=float),
):
    """Jetson command through transport delay, firmware or direct mix, and drivetrain delay."""
    w = wp.tid()
    cap = transport_ring.shape[1]
    h = head[0] % cap
    transport_ring[w, h, 0] = cmd[w, 0]
    transport_ring[w, h, 1] = cmd[w, 1]
    th = (h - transport_lag[w] + cap) % cap
    linear = transport_ring[w, th, 0]
    angular = transport_ring[w, th, 1]
    a_percent = -100.0 * wp.clamp(linear, -1.0, 1.0)
    b_percent = -100.0 * wp.clamp(angular, -1.0, 1.0)

    q = mj_quat(qpos, w)
    yaw = yaw_of_warp_quat(q)
    heading = 0.0 - yaw * 180.0 / PI
    heading = heading - 360.0 * wp.floor(heading / 360.0)
    omega_world = wp.quat_rotate(q, wp.vec3(qvel[w, 3], qvel[w, 4], qvel[w, 5]))
    rate = wp.clamp(0.0 - omega_world[2] * 180.0 / PI, -GYRO_RANGE_DPS, GYRO_RANGE_DPS)
    up = wp.quat_rotate(q, wp.vec3(0.0, 0.0, 1.0))
    if up[2] < -0.1:
        upside[w] = 1
    elif up[2] > 0.1:
        upside[w] = 0
    lr = wp.vec2(0.0, 0.0)
    if direct == 1:
        lin = wp.clamp(linear, -1.0, 1.0)
        ang = wp.clamp(angular, -1.0, 1.0)
        lr = wp.vec2(100.0 * (lin - ang), 100.0 * (lin + ang))
    else:
        lr = firmware_mix(fw, w, a_percent, b_percent, heading, rate, dt, upside[w])

    drive_ring[w, h, 0] = lr[0]
    drive_ring[w, h, 1] = lr[1]
    dh = (h - drive_lag[w] + cap) % cap
    for k in range(2):
        u = wp.clamp(drive_ring[w, dh, k] / 100.0, -1.0, 1.0)
        dz = shaping[w, SHAPE_DEADZONE_LEFT + k]
        mag = wp.clamp((wp.abs(u) - dz) / (1.0 - dz), 0.0, 1.0)
        bent = mag * (1.0 + shaping[w, SHAPE_CURVE] * (1.0 - mag))
        shaped = wp.sign(u) * bent
        if mag <= 0.0:
            shaped = 0.0
        gain = shaping[w, SHAPE_GAIN_LEFT + k]
        if shaped < 0.0:
            gain = gain * shaping[w, SHAPE_REVERSE_GAIN_LEFT + k]
        volts = shaped * shaping[w, SHAPE_PACK_VOLTS] * gain
        if shaped == 0.0 and shaping[w, SHAPE_COAST] > 0.5:
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
    q = mocap_quat[w, body]
    yaw = wp.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
    yaw = yaw + velocity[w, i, 2] * dt
    mocap_quat[w, body] = wp.quat(wp.cos(0.5 * yaw), 0.0, 0.0, wp.sin(0.5 * yaw))


@wp.func
def box_distance(p: wp.vec3, centre: wp.vec3, yaw: float, half: wp.vec3) -> float:
    """Distance from a point to a box turned by yaw about z (0 inside)."""
    d = p - centre
    c = wp.cos(yaw)
    s = wp.sin(yaw)
    local = wp.vec3(c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2])
    q = wp.vec3(
        wp.max(wp.abs(local[0]) - half[0], 0.0),
        wp.max(wp.abs(local[1]) - half[1], 0.0),
        wp.max(wp.abs(local[2]) - half[2], 0.0),
    )
    return wp.length(q)


@wp.func
def weapon_sweep_hit(
    qpos: wp.array2d(dtype=float),
    w: int,
    sweep: wp.vec4,
    sweep_y: float,
    box_centre: wp.vec3,
    box_yaw: float,
    box_half: wp.vec3,
) -> int:
    """1 if sample points on the weapon's sweep cylinder come within the margin of the box."""
    q = mj_quat(qpos, w)
    origin = wp.vec3(qpos[w, 0], qpos[w, 1], qpos[w, 2])
    for k in range(SWEEP_ANGLES):
        a = 2.0 * PI * float(k) / float(SWEEP_ANGLES)
        for ring in range(3):
            r = sweep[SWEEP_RADIUS] * (1.0 - 0.5 * float(ring))
            if ring == 2:
                r = 0.0
            for face in range(3):
                y = sweep_y + sweep[SWEEP_HALF_WIDTH] * float(face - 1)
                local = wp.vec3(
                    sweep[SWEEP_CENTRE_X] + r * wp.cos(a), y, sweep[SWEEP_CENTRE_Z] + r * wp.sin(a)
                )
                p = origin + wp.quat_rotate(q, local)
                if box_distance(p, box_centre, box_yaw, box_half) < WEAPON_SWEEP_MARGIN_M:
                    return 1
    return 0


@wp.func
def on_wedge(
    qpos: wp.array2d(dtype=float), w: int, p: wp.vec3, strike_x: float, strike_half_width: float
) -> int:
    """1 if a contact point is on Mr Stabs Mk2's wedge: ahead of strike_x, near the centre line."""
    q = mj_quat(qpos, w)
    rel = wp.vec3(p[0] - qpos[w, 0], p[1] - qpos[w, 1], p[2] - qpos[w, 2])
    b = wp.quat_rotate_inv(q, rel)
    if b[0] > strike_x and wp.abs(b[1]) < strike_half_width:
        return 1
    return 0


@wp.func
def on_weapon(
    qpos: wp.array2d(dtype=float),
    w: int,
    p: wp.vec3,
    weapon: int,
    strike_x: float,
    strike_half_width: float,
    sweep: wp.vec4,
    sweep_y: float,
    box_centre: wp.vec3,
    box_yaw: float,
    box_half: wp.vec3,
) -> bool:
    if weapon == WEAPON_DISK:
        return weapon_sweep_hit(qpos, w, sweep, sweep_y, box_centre, box_yaw, box_half) == 1
    return on_wedge(qpos, w, p, strike_x, strike_half_width) == 1


@wp.func
def in_opponent_arc(
    p: wp.vec3,
    op: wp.vec3,
    oyaw: float,
    box_half: wp.vec3,
    phi: float,
    arc_half_angle: float,
    rule: int,
) -> bool:
    if wp.abs(phi) >= arc_half_angle:
        return False
    if rule == ARC_FRONT_FACE:
        along = wp.cos(oyaw) * (p[0] - op[0]) + wp.sin(oyaw) * (p[1] - op[1])
        return along > box_half[0] - FRONT_FACE_MARGIN_M
    return True


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
    weapon: int,
    sweep: wp.vec4,
    sweep_y: float,
    geom_size: wp.array2d(dtype=wp.vec3),
    opponent_geom: int,
    arc_rule: int,
    flags: wp.array2d(dtype=float),
):
    """OR this substep's contacts into per-world flag columns (`FLAG_*`)."""
    c = wp.tid()
    if c >= nacon[0]:
        return
    g = contact_geom[c]
    r0 = geom_role[g[0]]
    r1 = geom_role[g[1]]
    other = int(0)
    if r0 == GEOM_ROLE_OURS:
        other = r1
    elif r1 == GEOM_ROLE_OURS:
        other = r0
    else:
        return
    w = contact_world[c]
    if other == GEOM_ROLE_WALL:
        wp.atomic_max(flags, w, FLAG_WALL, 1.0)
        return
    if other == GEOM_ROLE_WALL_BLOCK:
        wp.atomic_max(flags, w, FLAG_WALL_BLOCK, 1.0)
        return
    if other == GEOM_ROLE_HOUSE_BOT:
        wp.atomic_max(flags, w, FLAG_HOUSE_BOT, 1.0)
        return
    if other != GEOM_ROLE_OPPONENT:
        return
    p = contact_pos[c]
    op = mocap_pos[w, opponent_mocap]
    oq = mocap_quat[w, opponent_mocap]
    oyaw = wp.atan2(
        2.0 * (oq[0] * oq[3] + oq[1] * oq[2]), 1.0 - 2.0 * (oq[2] * oq[2] + oq[3] * oq[3])
    )
    ang = wp.atan2(p[1] - op[1], p[0] - op[0]) - oyaw
    phi = ang - 2.0 * PI * wp.floor((ang + PI) / (2.0 * PI))
    wp.atomic_max(flags, w, FLAG_OPPONENT_CONTACT, 1.0)
    box_half = geom_size[w, opponent_geom]
    if in_opponent_arc(p, op, oyaw, box_half, phi, arc_half_angle, arc_rule):
        wp.atomic_max(flags, w, FLAG_ARC_CONTACT, 1.0)
        return
    if on_weapon(
        qpos, w, p, weapon, strike_x, strike_half_width, sweep, sweep_y, op, oyaw, box_half
    ):
        wp.atomic_max(flags, w, FLAG_WEDGE_OUTSIDE_ARC, 1.0)
        wp.atomic_max(flags, w, FLAG_WEDGE_MAX_PHI, wp.abs(phi))


@wp.kernel
def advance(head: wp.array(dtype=int)):
    head[0] = head[0] + 1
