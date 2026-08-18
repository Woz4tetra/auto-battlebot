// Excitation generators, one per protocol the runbook calls for.
//
// A program is `{durationS, at(t) -> {linear, angular, label}}`. Piecewise
// protocols also expose `segments` so the UI can draw them. The player samples
// at 50 Hz; nothing here knows about serial ports.
//
// Two conventions worth stating once:
//
// - Commands are normalized to [-1, 1]. trainer.js maps to the OpenTX range.
// - `shuttle` alternates the sign of successive repetitions so the robot ends
//   up near where it started. It is the cheapest way to fit a step battery into
//   a short board, because it costs nothing in the fit: a reverse step is a
//   step, and E8 wants both directions anyway.

import { PLANT, MIN_HOLD_S, MIN_COAST_S } from './plant.js';

/** Every generator clamps its hold here, so no caller can ask for a stub run. */
const hold = (s) => Math.max(MIN_HOLD_S, s ?? MIN_HOLD_S);

function piecewise(segments, label = '') {
    const durationS = segments.length ? segments[segments.length - 1].t1 : 0;
    return {
        label,
        durationS,
        segments,
        at(t) {
            for (const s of segments) {
                if (t >= s.t0 && t < s.t1) return s;
            }
            return { linear: 0, angular: 0, label: 'done' };
        },
    };
}

function push(segs, dur, linear, angular, label) {
    const t0 = segs.length ? segs[segs.length - 1].t1 : 0;
    segs.push({ t0, t1: t0 + dur, linear, angular, label });
    return segs;
}

/** Settle time to let a coast finish before the next repetition. */
function coastTime(p = PLANT) {
    return Math.max(MIN_COAST_S, p.delayS + 4 * p.tauDecel + 0.3);
}

/**
 * Scale an amplitude set so its largest magnitude lands on the cap.
 *
 * Scaling, not clamping. Clamping flattens every over-cap level onto the same
 * value, which turns four distinct steps into two and silently destroys the
 * coverage the experiment was recording. Scaling keeps the levels distinct and
 * their ratios intact, which is what a regression through them needs.
 *
 * Under-cap sets are returned untouched: the cap is a ceiling, not a target.
 */
export function capAmplitudes(amplitudes, cap) {
    if (!cap || cap <= 0) return amplitudes;
    const peak = Math.max(...amplitudes.map(Math.abs));
    if (peak <= cap) return amplitudes;
    const k = cap / peak;
    return amplitudes.map((a) => Math.round(a * k * 1000) / 1000);
}

/**
 * E7, E10. Staircase from `from` to `to` in `step`, holding each level.
 *
 * No coast between levels: the staircase is continuous by design, because the
 * deadzone is the level at which motion first appears and returning to zero
 * between levels would double the run length for nothing.
 */
export function staircase({
    channel = 'linear',
    from = 0.01,
    to = 0.1,
    step = 0.01,
    holdS = MIN_HOLD_S,
    sign = 1,
}) {
    const segs = [];
    const h = hold(holdS);
    for (let a = from; a <= to + 1e-9; a += step) {
        const v = Math.round(a * 1000) / 1000;
        push(
            segs,
            h,
            channel === 'linear' ? sign * v : 0,
            channel === 'angular' ? sign * v : 0,
            `${channel} ${(sign * v).toFixed(2)}`,
        );
    }
    push(segs, coastTime(), 0, 0, 'coast');
    return piecewise(segs, `${channel} staircase ${from}-${to}`);
}

/**
 * E8, E11. Amplitude steps with a full coast between each.
 *
 * `holdS` is the knob that trades floor for fit quality, and MIN_HOLD_S is where
 * it stops. Two seconds is over six time constants, which reaches the plateau
 * with room to spare, so max speed reads straight off the trace instead of
 * coming out of a fitted model. Shorter holds are what stage 2 ran, and stage 2
 * could not fit the forward accel constant.
 */
export function steps({
    channel = 'linear',
    amplitudes = [0.25, 0.5, 0.75, 1.0],
    holdS = MIN_HOLD_S,
    sign = 1,
    shuttle = false,
    scale = 1.0,
}) {
    const segs = [];
    const h = hold(holdS);
    let dir = sign;
    for (const a of amplitudes) {
        const v = a * scale * dir;
        push(
            segs,
            h,
            channel === 'linear' ? v : 0,
            channel === 'angular' ? v : 0,
            `${channel} ${v.toFixed(2)}`,
        );
        push(segs, coastTime(), 0, 0, 'coast');
        if (shuttle) dir = -dir;
    }
    return piecewise(segs, `${channel} steps${shuttle ? ' (shuttle)' : ''}`);
}

/** E9. Dedicated coast tails: hold, then drop to zero instantly. */
export function coastTails({
    amplitude = 0.6,
    holdS = MIN_HOLD_S,
    reps = 10,
    sign = 1,
    shuttle = false,
}) {
    const segs = [];
    const h = hold(holdS);
    let dir = sign;
    for (let i = 0; i < reps; i++) {
        push(segs, h, amplitude * dir, 0, `hold ${(amplitude * dir).toFixed(2)}`);
        push(segs, coastTime() + 0.5, 0, 0, 'coast to standstill');
        if (shuttle) dir = -dir;
    }
    return piecewise(segs, `coast tails x${reps}`);
}

/** E5. Pure spins at increasing yaw rate, encoder attached and scrubbing. */
export function spins({ amplitudes = [0.15, 0.25, 0.35, 0.45], holdS = 4.0, sign = 1, angularCap = null }) {
    const segs = [];
    // E5 finds the rate limit, so it has to approach it, but the runbook caps
    // the top rate and the wheel is scrubbing the whole time.
    for (const a of capAmplitudes(amplitudes, angularCap)) {
        push(segs, hold(holdS), 0, a * sign, `spin ${(a * sign).toFixed(2)}`);
        push(segs, MIN_COAST_S, 0, 0, 'settle');
    }
    return piecewise(segs, 'lever-arm spins');
}

/**
 * E13. Coupling grid, linear crossed with angular.
 *
 * Cells are ordered so the linear command alternates sign when shuttling: a
 * grid at full linear amplitude is sixteen steps of floor otherwise.
 */
export function couplingGrid({
    linear = [0.25, 0.5, 0.75, 1.0],
    angular = [0, 0.1, 0.2, 0.3],
    holdS = MIN_HOLD_S,
    shuttle = false,
    scale = 1.0,
    angularCap = 1.0,
}) {
    const segs = [];
    const h = hold(holdS);
    let dir = 1;
    const cells = [];
    const angScaled = capAmplitudes(angular, angularCap);
    for (const l of linear) for (const a of angScaled) cells.push([l, a]);
    for (const [l, a] of cells) {
        const lin = l * scale * dir;
        const ang = a;
        push(segs, h, lin, ang, `lin ${lin.toFixed(2)} ang ${ang.toFixed(2)}`);
        push(segs, coastTime(), 0, 0, 'coast');
        if (shuttle) dir = -dir;
    }
    return piecewise(segs, `coupling grid ${linear.length}x${angular.length}`);
}

/**
 * E14, E15, E16. Maximal-length LFSR, deterministic from the seed.
 *
 * A 60 ms bit period covers roughly 0.2 to 8 Hz, which brackets the 58-78 ms
 * time constants and the 33 ms camera frame period. Seeds are logged so a
 * repetition can be replayed exactly or deliberately varied.
 */
export function prbs({
    channels = ['linear'],
    amplitude = 0.6,
    bitMs = 60,
    durationS = 30,
    seed = 0xace1,
    angularAmplitude = null,
    zeroMean = true,
}) {
    const bits = Math.ceil((durationS * 1000) / bitMs);
    const seq = {};
    let s = seed >>> 0 || 0xace1;
    for (const ch of channels) {
        const out = new Int8Array(bits);
        for (let i = 0; i < bits; i++) {
            // 16-bit maximal LFSR, taps 16,14,13,11
            const bit = ((s >> 0) ^ (s >> 2) ^ (s >> 3) ^ (s >> 5)) & 1;
            s = ((s >> 1) | (bit << 15)) & 0xffff;
            out[i] = bit ? 1 : -1;
        }
        seq[ch] = out;
        s = (s * 1103515245 + 12345) & 0xffff || 0xbeef; // decorrelate channels
    }
    const angAmp = angularAmplitude ?? amplitude;
    return {
        label: `PRBS ${channels.join('+')} ${bitMs} ms`,
        durationS,
        segments: null,
        seed,
        at(t) {
            const i = Math.min(bits - 1, Math.floor((t * 1000) / bitMs));
            const lin = seq.linear ? seq.linear[i] * amplitude : 0;
            const ang = seq.angular ? seq.angular[i] * angAmp : 0;
            return {
                linear: zeroMean ? lin : (lin + amplitude) / 2,
                angular: ang,
                label: `prbs bit ${i}`,
            };
        },
    };
}

/**
 * E17. Logarithmic chirp, 0.2 to 8 Hz.
 *
 * Phase is integrated in closed form so the sweep is continuous. A first-order
 * model with tau = 58 ms has its corner near 2.7 Hz, and the chirp should show
 * that corner where the step fit says it is.
 */
export function chirp({ channel = 'linear', f0 = 0.2, f1 = 8, amplitude = 0.4, durationS = 30 }) {
    const k = Math.log(f1 / f0) / durationS;
    return {
        label: `chirp ${channel} ${f0}-${f1} Hz`,
        durationS,
        segments: null,
        at(t) {
            const phase = (2 * Math.PI * f0 * (Math.exp(k * t) - 1)) / k;
            const v = amplitude * Math.sin(phase);
            return {
                linear: channel === 'linear' ? v : 0,
                angular: channel === 'angular' ? v : 0,
                label: `${(f0 * Math.exp(k * t)).toFixed(2)} Hz`,
            };
        },
    };
}

/** E18, E19. The human drives; the tool only logs what the sticks are doing. */
export function manual({ durationS = 60 }) {
    return {
        label: 'operator driving',
        durationS,
        segments: null,
        manual: true,
        at() {
            return { linear: 0, angular: 0, label: 'operator' };
        },
    };
}

/**
 * Predicted floor a program needs, by integrating the plant against it.
 *
 * Closed-form distance works for a single step. It does not work for a shuttle,
 * a PRBS, or a chirp, and those are exactly the protocols whose footprint is
 * hard to guess. So the whole program gets run through a first-order sim with
 * transport delay and separate accel and decel constants, and the answer is the
 * span between the furthest points reached in each direction.
 */
export function predictFootprint(program, p = PLANT, dt = 0.005) {
    const n = Math.ceil((program.durationS + 2.0) / dt);
    const delaySteps = Math.round(p.delayS / dt);
    const cmdHistory = [];
    let v = 0;
    let x = 0;
    let heading = 0;
    let minX = 0;
    let maxX = 0;
    let peak = 0;
    let peakYaw = 0;

    for (let i = 0; i < n; i++) {
        const t = i * dt;
        const c = t < program.durationS ? program.at(t) : { linear: 0, angular: 0 };
        cmdHistory.push(c);
        const applied = cmdHistory[Math.max(0, cmdHistory.length - 1 - delaySteps)];

        const target = applied.linear * (applied.linear >= 0 ? p.vSsFwd : p.vSsRev);
        const tau = Math.abs(target) > Math.abs(v) ? p.tauAccel : p.tauDecel;
        v += ((target - v) / tau) * dt;

        const w = applied.angular * p.wSs;
        heading += w * dt;
        x += v * dt;
        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        peak = Math.max(peak, Math.abs(v));
        peakYaw = Math.max(peakYaw, Math.abs(w));
    }
    return {
        spanM: maxX - minX,
        forwardM: maxX,
        backwardM: -minX,
        endM: x,
        peakSpeed: peak,
        peakYawRate: peakYaw,
        headingTurns: heading / (2 * Math.PI),
    };
}

/**
 * Peak combined demand a program asks for, and where it asks for it.
 *
 * The transmitter spends a single budget across both axes (`|linear| + |angular|
 * <= 1`, angular first), so a cell that asks for more comes back with its linear
 * term cut. That is what the robot does in a match and the command log records
 * the reduced value, but a grid whose top row quietly loses a third of its
 * linear command is a grid that measured something else. Worth saying before the
 * run, not after.
 */
export function peakDemand(program, dt = 0.01) {
    let peak = 0;
    let label = '';
    for (let t = 0; t < program.durationS; t += dt) {
        const c = program.at(t);
        const d = Math.abs(c.linear ?? 0) + Math.abs(c.angular ?? 0);
        if (d > peak) {
            peak = d;
            label = c.label ?? '';
        }
    }
    return { peak, label, saturates: peak > 1 + 1e-9 };
}

/**
 * Fit a program to a budget by scaling amplitude, retrying until it fits.
 *
 * Amplitude is the last knob for a reason (see plant.js), so `build` should
 * already have shortened the dwell and enabled shuttling before this runs.
 */
export function fitToBudget(build, budgetM, p = PLANT) {
    let scale = 1.0;
    for (let i = 0; i < 12; i++) {
        const program = build(scale);
        const fp = predictFootprint(program, p);
        if (fp.spanM <= budgetM || scale < 0.06) {
            return { program, footprint: fp, scale, fits: fp.spanM <= budgetM };
        }
        scale *= Math.max(0.5, (budgetM / fp.spanM) * 0.95);
    }
    const program = build(scale);
    return { program, footprint: predictFootprint(program, p), scale, fits: false };
}
