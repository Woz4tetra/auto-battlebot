// Straight-line trim: close a slow loop on the jig's live gyro so an open-loop
// run does not veer off a narrow board.
//
// Two constraints that shape everything here:
//
// 1. The trim is logged as a commanded angular value, never applied as a hidden
//    offset. A trimmed linear step is a two-input excitation, and a fit that
//    does not know about the second input will attribute its effect to the
//    first. trainer.js adds `trim` into the angular command and reports the sum
//    in the command log, so the fit sees what the robot saw.
//
// 2. This does not replace E13. It is the low-speed end of the same coupling,
//    measured with one number instead of a grid, and it should be fit as a
//    fraction of the linear command rather than treated as a constant.
//
// Trim runs on the live STREAM at 10 Hz and is never recorded: the console is
// blind during a recording, so a trim pass and a logged run cannot overlap.

import { PLANT } from './plant.js';

const DEFAULTS = {
    linearAmplitude: 0.15,
    kp: 0.004, // per rad/s of yaw rate error
    ki: 0.010, // per rad/s per second; the integral IS the trim estimate
    trimLimit: 0.15,
    stillSeconds: 1.5,
    runSeconds: 5.0,
};

/**
 * Yaw axis from gravity, per the runbook.
 *
 * At rest the accelerometer measures specific force, so it reads +1 g on
 * whichever axis points up. That is a signed vector: it gives the vertical axis
 * and which end is up. The ISM330DHCX puts gyro and accel on one right-handed
 * triad, so up plus the right-hand rule fixes positive yaw as counter-clockwise
 * viewed from above. Projecting the gyro onto it survives a tilted remount,
 * where no single gyro axis is yaw anymore.
 *
 * Requires a still segment: at 50 mm off-axis and 35 rad/s, centripetal
 * acceleration reaches 6.2 g and swamps gravity.
 */
export function yawAxisFromStill(rows) {
    if (!rows.length) return null;
    const s = [0, 0, 0];
    for (const r of rows) for (let i = 0; i < 3; i++) s[i] += r.a[i];
    const m = s.map((x) => x / rows.length);
    const n = Math.hypot(m[0], m[1], m[2]);
    if (!(n > 0)) return null;
    return m.map((x) => x / n);
}

/** Mean gyro over a still segment, in raw counts, per axis. */
export function gyroBiasFromStill(rows) {
    const s = [0, 0, 0];
    for (const r of rows) for (let i = 0; i < 3; i++) s[i] += r.g[i];
    return s.map((x) => x / Math.max(1, rows.length));
}

/** Yaw rate in rad/s from one stream row, given the gravity axis and bias. */
export function yawRate(row, axis, bias, gyroDpsPerLsb = 0.07) {
    let dot = 0;
    for (let i = 0; i < 3; i++) dot += (row.g[i] - bias[i]) * axis[i];
    return (dot * gyroDpsPerLsb * Math.PI) / 180;
}

/**
 * Measure the still reference. Robot stationary, motors disarmed.
 * Returns the gravity axis and the gyro bias the trim loop will subtract.
 */
export async function calibrateStill(jig, opts = {}) {
    const seconds = opts.stillSeconds ?? DEFAULTS.stillSeconds;
    const rows = [];
    await jig.startStream((r) => rows.push(r));
    await new Promise((r) => setTimeout(r, seconds * 1000));
    await jig.stopStream();
    if (rows.length < 5) throw new Error(`still reference too short: ${rows.length} samples`);
    const axis = yawAxisFromStill(rows);
    if (!axis) throw new Error('could not derive gravity axis');
    return { axis, bias: gyroBiasFromStill(rows), samples: rows.length };
}

/**
 * Drive forward at low amplitude and integrate the yaw rate to zero.
 *
 * The loop is deliberately slow. Feedback arrives at 10 Hz through a ~60 ms
 * transport delay, so anything with real bandwidth would oscillate; the job is
 * to find a constant, not to steer. The integral term is the answer, and the
 * proportional term only keeps the robot on the board while it converges.
 */
export async function runTrim(jig, link, reference, opts = {}) {
    const o = { ...DEFAULTS, ...opts };
    const { axis, bias } = reference;
    const history = [];
    let integral = link.trim ?? 0;
    let last = performance.now();
    let latestRate = 0;

    const onRow = (row) => {
        const now = performance.now();
        const dt = Math.min(0.5, (now - last) / 1000);
        last = now;
        const rate = yawRate(row, axis, bias, o.gyroDpsPerLsb ?? 0.07);
        latestRate = rate;

        // Error is "yaw rate should be zero"; the sign convention comes from
        // the gravity axis, so a positive rate means CCW and needs negative
        // angular command to cancel.
        const err = -rate;
        integral += o.ki * err * dt;
        integral = Math.max(-o.trimLimit, Math.min(o.trimLimit, integral));
        const command = integral + o.kp * err;
        link.trim = Math.max(-o.trimLimit, Math.min(o.trimLimit, command));
        history.push({ t: now, rate, integral, command: link.trim });
    };

    await jig.startStream(onRow);
    const t0 = performance.now();
    try {
        while (performance.now() - t0 < o.runSeconds * 1000) {
            if (!link.armed) break;
            if (opts.signal?.aborted) break;
            link.set(o.linearAmplitude, 0);
            await new Promise((r) => setTimeout(r, 20));
        }
    } finally {
        link.set(0, 0);
        await jig.stopStream();
    }

    link.trim = integral; // steady-state feed-forward, no proportional term
    const tail = history.slice(-Math.max(3, Math.floor(history.length / 3)));
    const meanRate = tail.reduce((s, h) => s + h.rate, 0) / Math.max(1, tail.length);

    return {
        trim: integral,
        residualYawRate: meanRate,
        latestRate,
        samples: history.length,
        history,
        // A trim this large is not a trim, it is a mechanical problem.
        suspicious: Math.abs(integral) > 0.6 * o.trimLimit,
        /**
         * Implied wheel speed mismatch, so the number can be compared against
         * the lateral-excursion budget rather than read as a raw command.
         */
        impliedMismatch: (Math.abs(integral) * PLANT.wSs * PLANT.trackWidthM) / PLANT.vSsFwd,
    };
}

/**
 * Post-run rail-rub check.
 *
 * The rail is a backstop, not a guide. Contact shows up as a yaw-rate impulse
 * that lines up with a dip in linear speed, which a clean run does not have.
 * This runs on the live trim history rather than the log, so it is a coarse
 * screen: a flagged pass gets re-trimmed and re-run, and the real check happens
 * offline against the recorded samples.
 */
export function detectRub(history, threshold = 3.0) {
    if (history.length < 6) return { rubbed: false, reason: 'too few samples' };
    const rates = history.map((h) => h.rate);
    const mean = rates.reduce((a, b) => a + b, 0) / rates.length;
    const sd = Math.sqrt(rates.reduce((s, r) => s + (r - mean) ** 2, 0) / rates.length);
    if (sd === 0) return { rubbed: false, reason: 'no variation' };
    const spikes = rates.filter((r) => Math.abs(r - mean) > threshold * sd);
    return {
        rubbed: spikes.length > 0,
        spikes: spikes.length,
        sd,
        reason: spikes.length ? `${spikes.length} yaw-rate impulses over ${threshold} sigma` : 'clean',
    };
}
