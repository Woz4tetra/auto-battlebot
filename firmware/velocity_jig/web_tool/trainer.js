// The drive link: a second Web Serial port to the OpenTX transmitter, plus the
// 50 Hz player that turns a program into commands.
//
// Why the browser drives instead of a separate Python script: command
// timestamps and clock probes then share one performance.now() origin, so the
// only unknown left is the jig offset, which is exactly what TIME measures. A
// separate driver process would stack a second unknown host offset on top of
// the one we are trying to solve, and command-to-motion delay is ~60 ms, which
// is the same size as the error that would introduce.
//
// Wire protocol matches playground/calibration/calib_lib/drive_protocol.py:
// ASCII lines at 115200, `trainer <channel> <value>` with value in [-500, 500].
//
// The human driver's SF arm switch is the failsafe. Everything below reduces
// how often it has to be used; none of it replaces it.

import { WebSerialTransport } from './jig.js';

export const OPENTX_FILTERS = [{ usbVendorId: 0x0483, usbProductId: 0x5740 }];
export const TRAINER_MAX = 500;

// mrs_buff_mk3 runs TankDriveProcessor (config/_common.toml), so the two trainer
// channels carry the left and right wheel, not linear and angular. Driving
// straight forward means putting 1 on both. Excitations are written in body
// axes, so the mix below converts, and it matches src/transmitter/drive_mixing.cpp
// exactly: saturate, reverse angular, then left = lin + ang, right = lin - ang.
//
// Matching the deployed path matters more than it looks. The plant this fits is
// the plant the controller drives through, so any transform sitting between a
// command and the motors has to sit in the same place here or the fit describes
// a vehicle nobody drives.
//
// One transform is deliberately left out: the per-wheel lifted deadzone from the
// same config. E7 and E10 exist to measure the deadzone, and pre-compensating
// for it would erase the thing being measured.
export const LEFT_CHANNEL = 0;
export const RIGHT_CHANNEL = 1;
export const REVERSE_ANGULAR = true; // reverse_angular_channel in config/_common.toml
export const SATURATION_LIMIT = 1.0; // velocity_saturation_limit, |linear| + |angular| <= 1

const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

/**
 * Angular gets priority and linear fills the remaining headroom, per
 * saturate_velocity(). A commanded pair that exceeds the budget comes back
 * reduced, and the reduced value is what gets logged: the fit needs the command
 * the robot saw, not the one the program asked for.
 */
export function saturate(linear, angular, limit = SATURATION_LIMIT) {
    const ang = clamp(angular, -1, 1);
    if (limit > 0) {
        const headroom = Math.max(0, clamp(limit, 0, 1) - Math.abs(ang));
        return { linear: clamp(linear, -headroom, headroom), angular: ang };
    }
    return { linear: clamp(linear, -1, 1), angular: ang };
}

/** Body command to wheel commands. Returns the saturated body pair alongside. */
export function mixToWheels(linear, angular, limit = SATURATION_LIMIT) {
    const body = saturate(linear, angular, limit);
    const ang = REVERSE_ANGULAR ? -body.angular : body.angular;
    return {
        left: body.linear + ang,
        right: body.linear - ang,
        linear: body.linear,
        angular: body.angular,
    };
}

export const TICK_HZ = 50;
export const WATCHDOG_MS = 100;

export function toTrainer(x) {
    return Math.round(clamp(x, -1, 1) * TRAINER_MAX);
}

export class TrainerLink extends EventTarget {
    constructor(transport = null) {
        super();
        this.t = transport ?? new WebSerialTransport(OPENTX_FILTERS);
        this.armed = false;
        this.linear = 0;
        this.angular = 0;
        this.trim = 0; // straight-line trim, added to angular and logged as command
        this._timer = null;
        this._lastSetpoint = 0;
        this._wakeLock = null;
        this._hardStopAt = null;
        this._bound = false;
    }

    get connected() {
        return this.t.connected;
    }

    async connect() {
        await this.t.open(115200);
        await this.t.write(new TextEncoder().encode('telemetry on\r\n'));
        await this.t.write(new TextEncoder().encode('channels on\r\n'));
        this._bindSafety();
        this._emit('status');
    }

    async disconnect() {
        await this.disarm('disconnect');
        try {
            await this.t.close();
        } finally {
            this._emit('status');
        }
    }

    _emit(name, detail = {}) {
        this.dispatchEvent(new CustomEvent(name, { detail }));
    }

    /**
     * Every disarm path, wired once.
     *
     * Tab hidden is the one that matters in practice: a background tab gets its
     * timers throttled to once a second, so the 50 Hz loop stops feeding
     * commands while the last one keeps being obeyed. Disarming on hide turns a
     * stuck-throttle failure into a stop.
     */
    _bindSafety() {
        if (this._bound) return;
        this._bound = true;
        document.addEventListener('visibilitychange', () => {
            if (document.hidden && this.armed) this.disarm('tab hidden');
        });
        window.addEventListener('blur', () => {
            if (this.armed) this.disarm('window blurred');
        });
        window.addEventListener('beforeunload', () => {
            // Best effort. The write may not flush before teardown, which is
            // why the SF switch is the failsafe and this is not.
            if (this.armed) this._writeRaw(0, 0);
        });
        window.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && this.armed) this.disarm('escape key');
        });
    }

    async arm(maxSeconds = 120) {
        if (!this.connected) throw new Error('transmitter not connected');
        this.armed = true;
        this.linear = 0;
        this.angular = 0;
        this._lastSetpoint = performance.now();
        this._hardStopAt = performance.now() + maxSeconds * 1000;
        try {
            this._wakeLock = await navigator.wakeLock?.request('screen');
        } catch {
            /* wake lock is a convenience, not a requirement */
        }
        this._timer = setInterval(() => this._tick(), 1000 / TICK_HZ);
        this._emit('armed');
    }

    async disarm(reason = 'manual') {
        const wasArmed = this.armed;
        this.armed = false;
        clearInterval(this._timer);
        this._timer = null;
        this.linear = 0;
        this.angular = 0;
        await this._writeRaw(0, 0);
        try {
            await this._wakeLock?.release();
        } catch {
            /* ignore */
        }
        this._wakeLock = null;
        if (wasArmed) this._emit('disarmed', { reason });
    }

    /** Setpoint for the next tick. Refreshing it also feeds the watchdog. */
    set(linear, angular) {
        this.linear = linear;
        this.angular = angular;
        this._lastSetpoint = performance.now();
    }

    async _writeRaw(linear, angular) {
        if (!this.connected) return null;
        const wheels = mixToWheels(linear, angular);
        const left = toTrainer(wheels.left);
        const right = toTrainer(wheels.right);
        const msg = `trainer ${LEFT_CHANNEL} ${left}\r\ntrainer ${RIGHT_CHANNEL} ${right}\r\n`;
        try {
            await this.t.write(new TextEncoder().encode(msg));
        } catch {
            await this.disarm('write failed');
            return null;
        }
        return { left, right, linear: wheels.linear, angular: wheels.angular };
    }

    _tick() {
        const now = performance.now();
        if (now - this._lastSetpoint > WATCHDOG_MS) {
            this.disarm('watchdog: no setpoint');
            return;
        }
        if (this._hardStopAt && now > this._hardStopAt) {
            this.disarm('hard timeout');
            return;
        }
        const angular = this.angular + this.trim;
        this._writeRaw(this.linear, angular).then((raw) => {
            if (raw) {
                this._emit('command', {
                    tHost: now,
                    // What the robot saw, after the saturation budget. The
                    // requested pair is kept alongside so a run where the two
                    // differ is visible in the log rather than inferred.
                    linear: raw.linear,
                    angular: raw.angular,
                    linearRequested: this.linear,
                    angularRequested: angular,
                    trim: this.trim,
                    trainerLeft: raw.left,
                    trainerRight: raw.right,
                });
            }
        });
    }
}

/**
 * Play a program and return the command log.
 *
 * Commands are recorded as issued, with host timestamps, non-uniformly spaced.
 * That is fine and deliberate: the fit is a prediction-error method that takes
 * timestamped commands, not a fixed-rate sequence, so jitter in the browser
 * timer costs nothing as long as the timestamps are honest.
 */
export async function play(link, program, opts = {}) {
    const { onProgress, onCommand, signal } = opts;
    if (!link.armed) throw new Error('not armed');
    const commands = [];
    const record = (e) => {
        commands.push(e.detail);
        onCommand?.(e.detail);
    };
    link.addEventListener('command', record);

    const t0 = performance.now();
    try {
        for (;;) {
            const t = (performance.now() - t0) / 1000;
            if (signal?.aborted) break;
            if (!link.armed) break;
            if (t >= program.durationS) break;
            const c = program.at(t);
            if (!program.manual) link.set(c.linear, c.angular);
            else link.set(0, 0); // operator drives; we only keep the clock running
            onProgress?.(t, program.durationS, c);
            await new Promise((r) => setTimeout(r, 10));
        }
    } finally {
        link.set(0, 0);
        link.removeEventListener('command', record);
    }
    return { commands, tStart: t0, tEnd: performance.now() };
}
