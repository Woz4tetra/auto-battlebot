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
// No deadzone pre-compensation. E7 and E10 exist to measure the deadzone, so
// compensating for it here would erase the thing being measured.
//
// The human driver's SF arm switch is the failsafe. Everything below reduces
// how often it has to be used; none of it replaces it.

import { WebSerialTransport } from './jig.js';

export const OPENTX_FILTERS = [{ usbVendorId: 0x0483, usbProductId: 0x5740 }];
export const TRAINER_MAX = 500;
export const LINEAR_CHANNEL = 0;
export const ANGULAR_CHANNEL = 1;
export const TICK_HZ = 50;
export const WATCHDOG_MS = 100;

export function toTrainer(x) {
    return Math.round(Math.max(-1, Math.min(1, x)) * TRAINER_MAX);
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
        const lin = toTrainer(linear);
        const ang = toTrainer(angular);
        const msg = `trainer ${LINEAR_CHANNEL} ${lin}\r\ntrainer ${ANGULAR_CHANNEL} ${ang}\r\n`;
        try {
            await this.t.write(new TextEncoder().encode(msg));
        } catch {
            await this.disarm('write failed');
            return null;
        }
        return { lin, ang };
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
                    linear: this.linear,
                    angular,
                    trim: this.trim,
                    trainerLin: raw.lin,
                    trainerAng: raw.ang,
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
