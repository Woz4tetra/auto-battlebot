// Web Serial transport and console protocol for the velocity jig.
//
// Framing: the jig speaks newline-terminated ASCII, except GET, which emits a
// SIZE header followed by raw file bytes. The reader therefore keeps a byte
// buffer with an optional binary sink rather than piping through a
// TextDecoderStream.
//
// Unsolicited lines matter as much as replies. "recording <name>" and
// "stopped, n=... dropped=..." arrive on button presses with no request, and
// binding them to the selected experiment is the whole point of this tool. They
// are routed to events and never handed to an in-flight command collector.
//
// See PLAN.md, "Console protocol".

export const JIG_BAUD = 115200;

const UNSOLICITED = [
    { re: /^recording (\S+)/, name: 'recording', map: (m) => ({ logFile: m[1] }) },
    {
        re: /^stopped, n=(\d+) dropped=(\d+)/,
        name: 'stopped',
        map: (m) => ({ samples: Number(m[1]), dropped: Number(m[2]) }),
    },
];

/** Web Serial backing store. Swappable for MockTransport in mock.js. */
export class WebSerialTransport {
    constructor(filters = []) {
        this.filters = filters;
        this.port = null;
        this._reader = null;
        this._onData = null;
        this._onStatus = null;
        this._pump = null;
        this._closing = false;
        this._baud = JIG_BAUD;
        // The port the operator picked, kept across an unplug. The jig runs off
        // its LiPo and USB only overrides and charges it, so pulling the cable
        // for a driven run leaves the capture running and is a normal part of
        // every experiment that moves the robot. Holding the chosen port is
        // what lets the replug reopen silently instead of showing the chooser
        // several times per run.
        this._chosen = null;
        this._chosenInfo = null;
        this._wanted = false;
        this._watching = false;
    }

    get connected() {
        return this.port !== null;
    }

    /** True once a port has been picked, whether or not the cable is in. */
    get adopted() {
        return this._chosen !== null;
    }

    onStatus(cb) {
        this._onStatus = cb;
    }

    /** Does this authorized port look like the one we adopted? */
    _matches(p) {
        if (p === this._chosen) return true;
        const a = p?.getInfo?.() ?? {};
        const b = this._chosenInfo ?? {};
        return (
            a.usbVendorId != null &&
            a.usbVendorId === b.usbVendorId &&
            a.usbProductId === b.usbProductId
        );
    }

    /**
     * Follow the cable.
     *
     * Registered once per transport. Chrome fires these for authorized ports
     * whether or not this tab opened them, so both handlers check the port
     * against the adopted one: the jig and the transmitter are two transports
     * watching the same event.
     */
    _watch() {
        if (this._watching || !navigator.serial?.addEventListener) return;
        this._watching = true;

        navigator.serial.addEventListener('disconnect', (e) => {
            if (!this._matches(e.target)) return;
            this._teardown();
            this._onStatus?.({ connected: false, reason: 'unplugged' });
        });

        navigator.serial.addEventListener('connect', async (e) => {
            if (!this._wanted || this.port || !this._matches(e.target)) return;
            this._chosen = e.target; // Chrome may hand back a fresh object
            // Re-enumeration is not instant. The event can land before the CDC
            // interface is ready to open, and the first attempt then fails on a
            // cable that is perfectly seated, so retry briefly before reporting.
            for (let i = 0; i < 5; i++) {
                try {
                    await this._openPort(this._chosen, this._baud);
                    this._onStatus?.({ connected: true, reason: 'replugged' });
                    return;
                } catch (err) {
                    if (i === 4) {
                        this._onStatus?.({ connected: false, reason: err.message });
                        return;
                    }
                    await new Promise((r) => setTimeout(r, 250));
                }
            }
        });
    }

    /** Drop a port that went away. No close(): the device is already gone. */
    _teardown() {
        this._closing = true;
        try {
            this._reader?.cancel();
        } catch {
            /* the stream errored out with the device */
        }
        this.port = null;
        this._reader = null;
    }

    async _openPort(port, baud) {
        // Let the previous pump finish unwinding first. It holds the readable
        // lock until it does, and on a replug Chrome hands back the same
        // SerialPort object, so starting a second pump on top of it throws
        // "already locked" and the reconnect fails on a cable that is
        // perfectly seated. `_closing` is what makes the old pump stop looping.
        this._closing = true;
        await this._pump?.catch(() => {});
        this._pump = null;
        try {
            await port.open({ baudRate: baud });
        } catch (e) {
            throw new Error(
                `${e.message}. The device is claimed by something else: close other tabs ` +
                    'with this tool open, and any serial monitor.',
            );
        }
        this.port = port;
        this._chosen = port;
        this._chosenInfo = port.getInfo?.() ?? null;
        // Some CDC stacks hold output until DTR is asserted.
        try {
            await this.port.setSignals({ dataTerminalReady: true, requestToSend: true });
        } catch {
            /* not fatal; not every platform implements setSignals */
        }
        this._closing = false;
        this._pump = this._read();
    }

    async open(baud = JIG_BAUD) {
        if (!navigator.serial) throw new Error('Web Serial unavailable. Use Chrome or Edge over http://localhost.');
        this._baud = baud;
        this._watch();
        // The chooser is a user gesture and cannot be raised from the connect
        // event, so anything past the first connect has to reuse the port
        // already adopted. Deliberately not getPorts()[0]: this transport takes
        // no filters when it is the jig, and the transmitter is authorized in
        // the same origin, so picking the first authorized port would happily
        // open the transmitter and call it the jig.
        const port =
            this._chosen ??
            (await navigator.serial.requestPort(
                this.filters.length ? { filters: this.filters } : {},
            ));
        // _openPort assigns this.port only once the port is really open.
        // `connected` is just `port !== null`, so assigning earlier would report
        // a live link for a port that failed to open, and the usual reason it
        // fails to open is that another tab still holds the device.
        await this._openPort(port, baud);
        this._wanted = true;
    }

    /** Forget the adopted port, so the next open() asks again. */
    release() {
        this._wanted = false;
        this._chosen = null;
        this._chosenInfo = null;
    }

    /**
     * Pump bytes out of the port until it closes.
     *
     * The outer loop exists for one case only: a recoverable stream error, such
     * as a framing or parity glitch, after which Web Serial replaces
     * `port.readable` with a fresh stream that has to be re-acquired. That case
     * arrives as a *rejection*.
     *
     * A read that resolves `done` is the opposite: the stream is finished for
     * good, and re-acquiring returns a reader that is done on its first call.
     * Looping on that spins a core at 100% until the tab is closed, which on a
     * laptop means thermal throttling and jitter in the 50 Hz command loop, so
     * it corrupts the very data the afternoon is being spent to collect.
     *
     * `_closing` is load-bearing too. cancel() makes the pending read resolve
     * done and the finally releases the lock, but close() has not nulled `port`
     * yet, so without the flag the loop re-locks `readable` and port.close()
     * rejects with the device still claimed by the tab.
     */
    async _read() {
        let errors = 0;
        while (this.port?.readable && !this._closing) {
            // Held locally as well as on `this`. An unplug tears the transport
            // down from an event handler, which nulls `this._reader` while this
            // loop is still parked inside read(), and the finally below would
            // then throw instead of releasing the lock. The stream stays locked
            // for good after that, so the replug cannot start a new pump.
            const reader = this.port.readable.getReader();
            this._reader = reader;
            let ended = false;
            try {
                for (;;) {
                    const { value, done } = await reader.read();
                    if (done) {
                        ended = true;
                        break;
                    }
                    errors = 0;
                    if (value && this._onData) this._onData(value);
                }
            } catch {
                // Recoverable in principle, but a port that keeps erroring
                // without ever going away would spin just as hard as the done
                // case, so give up after a few in a row.
                if (++errors > 5) ended = true;
            } finally {
                try {
                    reader.releaseLock();
                } catch {
                    /* already released */
                }
            }
            if (ended) break;
        }
    }

    onData(cb) {
        this._onData = cb;
    }

    async write(bytes) {
        if (!this.port?.writable) throw new Error('port not writable');
        const w = this.port.writable.getWriter();
        try {
            await w.write(bytes);
        } finally {
            w.releaseLock();
        }
    }

    /**
     * Release the port back to the OS.
     *
     * Order matters: stop the pump, wait for it to actually exit so the lock is
     * gone, then close. A failure here is reported rather than swallowed, since
     * the symptom of a silent one is a device that stays claimed until the tab
     * is closed, with nothing on screen saying so.
     */
    async close() {
        this._closing = true;
        // An explicit disconnect means the operator wants the port let go, so
        // the watcher must not helpfully reopen it on the next connect event.
        this.release();
        let err = null;
        try {
            await this._reader?.cancel();
        } catch {
            /* the read may already be dead; the pump await below is the gate */
        }
        try {
            await this._pump;
        } catch {
            /* _read swallows its own errors; this is only for the lock */
        }
        try {
            await this.port?.close();
        } catch (e) {
            err = e;
        }
        this.port = null;
        this._reader = null;
        this._pump = null;
        this._closing = false;
        if (err) throw err;
    }
}

/** Parse one "S t_us,count,gx,gy,gz,ax,ay,az" stream row into raw counts. */
export function parseStreamRow(line) {
    if (!line.startsWith('S ')) return null;
    const f = line.slice(2).split(',').map(Number);
    if (f.length !== 8 || f.some(Number.isNaN)) return null;
    return { tUs: f[0], count: f[1], g: [f[2], f[3], f[4]], a: [f[5], f[6], f[7]] };
}

export class JigLink extends EventTarget {
    constructor(transport) {
        super();
        this.t = transport;
        this.t.onData((bytes) => this._feed(bytes));
        this.t.onStatus?.((s) => this._onLink(s));
        this._buf = new Uint8Array(0);
        this._collector = null; // {lines, done, isEnd, timer}
        this._binary = null; // {remaining, chunks, done}
        this._queue = Promise.resolve();
        this._streaming = false;
        this.recording = false;
        // Last `recording` / `stopped` line seen, held until someone takes it.
        //
        // `_emit` is fire-and-forget, so a line that arrives while nothing is
        // listening is gone. Both of these are announced by the operator
        // pressing a button on the jig, and the operator does not wait for the
        // tool: pressing A during the pre-run clock probe, which is 200 round
        // trips long, fires `recording` seconds before the coach subscribes to
        // it. The coach then waits forever for a line that already came, with
        // the capture running and no way out but abort.
        this._latched = { recording: null, stopped: null };
    }

    /**
     * The cable came out or went back in, without anyone clicking anything.
     *
     * An unplug is not an error here. It is a step in every experiment that
     * moves the robot. What it must not do is leave a command waiting out its
     * four second timeout on a port that is physically gone, so the in-flight
     * collector is failed at once and the partial line buffer is dropped: a
     * reply cut in half by an unplug must not be glued to the next reply.
     */
    _onLink({ connected, reason }) {
        if (!connected) {
            this._streaming = false;
            this._buf = new Uint8Array(0);
            this._binary = null;
            this._collector?.fail(new Error('cable unplugged during the command'));
        }
        this._emit('status', { connected, reason });
    }

    /** Take a line that arrived before anyone was listening. Consumes it. */
    takeLatched(name) {
        const v = this._latched[name] ?? null;
        this._latched[name] = null;
        return v;
    }

    /** Drop both latches, so one run cannot consume the previous run's line. */
    clearLatched() {
        this._latched = { recording: null, stopped: null };
    }

    get connected() {
        return this.t.connected;
    }

    /**
     * A port has been picked, whether or not the cable is in right now.
     *
     * Adopted-but-not-connected is the normal state for half of every driven
     * run, and it is worth telling apart from a jig that was never connected:
     * one reconnects by itself when the cable goes back in, the other needs a
     * click and a trip through the browser's port chooser.
     */
    get adopted() {
        return this.t.adopted ?? this.t.connected;
    }

    async connect() {
        await this.t.open(JIG_BAUD);
        this._emit('status', { connected: true });
    }

    async disconnect() {
        await this.stopStream().catch(() => {});
        try {
            await this.t.close();
        } finally {
            // The port is released either way, so the UI has to be told either
            // way. Reporting still-connected after a close failure would leave
            // the chip lying about a device the tab no longer holds.
            this._emit('status', { connected: false });
        }
    }

    _emit(name, detail) {
        this.dispatchEvent(new CustomEvent(name, { detail }));
    }

    // --- framing ---------------------------------------------------------

    _feed(chunk) {
        const merged = new Uint8Array(this._buf.length + chunk.length);
        merged.set(this._buf);
        merged.set(chunk, this._buf.length);
        this._buf = merged;

        // A binary sink (GET payload) consumes bytes ahead of line framing.
        if (this._binary) {
            const take = Math.min(this._binary.remaining, this._buf.length);
            if (take > 0) {
                this._binary.chunks.push(this._buf.slice(0, take));
                this._binary.remaining -= take;
                this._buf = this._buf.slice(take);
            }
            if (this._binary.remaining === 0) {
                const b = this._binary;
                this._binary = null;
                b.done(b.chunks);
            }
            if (this._buf.length === 0) return;
        }

        for (;;) {
            const nl = this._buf.indexOf(0x0a);
            if (nl < 0) break;
            const raw = this._buf.slice(0, nl);
            this._buf = this._buf.slice(nl + 1);
            const line = new TextDecoder().decode(raw).replace(/\r$/, '');
            this._line(line);
            if (this._binary) {
                // SIZE just armed the sink; re-enter to drain the payload.
                this._feed(new Uint8Array(0));
                return;
            }
        }
    }

    _line(line) {
        this._emit('line', line); // raw console view sees everything

        for (const u of UNSOLICITED) {
            const m = line.match(u.re);
            if (!m) continue;
            if (u.name === 'recording') this.recording = true;
            if (u.name === 'stopped') this.recording = false;
            const detail = { ...u.map(m), tHost: performance.now() };
            // Latched before it is emitted, and left latched even when a
            // listener takes it live: whoever consumes it clears the latch.
            // tHost matters here. It is the run's start or stop time, so an
            // early press has to keep the timestamp of the press rather than
            // the moment the coach got around to noticing.
            this._latched[u.name] = detail;
            this._emit(u.name, detail);
            return; // never fed to a command collector
        }

        if (this._collector) this._collector.push(line);
    }

    // --- commands --------------------------------------------------------

    /**
     * Serialize a command and collect reply lines until isEnd matches.
     * Unsolicited lines are filtered out upstream, so a button press during a
     * command cannot corrupt the reply.
     */
    command(text, isEnd, timeoutMs = 4000) {
        const run = async () => {
            const lines = [];
            const result = new Promise((resolve, reject) => {
                const timer = setTimeout(() => {
                    this._collector = null;
                    reject(new Error(`timeout waiting for reply to ${text.trim()}`));
                }, timeoutMs);
                this._collector = {
                    push: (l) => {
                        lines.push(l);
                        if (isEnd(l, lines)) {
                            clearTimeout(timer);
                            this._collector = null;
                            resolve(lines);
                        }
                    },
                    // Called when the cable goes, so the caller hears about it
                    // now instead of waiting out the full timeout on a port
                    // that is physically gone.
                    fail: (err) => {
                        clearTimeout(timer);
                        this._collector = null;
                        reject(err);
                    },
                };
            });
            await this.t.write(new TextEncoder().encode(text));
            return result;
        };
        this._queue = this._queue.then(run, run);
        return this._queue;
    }

    async list() {
        const lines = await this.command('LIST\n', (l) => l === 'END');
        return lines
            .filter((l) => l.startsWith('F '))
            .map((l) => {
                const p = l.slice(2).split(' ');
                return { name: p[0], size: Number(p[1]) };
            });
    }

    async del(name) {
        const lines = await this.command(`DEL ${name}\n`, (l) => l === 'OK' || l.startsWith('ERR'));
        return lines[lines.length - 1] === 'OK';
    }

    /** Pull a log file. Slow over CDC; PLAN.md says pull the SD card for bulk. */
    async get(name, onProgress) {
        const run = async () => {
            const chunks = await new Promise((resolve, reject) => {
                const timer = setTimeout(() => reject(new Error('GET timeout')), 600000);
                this._collector = {
                    push: (l) => {
                        if (l.startsWith('ERR')) {
                            clearTimeout(timer);
                            this._collector = null;
                            reject(new Error(l));
                            return;
                        }
                        const m = l.match(/^SIZE (\d+)/);
                        if (!m) return;
                        this._collector = null;
                        const total = Number(m[1]);
                        this._binary = {
                            remaining: total,
                            chunks: [],
                            done: (cs) => {
                                clearTimeout(timer);
                                resolve(cs);
                            },
                        };
                        if (onProgress) {
                            const tick = setInterval(() => {
                                if (!this._binary) return clearInterval(tick);
                                onProgress(total - this._binary.remaining, total);
                            }, 200);
                        }
                    },
                };
            });
            let n = 0;
            for (const c of chunks) n += c.length;
            const out = new Uint8Array(n);
            let o = 0;
            for (const c of chunks) {
                out.set(c, o);
                o += c.length;
            }
            return out;
        };
        this._queue = this._queue.then(run, run);
        return this._queue;
    }

    // --- clock probe -----------------------------------------------------

    /**
     * NTP-style offset against the jig's time_us_64() base.
     *
     * Keeps the lowest decile by round trip: USB Full Speed polls in 1 ms
     * frames, so most spread is host scheduling and the fastest probes are the
     * ones whose transport was most symmetric. Skew is deliberately not fit
     * here. A burst spans ~1 s and 30 ppm over that is 0.03 ms, far under the
     * noise; it is fit across a run from the pre and post bursts instead.
     * PLAN.md, "Clock probe".
     */
    async probeClock(n = 200, onProgress) {
        const probes = [];
        for (let i = 0; i < n; i++) {
            const t0 = performance.now();
            let lines;
            try {
                lines = await this.command('TIME\n', (l) => l.startsWith('TIME '), 1000);
            } catch {
                continue; // a dropped probe is not a failed burst
            }
            const t1 = performance.now();
            const m = lines[lines.length - 1].match(/^TIME (\d+) (\d+)/);
            if (!m) continue;
            probes.push({
                rtt: t1 - t0,
                hostMid: (t0 + t1) / 2,
                jigMid: (Number(m[1]) + Number(m[2])) / 2000, // us -> ms
            });
            if (onProgress && i % 10 === 0) onProgress(i, n);
        }
        if (probes.length < 8) throw new Error(`clock probe failed: only ${probes.length} replies`);

        probes.sort((a, b) => a.rtt - b.rtt);
        const keep = probes.slice(0, Math.max(4, Math.ceil(probes.length / 10)));
        const offsets = keep.map((p) => p.hostMid - p.jigMid).sort((a, b) => a - b);
        const median = offsets[Math.floor(offsets.length / 2)];
        const residualMs = Math.sqrt(
            offsets.reduce((s, o) => s + (o - median) ** 2, 0) / offsets.length,
        );
        return {
            offsetMs: median,
            residualMs,
            kept: keep.length,
            total: probes.length,
            rttMedianMs: keep[Math.floor(keep.length / 2)].rtt,
            atHostMs: keep[0].hostMid,
            atJigMs: keep[0].jigMid,
        };
    }

    /** Skew across a run, in ppm. Pre and post are probeClock() results. */
    static skewPpm(pre, post) {
        const span = post.atJigMs - pre.atJigMs;
        if (!(span > 0)) return null;
        return ((post.offsetMs - pre.offsetMs) / span) * 1e6;
    }

    // --- stream ----------------------------------------------------------

    /**
     * Live 10 Hz sample rows. Idle only: during a recording the jig answers
     * BUSY to everything but TIME. STREAM blocks the console loop, so it can
     * never overlap a clock probe and both go through the same queue.
     */
    async startStream(onRow) {
        if (this._streaming) return;
        const run = async () => {
            this._streaming = true;
            this._collector = {
                push: (l) => {
                    if (l === 'END') {
                        this._collector = null;
                        this._streaming = false;
                        return;
                    }
                    const row = parseStreamRow(l);
                    if (row) onRow(row);
                },
            };
            await this.t.write(new TextEncoder().encode('STREAM\n'));
        };
        this._queue = this._queue.then(run, run);
        return this._queue;
    }

    async stopStream() {
        if (!this._streaming) return;
        await this.t.write(new TextEncoder().encode('\n'));
        const t0 = performance.now();
        while (this._streaming && performance.now() - t0 < 1500) {
            await new Promise((r) => setTimeout(r, 20));
        }
        this._streaming = false;
        this._collector = null;
    }

    get streaming() {
        return this._streaming;
    }

    /**
     * Net encoder counts for the run that just finished.
     *
     * startRecording() zeroes g_encCount and nothing resets it after, so the
     * free-running count read right after button B is that run's total. This is
     * the automatic encoder-state check from PLAN.md.
     */
    async readEncoderCount(windowMs = 600) {
        let last = null;
        await this.startStream((row) => {
            last = row.count;
        });
        await new Promise((r) => setTimeout(r, windowMs));
        await this.stopStream();
        return last;
    }
}
