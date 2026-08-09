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
        this._pump = null;
        this._closing = false;
    }

    get connected() {
        return this.port !== null;
    }

    async open(baud = JIG_BAUD) {
        if (!navigator.serial) throw new Error('Web Serial unavailable. Use Chrome or Edge over http://localhost.');
        this.port = await navigator.serial.requestPort(
            this.filters.length ? { filters: this.filters } : {},
        );
        await this.port.open({ baudRate: baud });
        // Some CDC stacks hold output until DTR is asserted.
        try {
            await this.port.setSignals({ dataTerminalReady: true, requestToSend: true });
        } catch {
            /* not fatal; not every platform implements setSignals */
        }
        this._pump = this._read();
    }

    async _read() {
        // `_closing` is load-bearing, not belt-and-braces. cancel() makes the
        // pending read resolve done, the inner loop breaks, and the finally
        // releases the lock. Without this flag the outer condition is still
        // true, because close() has not nulled `port` yet, so the loop
        // immediately re-acquires the reader and re-locks readable. port.close()
        // then rejects with the stream locked and the device stays claimed by
        // the tab while the UI happily reports it disconnected.
        while (this.port?.readable && !this._closing) {
            this._reader = this.port.readable.getReader();
            try {
                for (;;) {
                    const { value, done } = await this._reader.read();
                    if (done) break;
                    if (value && this._onData) this._onData(value);
                }
            } catch {
                break; // device unplugged; close() tidies up
            } finally {
                try {
                    this._reader.releaseLock();
                } catch {
                    /* already released */
                }
            }
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
        this._buf = new Uint8Array(0);
        this._collector = null; // {lines, done, isEnd, timer}
        this._binary = null; // {remaining, chunks, done}
        this._queue = Promise.resolve();
        this._streaming = false;
        this.recording = false;
    }

    get connected() {
        return this.t.connected;
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
            this._emit(u.name, { ...u.map(m), tHost: performance.now() });
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
