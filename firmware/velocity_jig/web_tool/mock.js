// Simulated jig and transmitter, so the whole flow can be exercised without
// hardware and every gate path can be driven on purpose.
//
// The jig mock answers the real console protocol byte for byte, including the
// awkward parts: STREAM stops on any input, button A during a stream emits END
// then "recording <name>", and TIME is the only command answered while
// recording. It also carries a deliberate clock offset and skew so the probe
// has something real to recover.

const enc = new TextEncoder();

export class MockJigTransport {
    constructor(opts = {}) {
        this.connected = false;
        this.adopted = false;
        this._onData = null;
        this._onStatus = null;
        this._line = '';
        this._streamTimer = null;
        this._recording = false;
        this._fileIndex = 0;
        this._files = [];

        // Truth the clock probe should recover, as host-minus-jig.
        //
        // Negative, and it has to be: the jig is plugged in and booted before
        // the page is opened, so its uptime leads `performance.now()`. A
        // positive offset would put the jig's clock before its own boot, and
        // TIME reports unsigned microseconds, so it could not be sent on the
        // wire at all.
        this.offsetMs = opts.offsetMs ?? -1234.567;
        this.skewPpm = opts.skewPpm ?? 30;
        this.replyJitterMs = opts.replyJitterMs ?? 1.2;

        // Truth the encoder gate should see. The wheel is always mounted, so
        // this simulates the one failure left: a connector that fell off.
        this.encoderConnected = opts.encoderConnected ?? true;
        this.count = 0;
        this.dropped = 0;
        this.samples = 0;
        this._t0 = performance.now();
        this._recStart = 0;
        this._motion = { v: 0, w: 0 };
    }

    async open() {
        this.connected = true;
        this.adopted = true;
    }
    onData(cb) {
        this._onData = cb;
    }
    onStatus(cb) {
        this._onStatus = cb;
    }
    async close() {
        this.connected = false;
        this.adopted = false;
        clearInterval(this._streamTimer);
        this._streamTimer = null;
    }

    /**
     * Pull the USB cable.
     *
     * The jig runs off its LiPo, so the recording keeps going and the sample
     * count keeps climbing. Only the wire goes away. Rehearsing this in mock is
     * worth the twenty lines: every experiment that moves the robot needs the
     * cable out, and finding out on the day that the tool wedges when it does
     * costs an afternoon that cannot be rerun.
     */
    unplug() {
        if (!this.connected) return;
        this.connected = false;
        clearInterval(this._streamTimer);
        this._streamTimer = null;
        this._line = '';
        this._onStatus?.({ connected: false, reason: 'unplugged' });
    }

    /** Plug it back in. Reopens by itself, the same as the real transport. */
    replug() {
        if (this.connected) return;
        this.connected = true;
        this._onStatus?.({ connected: true, reason: 'replugged' });
    }

    /**
     * The jig's clock, in microseconds since its boot.
     *
     * Built so that `hostMid - jigMid` lands on `offsetMs`, which is what
     * probeClock() reports. Anchoring on `_t0` instead would make the recovered
     * offset depend on when the page happened to load, and the probe would look
     * correct while measuring nothing in particular.
     *
     * Skew is applied to elapsed time only, so it accumulates across a run the
     * way a real crystal error does. It comes back out of `JigLink.skewPpm()`
     * negated: a jig clock running fast shrinks host-minus-jig over the run.
     */
    _jigUs() {
        const elapsed = performance.now() - this._t0;
        const jigMs = elapsed * (1 + this.skewPpm / 1e6) + (this._t0 - this.offsetMs);
        return Math.round(jigMs * 1000);
    }

    _send(text) {
        // Nothing crosses a cable that is out. This is the whole point of
        // rehearsing the unplug: a `stopped` line sent with the cable out is
        // gone for good, and that is exactly why the coach asks for the replug
        // before B rather than after it.
        if (!this.connected) return;
        // Reply latency, so lowest-decile filtering has something to filter.
        const delay = Math.random() * this.replyJitterMs;
        setTimeout(() => this._onData?.(enc.encode(text)), delay);
    }

    async write(bytes) {
        const s = new TextDecoder().decode(bytes);
        for (const ch of s) {
            if (ch === '\n') {
                const cmd = this._line.trim();
                this._line = '';
                this._handle(cmd);
            } else if (ch !== '\r') {
                this._line += ch;
            }
        }
        if (s.includes('\n') === false && s.length) this._stopStream();
    }

    _handle(cmd) {
        if (this._streamTimer) {
            this._stopStream();
            if (cmd === '') return;
        }
        if (cmd === 'TIME') {
            const rx = this._jigUs();
            this._send(`TIME ${rx} ${rx + 24}\n`);
            return;
        }
        if (this._recording) {
            this._send('BUSY\n');
            return;
        }
        if (cmd === 'LIST') {
            for (const f of this._files) this._send(`F ${f.name} ${f.size}\n`);
            this._send('END\n');
        } else if (cmd === 'STREAM') {
            this._send('STREAM\n');
            this._streamTimer = setInterval(() => this._streamRow(), 100);
        } else if (cmd.startsWith('DEL ')) {
            const n = cmd.slice(4);
            const before = this._files.length;
            this._files = this._files.filter((f) => f.name !== n);
            this._send(this._files.length < before ? 'OK\n' : 'ERR\n');
        } else if (cmd.startsWith('GET ')) {
            const f = this._files.find((x) => x.name === cmd.slice(4));
            if (!f) return this._send('ERR no file\n');
            this._send(`SIZE ${f.size}\n`);
            this._send('x'.repeat(f.size));
            this._send('\nEND\n');
        } else if (cmd !== '') {
            this._send('ERR unknown\n');
        }
    }

    _stopStream() {
        if (!this._streamTimer) return;
        clearInterval(this._streamTimer);
        this._streamTimer = null;
        this._send('END\n');
    }

    _streamRow() {
        const { v, w } = this._motion;
        if (this.encoderConnected) this.count += Math.round(v * 100);
        // Gravity on +z, yaw rate on +z, plus a little noise and a bias.
        const g = [
            Math.round(randn() * 20),
            Math.round(randn() * 20),
            Math.round((w * 57.2958) / 0.07 + 14 + randn() * 20),
        ];
        const a = [Math.round(randn() * 30), Math.round(randn() * 30), Math.round(1 / 0.000244)];
        this._send(`S ${this._jigUs()},${this.count},${g.join(',')},${a.join(',')}\n`);
    }

    // --- simulated front-panel buttons -----------------------------------

    pressA() {
        if (this._recording) return;
        this._stopStream();
        this._recording = true;
        this.count = 0;
        this.dropped = 0;
        this._recStart = performance.now();
        const name = `LOG-${this._fileIndex++}.TXT`;
        this._curName = name;
        this._send(`recording ${name}\n`);
    }

    pressB() {
        if (!this._recording) return;
        this._recording = false;
        const dur = (performance.now() - this._recStart) / 1000;
        this.samples = Math.round(dur * 1660);
        this._files.push({ name: this._curName, size: this.samples * 40 });
        this._send(`stopped, n=${this.samples} dropped=${this.dropped}\n`);
    }

    /** Drive the simulated robot, so encoder counts and gyro reflect commands. */
    setMotion(v, w) {
        this._motion = { v, w };
        if (this._recording && this.encoderConnected) this.count += Math.round(v * 10);
    }
}

export class MockTrainerTransport {
    constructor() {
        this.connected = false;
        this.sent = [];
        this.linear = 0;
        this.angular = 0;
        this.onCommand = null;
    }
    async open() {
        this.connected = true;
    }
    onData() {}
    async write(bytes) {
        const s = new TextDecoder().decode(bytes);
        this.sent.push(s);
        for (const line of s.split('\n')) {
            const m = line.match(/^trainer (\d) (-?\d+)/);
            if (!m) continue;
            if (m[1] === '0') this.linear = Number(m[2]);
            if (m[1] === '1') this.angular = Number(m[2]);
        }
        this.onCommand?.(this.linear, this.angular);
    }
    async close() {
        this.connected = false;
    }
}

function randn() {
    let u = 0;
    let v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
