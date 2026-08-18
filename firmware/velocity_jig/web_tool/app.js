// UI wiring and the run coach.
//
// The coach walks the runbook's run card one step at a time, doing everything
// it can without being asked: clock probes, the excitation, the encoder-state
// check, the gates. The operator presses A and B on the jig. That is the whole
// manual burden.

import { JigLink, WebSerialTransport } from './jig.js';
import { MockJigTransport, MockTrainerTransport } from './mock.js';
import { TrainerLink, play } from './trainer.js';
import { calibrateStill, runTrim, detectRub } from './trim.js';
import { predictFootprint, fitToBudget, peakDemand } from './excitation.js';
import {
    PLANT,
    lateralExcursion,
    mismatchForHalfWidth,
    usableLength,
    solveHoldS,
    HOLD_LADDER_S,
    MIN_HOLD_S,
    stepDistance,
    encoderPasses,
} from './plant.js';
import {
    EXPERIMENTS,
    BLOCKS,
    getExperiment,
    getVariant,
    buildProgram,
} from './experiments.js';
import {
    newSession,
    newRun,
    applyGates,
    save,
    load,
    toJson,
    toMarkdown,
    download,
    IMU_ODR_HZ,
    GATE_LIMITS,
} from './session.js';

const $ = (id) => document.getElementById(id);

const state = {
    session: load() ?? newSession(),
    jig: null,
    link: null,
    mockJig: null,
    mock: new URLSearchParams(location.search).has('mock'),
    selected: { expId: 'E0', variantId: 'main' },
    active: null, // in-flight run
    diagOn: false,
    audio: null,
    abort: null,
    advance: null, // resolver for "operator pressed the button"
    cable: null, // resolver for "the USB cable moved"
    plan: null,
};

// --- console ------------------------------------------------------------

function logLine(text, cls = '') {
    const el = $('console');
    const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 20;
    const span = document.createElement('span');
    span.className = cls;
    span.textContent = `${text}\n`;
    el.appendChild(span);
    while (el.childNodes.length > 500) el.removeChild(el.firstChild);
    if (atBottom) el.scrollTop = el.scrollHeight;
}

// --- connections --------------------------------------------------------

function attachJig(transport) {
    const jig = new JigLink(transport);
    jig.addEventListener('line', (e) => logLine(e.detail));
    // These two are what the console buttons key off now, so each has to redraw
    // them. The jig announces both without being asked, including a stop it
    // decided on itself, so the buttons come back even on a capture that ended
    // in a way the coach did not plan.
    jig.addEventListener('recording', (e) => {
        logLine(`>> recording ${e.detail.logFile}`, 'evt');
        renderStatus();
    });
    jig.addEventListener('stopped', (e) => {
        logLine(`>> stopped n=${e.detail.samples} dropped=${e.detail.dropped}`, 'evt');
        renderStatus();
    });
    // The cable moving is a step, not a fault. Every experiment that drives the
    // robot needs it out, and the jig keeps logging on its LiPo while it is.
    // So this only reports and redraws: it must not abort the run, which is
    // what an explicit Disconnect click does.
    jig.addEventListener('status', (e) => {
        const { connected, reason } = e.detail;
        if (reason === 'unplugged') {
            state.diagOn = false;
            $('diagToggle').textContent = 'Start stream';
            $('diagOut').textContent = 'Jig unplugged.';
            logLine('jig cable out', 'evt');
        } else if (reason === 'replugged') {
            $('diagOut').textContent = 'Not streaming.';
            logLine('jig cable back in, reconnected', 'evt');
        } else if (!connected && reason) {
            // Only a reconnect that was tried and failed lands here. A plain
            // `connected: false` with no reason is the explicit Disconnect
            // click reporting itself, and calling that a failure would put a
            // red error under a button the operator just pressed on purpose.
            logLine(`jig reconnect failed: ${reason}`);
            $('diagOut').innerHTML = `<p class="note stop">Jig did not reopen. ${reason}</p>`;
        }
        state.cable?.check();
        renderStatus();
    });
    state.jig = jig;
    return jig;
}

/**
 * Abort whatever the coach is waiting on before a port goes away.
 *
 * A run parked on `waitEvent(jig, 'recording')` has no timeout, so closing the
 * port under it would leave the coach waiting on a line that can never arrive.
 * Aborting first closes the run with a reason and leaves a record of it.
 */
function abortForDisconnect(what) {
    if (!state.active) return;
    logLine(`${what} disconnected mid-run: aborting`, 'evt');
    state.link?.disarm(`${what} disconnected`);
    state.abort?.abort();
    state.advance?.reject(new Aborted());
}

async function connectJig() {
    // `state.jig`, not `state.jig.connected`. With the cable out the link still
    // exists, still owns the chosen port, and is still listening for the
    // replug. Treating that click as a fresh connect would build a second
    // transport whose watcher fights the first one for the device on replug,
    // so a click there means give the port up.
    if (state.jig) return disconnectJig();
    try {
        const t = state.mock ? new MockJigTransport() : new WebSerialTransport();
        if (state.mock) state.mockJig = t;
        attachJig(t);
        await state.jig.connect();
        logLine('jig connected', 'evt');
        $('diagOut').textContent = 'Not streaming.';
    } catch (err) {
        // Dropped here as well as the console, because the console is at the
        // bottom of the page and a cancelled port chooser otherwise looks
        // identical to a connect that worked but left the buttons dead.
        state.jig = null;
        const why = /No port selected|cancell?ed/i.test(err.message)
            ? 'No port chosen. Pick the jig in the browser dialog.'
            : err.message;
        $('diagOut').innerHTML = `<p class="note stop">Jig not connected. ${why}</p>`;
        logLine(`jig connect failed: ${err.message}`);
    }
    renderStatus();
}

async function disconnectJig() {
    abortForDisconnect('jig');
    try {
        // Order matters: the stream has to stop before the port closes, or the
        // read loop is left pulling on a closing port. jig.disconnect() does
        // that, but the diagnostics flag lives up here and has to follow.
        state.diagOn = false;
        await state.jig?.disconnect();
        logLine('jig disconnected', 'evt');
    } catch (err) {
        logLine(`jig disconnect failed: ${err.message}`);
    }
    state.jig = null;
    state.mockJig = null;
    $('diagToggle').textContent = 'Start stream';
    $('diagOut').textContent = 'Not streaming.';
    renderStatus();
}

async function disconnectTx() {
    abortForDisconnect('transmitter');
    try {
        // disconnect() disarms before it closes. Closing an armed link would
        // leave the receiver holding the last setpoint until its own failsafe
        // notices, which is exactly the case the watchdog exists to prevent.
        await state.link?.disconnect();
        logLine('transmitter disconnected', 'evt');
    } catch (err) {
        logLine(`transmitter disconnect failed: ${err.message}`);
    }
    state.link = null;
    renderStatus();
}

async function connectTx() {
    if (state.link?.connected) return disconnectTx();
    try {
        const link = new TrainerLink(state.mock ? new MockTrainerTransport() : undefined);
        link.addEventListener('armed', () => {
            logLine('ARMED', 'evt');
            renderStatus();
        });
        link.addEventListener('disarmed', (e) => {
            logLine(`disarmed: ${e.detail.reason}`, 'evt');
            renderStatus();
        });
        if (state.mock) {
            link.addEventListener('command', (e) =>
                state.mockJig?.setMotion(e.detail.linear * PLANT.vSsFwd, e.detail.angular * PLANT.wSs),
            );
        }
        state.link = link;
        await link.connect();
        logLine('transmitter connected', 'evt');
    } catch (err) {
        logLine(`transmitter connect failed: ${err.message}`);
    }
    renderStatus();
}

/**
 * Apply properties to an element, tolerating one that is not there.
 *
 * renderStatus() writes to eight elements in a row, and a plain `$(id).textContent`
 * on a missing one throws and abandons every write after it. That failure is
 * silent and asymmetric: the jig chip is set before the buttons are enabled, so a
 * throw in between leaves a green "connected" chip above three greyed-out buttons,
 * which reads as a logic bug and is not one.
 */
function setEl(id, props) {
    const el = $(id);
    if (!el) {
        logLine(`missing element #${id}: the page and app.js are out of step, reload`);
        return;
    }
    Object.assign(el, props);
}

function renderStatus() {
    const jOn = !!state.jig?.connected;
    const tOn = !!state.link?.connected;

    // Gated on the jig actually recording, not on a run being in flight. Those
    // are very different windows: a run is mostly the coach waiting on the
    // operator, through checklist steps and still holds, and the jig is idle for
    // all of it. Only the capture itself answers BUSY. Gating on the run instead
    // greys these out for minutes at a stretch, including the gap after a run
    // finishes while its notes are still unsaved, which is exactly when you want
    // to check the card or re-probe the clock.
    //
    // Computed and applied first, ahead of the chips, so that the buttons track
    // the port even if something further down this function goes wrong.
    const rec = !!state.jig?.recording;
    const why = !jOn ? 'Connect the jig first' : rec ? 'The jig is recording' : null;
    for (const id of ['cmdList', 'diagToggle']) {
        setEl(id, { disabled: !!why, title: why ?? '' });
    }
    // TIME is deliberately not in that list. The jig's recording branch answers
    // it for real and BUSYs everything else, because a clock probe touches
    // neither the SD card nor the capture path (loop() in src/main.cpp). It is
    // the only console command that works during a capture, which also makes it
    // the only way to catch a link going bad without throwing away the run in
    // progress, so greying it out would remove the one check worth having.
    setEl('cmdTime', {
        disabled: !jOn,
        title: !jOn ? 'Connect the jig first' : rec ? 'Works during a capture' : '',
    });

    // Both chips toggle. The tooltip says which way, because "Jig: connected"
    // reads as a status readout and gives no hint that it is also the button
    // that closes the port.
    // Three states, not two. A cable pulled for a driven run is not a fault and
    // must not read like one: the port is still ours, the capture is still
    // running on the jig's battery, and plugging back in reopens it with no
    // click. Showing that as "disconnected" next to a red chip sends the
    // operator to the connect button mid-run, which drops the adopted port and
    // brings up the chooser for no reason.
    const jOut = !jOn && !!state.jig?.adopted;
    setEl('connectJig', {
        textContent: jOn ? 'Jig: connected' : jOut ? 'Jig: cable out' : 'Jig: disconnected',
        className: `chip ${jOn ? 'ok' : jOut ? 'warn' : 'bad'}`,
        title: jOn
            ? 'Click to disconnect the jig'
            : jOut
              ? 'Plug the cable back in and it reconnects on its own. Click only to give up the port.'
              : 'Click to connect the jig',
    });
    setEl('connectTx', {
        textContent: tOn ? 'TX: connected' : 'TX: disconnected',
        className: `chip ${tOn ? 'ok' : 'bad'}`,
        title: tOn ? 'Click to disconnect the transmitter' : 'Click to connect the transmitter',
    });
    setEl('armState', {
        textContent: state.link?.armed ? 'ARMED' : 'Disarmed',
        className: `chip ${state.link?.armed ? 'armed' : 'idle'}`,
    });
    setEl('mockCable', {
        textContent: state.mockJig && !state.mockJig.connected ? 'Plug USB back in' : 'Unplug USB',
    });
    setEl('sessionId', { textContent: state.session.id + (state.mock ? '  [mock]' : '') });
    setEl('toggleMock', { className: state.mock ? 'chip ok' : 'ghost' });
}

// --- session form -------------------------------------------------------

const FORM = {
    fOperator: 'operator',
    fRobot: 'robot',
    fFloor: 'floorSurface',
    fMount: 'encoderMountSetting',
};

function loadForm() {
    const s = state.session;
    for (const [id, key] of Object.entries(FORM)) $(id).value = s[key] ?? '';
    $('fGuard').checked = s.guardPlatesOn;
    $('fWeapon').checked = s.weaponDisabled;
    $('fGyroRange').value = String(s.imu.gyroRangeDps);
    $('fAccelRange').value = String(s.imu.accelRangeG);
    $('fBoard').value = s.space.boardM;
    $('fHalfWidth').value = s.space.halfWidthM;
    $('fCourse').value = s.space.courseM;
    $('fRateLimit').value = s.encoderRateLimit ?? PLANT.encoderRateLimit;
    $('fNotes').value = s.notes ?? '';
}

function readForm() {
    const s = state.session;
    for (const [id, key] of Object.entries(FORM)) s[key] = $(id).value;
    s.guardPlatesOn = $('fGuard').checked;
    s.weaponDisabled = $('fWeapon').checked;
    s.imu.gyroRangeDps = Number($('fGyroRange').value);
    s.imu.accelRangeG = Number($('fAccelRange').value);
    s.imu.gyroDpsPerLsb = s.imu.gyroRangeDps === 4000 ? 0.14 : 0.07;
    s.space.boardM = Number($('fBoard').value);
    s.space.halfWidthM = Number($('fHalfWidth').value);
    s.space.courseM = Number($('fCourse').value);
    s.space.usableM = usableLength(s.space.boardM);
    s.encoderRateLimit = Number($('fRateLimit').value);
    s.notes = $('fNotes').value;
    save(s);
}

/**
 * Space panel. The floor a full-amplitude step needs is the throttle hold plus
 * the stopping distance: 1.84 m at the 2 s minimum dwell, 2.69 m at 3 s. So a
 * 2.4 m board is the shortest that runs E8 at all, and a 10 ft board is where
 * the dwell stops being the binding constraint.
 */
function renderSpace() {
    const s = state.session;
    const budget = s.space.usableM;
    const rows = [];
    const fmt = (x, n = 2) => x.toFixed(n);

    rows.push(['Usable straight', `${fmt(budget)} m`]);
    for (const holdS of [MIN_HOLD_S, 3.0]) {
        const d = stepDistance(1.0, holdS);
        const fits = d.total <= budget;
        rows.push([
            `Full-amplitude step, ${fmt(holdS, 1)} s dwell`,
            `<span class="${fits ? '' : 'stop'}">${fmt(d.total)} m ${fits ? 'fits' : 'over'}</span>`,
        ]);
    }
    const best = solveHoldS(budget);
    rows.push([
        'Longest dwell that fits at 1.0',
        best ? `${fmt(best, 1)} s` : '<span class="stop">none</span>',
    ]);

    const excursion = lateralExcursion(budget, 0.02, PLANT.trackWidthM);
    const needed = mismatchForHalfWidth(budget, s.space.halfWidthM, PLANT.trackWidthM);
    rows.push([
        'Drift at 2% wheel mismatch',
        `<span class="${excursion > s.space.halfWidthM ? 'stop' : ''}">${fmt(excursion)} m</span>`,
    ]);
    rows.push(['Mismatch that stays on the board', `${fmt(needed * 100, 2)}%`]);

    const passes = encoderPasses(s.space.courseM);
    rows.push([`E4 passes at ${fmt(s.space.courseM)} m`, `${passes} for statistics`]);

    let html = '<table>';
    for (const [k, v] of rows) html += `<tr><td>${k}</td><td>${v}</td></tr>`;
    html += '</table>';

    if (excursion > s.space.halfWidthM) {
        html += `<p class="note flag">Width binds before length here. Drift grows as the square of run
            length, so trim before every open-loop linear run and treat the rail as a backstop, not
            a guide.</p>`;
    }
    $('spaceSummary').innerHTML = html;
}

// --- planning -----------------------------------------------------------

/** Angular command that lands on the E5 rate limit. */
function angularCap(session) {
    const limit = session.encoderRateLimit ?? PLANT.encoderRateLimit;
    return Math.max(0.02, Math.min(1, limit / PLANT.wSs));
}

/**
 * Fit a run to the space available.
 *
 * Order matters and follows PLAN.md: shorten the dwell first, but only down to
 * MIN_HOLD_S, then shuttle, then scale amplitude. Amplitude is last because it
 * costs signal-to-noise and hides any nonlinearity that only appears near full
 * command.
 */
function planRun(exp, variant, session) {
    const budget = session.space.usableM;
    const base = {
        budgetM: budget,
        angularCap: angularCap(session),
        durationS: undefined,
        scale: 1.0,
    };
    if (!exp.program) return { program: null, cfg: base, footprint: null, notes: [] };

    const dwellTunable = ['E8', 'E9', 'E11', 'E13', 'E20'].includes(exp.id);
    const notes = [];
    const attempts = [];
    const dwells = dwellTunable ? HOLD_LADDER_S : [null];
    for (const holdS of dwells) {
        for (const shuttle of [false, true]) {
            const cfg = { ...base, shuttle };
            if (holdS) cfg.holdS = holdS;
            const program = buildProgram(exp, variant, cfg);
            const footprint = predictFootprint(program);
            attempts.push({ cfg, program, footprint, holdS, shuttle });
            if (footprint.spanM <= budget) {
                if (holdS && holdS < HOLD_LADDER_S[0])
                    notes.push(`Dwell shortened to ${holdS.toFixed(1)} s to fit.`);
                if (shuttle) notes.push('Shuttling: successive repetitions alternate direction.');
                return { ...attempts[attempts.length - 1], cfg, program, footprint, notes };
            }
        }
    }

    const last = attempts[attempts.length - 1];
    const fitted = fitToBudget(
        (scale) => buildProgram(exp, variant, { ...last.cfg, scale }),
        budget,
    );
    notes.push(
        `Dwell already at the ${MIN_HOLD_S.toFixed(1)} s floor, so amplitude scaled to ` +
            `${(fitted.scale * 100).toFixed(0)}%.`,
    );
    notes.push('Scaled amplitude weakens the fit and can hide nonlinearity near full command.');
    if (!fitted.fits) notes.push('Still does not fit. Shorten the program or find more floor.');
    return {
        cfg: { ...last.cfg, scale: fitted.scale },
        program: fitted.program,
        footprint: fitted.footprint,
        notes,
        holdS: last.holdS,
        shuttle: last.shuttle,
    };
}

function renderPlan() {
    const exp = getExperiment(state.selected.expId);
    const variant = getVariant(exp, state.selected.variantId);
    const s = state.session;
    state.plan = planRun(exp, variant, s);

    // The transmitter spends one budget across both axes, so a cell that asks
    // for more than that gets its linear term cut before the wheels see it.
    if (state.plan.program) {
        const demand = peakDemand(state.plan.program);
        if (demand.saturates) {
            state.plan.notes.push(
                `Peak demand ${demand.peak.toFixed(2)} exceeds the transmitter budget of 1.00` +
                    `${demand.label ? ` at "${demand.label}"` : ''}. Angular keeps its value and ` +
                    `linear is cut to fit. The command log records what was sent, not what was asked.`,
            );
        }
    }

    $('runTitle').textContent = `${exp.id}. ${exp.name}`;
    $('runMeta').textContent =
        `${variant.label} · ~${exp.durationMin} min · produces ${exp.produces ?? 'n/a'}`;

    const fp = state.plan.footprint;
    const rows = [];
    if (fp) {
        const over = fp.spanM > s.space.usableM;
        rows.push([
            'Predicted footprint',
            `<span class="${over ? 'stop' : ''}">${fp.spanM.toFixed(2)} m of ${s.space.usableM.toFixed(2)} m</span>`,
        ]);
        rows.push(['Peak speed', `${fp.peakSpeed.toFixed(2)} m/s`]);
        if (fp.peakYawRate > 0.01) {
            const limit = s.encoderRateLimit ?? PLANT.encoderRateLimit;
            const scrub = fp.peakYawRate > limit;
            const clip = (s.imu.gyroRangeDps * Math.PI) / 180;
            rows.push([
                'Peak yaw rate',
                `<span class="${scrub || fp.peakYawRate > clip ? 'stop' : ''}">${fp.peakYawRate.toFixed(1)} rad/s</span>`,
            ]);
            rows.push([
                'Gyro clips at',
                `<span class="${fp.peakYawRate > clip ? 'stop' : ''}">${clip.toFixed(1)} rad/s (${s.imu.gyroRangeDps} dps)</span>`,
            ]);
            if (fp.peakYawRate > clip) {
                state.plan.notes.push(
                    `This run saturates the gyro. Set IMU_GYRO_RANGE to 4000 dps and reflash before running it, or the samples are worthless.`,
                );
            }
            if (scrub) {
                state.plan.notes.push(
                    `Peak yaw exceeds the E5 rate limit of ${limit} rad/s. The encoder wheel scrubs sideways the whole time.`,
                );
            }
        }
        rows.push(['Program length', `${state.plan.program.durationS.toFixed(1)} s`]);
        rows.push(['Expected samples', `~${Math.round((state.plan.program.durationS + 20) * IMU_ODR_HZ)}`]);
        const drift = lateralExcursion(fp.spanM, 0.02, PLANT.trackWidthM);
        if (drift > s.space.halfWidthM && fp.spanM > 0.3) {
            rows.push(['Drift at 2% mismatch', `<span class="flag">${drift.toFixed(2)} m</span>`]);
        }
    } else if (exp.id === 'E4') {
        const p = exp.planPasses(s.space.courseM);
        rows.push(['Course', `${p.courseM.toFixed(2)} m`]);
        rows.push(['Passes needed', `${p.minimumPasses}`]);
        rows.push(['Recommended', `${p.recommendedPasses}`]);
    } else {
        rows.push(['Driving', 'not scripted']);
    }

    let html = '<table>';
    for (const [k, v] of rows) html += `<tr><td>${k}</td><td>${v}</td></tr>`;
    html += '</table>';
    for (const n of state.plan.notes) html += `<p class="note flag">${n}</p>`;
    if (exp.id === 'E4') html += `<p class="note">${exp.planPasses(s.space.courseM).note}</p>`;
    if (exp.liveWeapon) html += '<p class="note stop">Live weapon. Highest safety bar in the runbook.</p>';
    $('spaceReport').innerHTML = html;

    const ol = $('procedureSteps');
    ol.innerHTML = '';
    for (const st of exp.steps) {
        const li = document.createElement('li');
        li.textContent = st;
        ol.appendChild(li);
    }
    $('gatesPreview').innerHTML =
        '<strong>Gates:</strong> ' + exp.gates.map((g) => `${g.name} (${g.detail})`).join(' · ');
}

// --- battery list -------------------------------------------------------

function renderBattery() {
    const host = $('experimentList');
    host.innerHTML = '';
    let block = null;
    for (const exp of EXPERIMENTS) {
        if (exp.block !== block) {
            block = exp.block;
            const h = document.createElement('div');
            h.className = 'blockHead';
            h.textContent = BLOCKS[block].title;
            host.appendChild(h);
        }
        const wrap = document.createElement('div');
        wrap.className = 'exp';
        const name = document.createElement('div');
        name.className = 'expName';
        name.textContent = `${exp.id}. ${exp.name}`;
        wrap.appendChild(name);

        const vars = document.createElement('div');
        vars.className = 'vars';
        for (const v of exp.variants) {
            const passes = state.session.runs.filter(
                (r) => r.experimentId === exp.id && r.variant === v.id && r.verdict === 'pass',
            ).length;
            const b = document.createElement('button');
            const sel =
                state.selected.expId === exp.id && state.selected.variantId === v.id ? ' active' : '';
            b.className = `varBtn${passes ? ' done' : ''}${sel}`;
            b.innerHTML = `<span>${v.label}</span><span class="tag">${passes ? `✓${passes}` : ''}</span>`;
            b.onclick = () => {
                state.selected = { expId: exp.id, variantId: v.id };
                $('runner').hidden = false;
                renderBattery();
                renderPlan();
            };
            vars.appendChild(b);
        }
        wrap.appendChild(vars);
        host.appendChild(wrap);
    }
}

// --- diagnostics --------------------------------------------------------

const SATURATION = 32000;

function renderDiag(row) {
    const s = state.session.imu;
    const g = row.g.map((x) => x * s.gyroDpsPerLsb);
    const a = row.a.map((x) => x * s.accelGPerLsb);
    const norm = Math.hypot(...a);
    const u = norm > 0 ? a.map((x) => x / norm) : [0, 0, 0];
    const yawDps = g[0] * u[0] + g[1] * u[1] + g[2] * u[2];
    const sat = [...row.g, ...row.a].some((x) => Math.abs(x) > SATURATION);
    const f = (v, n = 2) => v.toFixed(n).padStart(8);
    $('diagOut').innerHTML = `<table>
        <tr><td>Encoder count</td><td>${row.count}</td></tr>
        <tr><td>Gyro (dps)</td><td>${g.map((x) => f(x, 1)).join(' ')}</td></tr>
        <tr><td>Accel (g)</td><td>${a.map((x) => f(x, 3)).join(' ')}</td></tr>
        <tr><td>|accel|</td><td>${norm.toFixed(3)} g</td></tr>
        <tr><td>Gravity unit vector</td><td>${u.map((x) => f(x, 3)).join(' ')}</td></tr>
        <tr><td>Yaw rate on gravity</td><td>${yawDps.toFixed(1)} dps</td></tr>
        <tr><td>Saturation</td><td class="${sat ? 'stop' : ''}">${sat ? `over ${SATURATION} counts` : 'clear'}</td></tr>
      </table>
      <p class="note">Positive yaw is counter-clockwise viewed from above, because gravity fixes
      which end is up and the gyro shares a right-handed triad with the accelerometer.</p>`;
}

async function toggleDiag() {
    const btn = $('diagToggle');
    if (state.diagOn) {
        state.diagOn = false;
        btn.textContent = 'Start stream';
        await state.jig?.stopStream().catch(() => {});
        return;
    }
    if (!state.jig?.connected) return logLine('diagnostics need the jig connected');
    state.diagOn = true;
    btn.textContent = 'Stop stream';
    try {
        await state.jig.startStream(renderDiag);
    } catch (err) {
        state.diagOn = false;
        btn.textContent = 'Start stream';
        $('diagOut').textContent = err.message;
    }
}

/**
 * Shared guard for the ad-hoc console buttons.
 *
 * Both preconditions are the jig's, not the tool's: a live STREAM blocks the
 * console loop, and during a recording the jig answers BUSY. Rather than let
 * either produce a confusing timeout, stop the stream and refuse mid-capture.
 *
 * `okWhileRecording` is for TIME, the one command the jig's recording branch
 * still answers for real.
 */
async function consoleReady(what, okWhileRecording = false) {
    // Refusals go to diagOut, not just the console panel. The console is at the
    // bottom of the page, well away from the button that was clicked, so a
    // guard that only logs there reads as a dead button.
    const refuse = (why) => {
        $('diagOut').innerHTML = `<p class="note stop">${what}: ${why}</p>`;
        logLine(`${what}: ${why}`);
        return false;
    };
    if (!state.jig?.connected) return refuse('connect the jig first.');
    if (state.jig.recording && !okWhileRecording) {
        return refuse('the jig is recording and answers BUSY. Wait for the capture to end.');
    }
    if (state.diagOn) await toggleDiag();
    return true;
}

/** LIST. What is on the card, and how much room is left for the afternoon. */
async function cmdList() {
    if (!(await consoleReady('LIST'))) return;
    $('diagOut').textContent = 'Listing...';
    try {
        const files = await state.jig.list();
        if (!files.length) {
            $('diagOut').innerHTML = '<p class="note">No files on the card.</p>';
            return;
        }
        const total = files.reduce((s, f) => s + f.size, 0);
        const mb = (b) => (b / 1e6).toFixed(1);
        const rows = files
            .map((f) => `<tr><td>${f.name}</td><td>${mb(f.size)} MB</td></tr>`)
            .join('');
        $('diagOut').innerHTML = `<table>${rows}
            <tr><td><b>${files.length} files</b></td><td><b>${mb(total)} MB</b></td></tr></table>`;
        logLine(`LIST: ${files.length} files, ${mb(total)} MB`, 'evt');
    } catch (err) {
        $('diagOut').textContent = `LIST failed: ${err.message}`;
    }
}

/**
 * TIME. A short clock probe on demand.
 *
 * Fifty probes, not the 200 a run takes: this is the pre-flight check on
 * whether the link is healthy enough to bother starting, and it wants to answer
 * in under a second. The residual is the number worth reading. If it is already
 * over the 2 ms gate here, every run of the afternoon will fail the same gate.
 */
async function cmdTime() {
    if (!(await consoleReady('TIME', true))) return;
    $('diagOut').textContent = 'Probing...';
    try {
        const p = await state.jig.probeClock(50, (i, n) => {
            $('diagOut').textContent = `Probing ${i}/${n}...`;
        });
        const bad = p.residualMs > GATE_LIMITS.clockResidualMs;
        $('diagOut').innerHTML = `<table>
            <tr><td>Host minus jig</td><td>${p.offsetMs.toFixed(3)} ms</td></tr>
            <tr><td>Residual</td><td class="${bad ? 'stop' : ''}">${p.residualMs.toFixed(3)} ms</td></tr>
            <tr><td>Kept</td><td>${p.kept} of ${p.total}</td></tr>
            <tr><td>Median RTT</td><td>${p.rttMedianMs.toFixed(2)} ms</td></tr>
          </table>
          <p class="note">${
              bad
                  ? `Residual is over the ${GATE_LIMITS.clockResidualMs} ms gate. Every run will fail it too. Close other USB traffic and probe again.`
                  : 'Within the gate. This is a spot check, not a logged probe.'
          }</p>`;
        logLine(`TIME: offset ${p.offsetMs.toFixed(3)} ms, residual ${p.residualMs.toFixed(3)} ms`, 'evt');
    } catch (err) {
        $('diagOut').textContent = `TIME failed: ${err.message}`;
    }
}

// --- run list -----------------------------------------------------------

/** Progress against the battery, and what is left in wall-clock minutes. */
function renderProgress() {
    const done = new Set(
        state.session.runs.filter((r) => r.verdict === 'pass').map((r) => `${r.experimentId}/${r.variant}`),
    );
    let total = 0;
    let left = 0;
    let variants = 0;
    let doneVariants = 0;
    for (const e of EXPERIMENTS) {
        const per = e.durationMin / e.variants.length;
        for (const v of e.variants) {
            variants++;
            total += per;
            if (done.has(`${e.id}/${v.id}`)) doneVariants++;
            else left += per;
        }
    }
    $('progress').innerHTML = `<table>
        <tr><td>Runs complete</td><td>${doneVariants}/${variants}</td></tr>
        <tr><td>Estimated remaining</td><td>${Math.round(left)} min of ${Math.round(total)}</td></tr>
      </table>`;
}


function renderRuns() {
    const host = $('runList');
    host.innerHTML = '';
    const runs = state.session.runs;
    $('runCount').textContent = `${runs.filter((r) => r.verdict === 'pass').length}/${runs.length} pass`;
    for (const r of [...runs].reverse()) {
        const el = document.createElement('div');
        el.className = `runItem ${r.verdict ?? 'incomplete'}`;
        const failed = r.gates.filter((g) => g.pass === false);
        const unknown = r.gates.filter((g) => g.pass === null);
        el.innerHTML = `
            <div class="hdr">
              <strong>${r.experimentId} ${r.variantLabel}</strong>
              <span class="muted">${r.logFile ?? '(no file)'}</span>
            </div>
            <div class="muted">${r.samples ?? '?'} samples · dropped ${r.dropped ?? '?'}</div>
            ${failed.map((g) => `<div class="gate fail">✗ ${g.name}: ${g.detail}</div>`).join('')}
            ${unknown.map((g) => `<div class="gate unknown">? ${g.name}: ${g.detail}</div>`).join('')}
            ${r.notes ? `<div class="gate">${r.notes}</div>` : ''}`;
        const del = document.createElement('button');
        del.className = 'del';
        del.textContent = '×';
        del.title = 'Delete this record. The log file on the SD card is not touched.';
        del.onclick = () => deleteRun(r.id);
        el.appendChild(del);
        host.appendChild(el);
    }
}

/**
 * Everything that reads `session.runs`, redrawn.
 *
 * The run list is not the only view of a run. A pass also marks its variant done
 * in the battery and moves the time estimate, so clearing runs without redrawing
 * those leaves green buttons for runs that no longer exist.
 */
function renderAfterRunChange() {
    renderRuns();
    renderProgress();
    if (!state.active) renderPlan();
}

/**
 * Forget one run.
 *
 * The record, not the capture: the jig wrote the log to its own SD card and this
 * only drops the row pointing at it. Reach for it when a run was botched in a way
 * the gates cannot see, an operator error rather than bad data, and you would
 * rather redo it than carry a discard through the sheet.
 */
function deleteRun(id) {
    const r = state.session.runs.find((x) => x.id === id);
    if (!r) return;
    const what = `${r.experimentId} ${r.variantLabel}`.trim();
    const file = r.logFile ? `${r.logFile} stays on the SD card.` : 'This run never named a file.';
    if (!confirm(`Delete the record for ${what}?\n\n${file}`)) return;
    state.session.runs = state.session.runs.filter((x) => x.id !== id);
    save(state.session);
    logLine(`deleted record for ${what}`, 'evt');
    renderAfterRunChange();
}

/**
 * Clear every run, keep the setup.
 *
 * Splitting this from "New session" is the whole point. After a rehearsal on the
 * mock, or a first block run with the encoder mounted wrong, the runs are worthless
 * but the operator, robot, board length and IMU ranges are all still correct and
 * nobody wants to retype them under time pressure.
 */
/**
 * Say no to a destructive action mid-run, and say where the exit is.
 *
 * "Finish or abort the run first" is true and useless: the console is at the
 * bottom of the page, the run panel is at the top, and nothing connects the
 * sentence to the Abort button that resolves it.
 */
function refuseMidRun(what) {
    const r = state.active;
    const where = `${r.experimentId} ${r.variantLabel}`.trim();
    const msg = `Cannot ${what} while ${where} is in progress. Press Abort in the run panel, then save or discard it.`;
    $('diagOut').innerHTML = `<p class="note stop">${msg}</p>`;
    logLine(msg);
}

function clearRuns() {
    if (state.active) return refuseMidRun('clear runs');
    const n = state.session.runs.length;
    if (!n) return logLine('no runs to clear');
    if (
        !confirm(
            `Clear all ${n} run records?\n\n` +
                'Setup fields and log files on the SD card are kept. ' +
                'Export the sheet first if you want a copy.',
        )
    ) {
        return;
    }
    state.session.runs = [];
    save(state.session);
    logLine(`cleared ${n} run records`, 'evt');
    renderAfterRunChange();
}

/** Start over completely: new id, empty runs, setup fields back to defaults. */
function resetSession() {
    if (state.active) return refuseMidRun('start a new session');
    const n = state.session.runs.length;
    if (
        !confirm(
            `Start a new session?\n\nThis clears ${n} run records and every setup field, ` +
                'and issues a new session id. Export first if you want a copy.',
        )
    ) {
        return;
    }
    state.session = newSession();
    save(state.session);
    loadForm();
    readForm();
    logLine(`new session ${state.session.id}`, 'evt');
    renderSpace();
    renderBattery();
    renderAfterRunChange();
    renderStatus();
}

// --- coach primitives ---------------------------------------------------

class Aborted extends Error {}

// Which step the coach is on. Steps are a list of closures, so the number has
// to come from the runner: a step cannot know its own place in a list that is
// built differently for every experiment. It is the operator's only sense of
// how much of the run is left, so it has to be right.
let curStep = 0;

function showStep(n, total, title, detail, big = '', bigClass = '') {
    $('stepIndex').textContent = total ? `Step ${n ?? curStep} of ${total}` : '';
    $('stepTitle').textContent = title;
    $('stepDetail').textContent = detail ?? '';
    $('stepBig').textContent = big;
    $('stepBig').className = `big ${bigClass}`;
}

function setButtons({ go = null, skip = false, abort = true }) {
    $('stepGo').textContent = go ?? 'Continue';
    $('stepGo').hidden = go === null;
    $('stepSkip').hidden = !skip;
    $('stepAbort').hidden = !abort;
}

/** Resolves when the operator presses the primary button. */
function waitAdvance() {
    return new Promise((resolve, reject) => {
        state.advance = { resolve, reject };
    });
}

/**
 * Wait for one of the two lines the operator triggers by hand on the jig.
 *
 * Checks the latch before subscribing. Both lines come from a button press on
 * the jig, and an operator who presses A while the pre-run clock probe is still
 * running fires `recording` well before this step is reached. Subscribing alone
 * would miss it and wait forever for a line that already came, which wedges the
 * run: the capture is live, the console is BUSY, the primary button is hidden,
 * and the only way out is an abort that throws away a capture that is sitting
 * on the card perfectly intact.
 */
async function waitJigLine(name) {
    const early = state.jig.takeLatched(name);
    if (early) {
        logLine(`${name}: you were ahead of the tool, picked up the line you already sent`, 'evt');
        return early;
    }
    const detail = await waitEvent(state.jig, name);
    // Taken live, so drop the copy the latch kept. Otherwise the next step to
    // ask for this line would be handed one that has already been used.
    state.jig.takeLatched(name);
    return detail;
}

/**
 * Wait for the USB cable to come out, or go back in.
 *
 * Advances on the browser's own connect and disconnect events, so the operator
 * moves the cable with both hands on the robot and never has to come back to
 * the laptop to confirm it. Skip step is there because a run that can only be
 * advanced by an event is a run that wedges if the event never fires, and this
 * one depends on the OS noticing a re-enumeration.
 */
async function waitCable(want) {
    if (!!state.jig?.connected === want) return;
    setButtons({ go: null, skip: true });
    try {
        await Promise.race([
            new Promise((resolve, reject) => {
                state.cable = {
                    check: () => {
                        if (!!state.jig?.connected === want) resolve();
                    },
                };
                state.abort.signal.addEventListener('abort', () => reject(new Aborted()));
            }),
            waitAdvance(),
        ]);
    } finally {
        state.cable = null;
    }
}

function waitEvent(target, name) {
    return new Promise((resolve, reject) => {
        const onEvt = (e) => {
            target.removeEventListener(name, onEvt);
            resolve(e.detail);
        };
        target.addEventListener(name, onEvt);
        state.abort.signal.addEventListener('abort', () => {
            target.removeEventListener(name, onEvt);
            reject(new Aborted());
        });
    });
}

/** Short tone, so the countdown can be followed without watching the screen. */
function beep(freq = 880, ms = 90) {
    try {
        state.audio ??= new (window.AudioContext ?? window.webkitAudioContext)();
        const ctx = state.audio;
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.frequency.value = freq;
        gain.gain.value = 0.08;
        osc.connect(gain).connect(ctx.destination);
        osc.start();
        osc.stop(ctx.currentTime + ms / 1000);
    } catch {
        /* audio is a convenience */
    }
}

async function countdown(seconds, title, detail) {
    const t0 = performance.now();
    const start = t0;
    let lastTick = null;
    for (;;) {
        if (state.abort.signal.aborted) throw new Aborted();
        const left = seconds - (performance.now() - t0) / 1000;
        if (left <= 0) break;
        const tick = Math.ceil(left);
        if (tick <= 3 && tick !== lastTick) beep(660, 70);
        lastTick = tick;
        showStep(null, null, title, detail, left.toFixed(1), left < 3 ? 'warn' : '');
        await new Promise((r) => setTimeout(r, 100));
    }
    beep(1320, 160);
    return { start, end: performance.now() };
}

// --- the run sequence ---------------------------------------------------

async function startRun() {
    readForm();
    // The jig stops streaming on any input, so a live diagnostics stream and a
    // clock probe cannot share the port. Diagnostics loses.
    if (state.diagOn) await toggleDiag();
    const exp = getExperiment(state.selected.expId);
    const variant = getVariant(exp, state.selected.variantId);
    const plan = state.plan;
    // Checked once here rather than at each use. Every experiment needs the jig
    // for at least a clock probe, and starting without it would fail several
    // steps in, after the operator has already held still for ten seconds.
    if (!state.jig?.connected) return logLine('connect the jig before starting a run');

    const run = newRun(exp, variant, {
        protocol: plan.program
            ? {
                  name: plan.program.label,
                  durationS: plan.program.durationS,
                  spaceBudgetM: state.session.space.usableM,
                  amplitudeScale: plan.cfg.scale ?? 1,
                  holdS: plan.cfg.holdS ?? null,
                  shuttle: !!plan.cfg.shuttle,
                  seed: plan.program.seed ?? null,
                  predictedDistanceM: plan.footprint?.spanM ?? null,
                  predictedPeakMs: plan.footprint?.peakSpeed ?? null,
              }
            : null,
    });
    state.active = run;
    state.abort = new AbortController();
    // A line left over from the previous run must not satisfy this one's wait.
    state.jig.clearLatched();
    renderStatus(); // LIST and the stream grey out once the capture starts

    const steps = [];
    const add = (fn) => steps.push(fn);

    // Bench experiments produce no log file. They get a checklist walk and, for
    // E1, the probe itself, because a probe set is the whole experiment.
    if (exp.recorded === false) {
        for (const [i, text] of exp.steps.entries()) {
            add(async () => {
                showStep(i + 1, steps.length, `${exp.id} step ${i + 1}`, text);
                setButtons({ go: 'Done', skip: true });
                await waitAdvance();
            });
        }
        if (exp.clockOnly) {
            add(async () => {
                showStep(null, steps.length, 'Probe sets', 'Three sets of 200, recorded for comparison.');
                setButtons({ go: null });
                const sets = [];
                for (let i = 0; i < 3; i++) {
                    sets.push(
                        await state.jig.probeClock(200, (k, n) =>
                            showStep(null, steps.length, `Probe set ${i + 1} of 3`, 'Probing', `${k}/${n}`),
                        ),
                    );
                }
                run.clockPre = sets[0];
                run.clockPost = sets[sets.length - 1];
                run.skewPpm = JigLink.skewPpm(run.clockPre, run.clockPost);
                for (const s of sets) {
                    logLine(`probe: offset ${s.offsetMs.toFixed(2)} ms residual ${s.residualMs.toFixed(3)} ms`, 'evt');
                }
            });
        }
        try {
            for (const [i, fn] of steps.entries()) {
                if (state.abort.signal.aborted) throw new Aborted();
                curStep = i + 1;
                await fn();
            }
        } catch (err) {
            if (!(err instanceof Aborted)) logLine(`run error: ${err.message}`);
            run.notes = err instanceof Aborted ? 'aborted' : err.message;
        }
        // Nothing was recorded, so the log-file gates do not apply. The operator
        // ticking the checklist is the gate.
        run.gates = exp.gates.map((g) => ({ name: g.name, pass: true, detail: g.detail }));
        run.verdict = run.notes === 'aborted' ? 'discard' : 'pass';
        await finishRun(run, exp);
        return;
    }

    // 1. clock probe, pre
    add(async () => {
        showStep(null, steps.length, 'Clock probe, pre', '200 round trips against the jig clock.');
        setButtons({ go: null });
        run.clockPre = await state.jig.probeClock(200, (i, n) =>
            showStep(null, steps.length, 'Clock probe, pre', 'Probing', `${i}/${n}`),
        );
        logLine(
            `clock pre: offset ${run.clockPre.offsetMs.toFixed(2)} ms residual ${run.clockPre.residualMs.toFixed(3)} ms`,
            'evt',
        );
    });

    // 2. optional trim, for open-loop linear runs on a narrow board
    const needsTrim =
        exp.driven &&
        plan.footprint &&
        plan.footprint.spanM > 0.5 &&
        lateralExcursion(plan.footprint.spanM, 0.02, PLANT.trackWidthM) > state.session.space.halfWidthM;
    if (needsTrim) {
        add(async () => {
            showStep(
                4,
                steps.length,
                'Straight-line trim',
                'Closed loop on the live gyro at low speed. Not recorded. The trim is logged as a commanded angular value, not a hidden offset.',
            );
            setButtons({ go: 'Run trim', skip: true });
            await waitAdvance();
            if (!state.link?.connected) throw new Error('transmitter not connected');
            const ref = await calibrateStill(state.jig);
            await state.link.arm(20);
            const res = await runTrim(state.jig, state.link, ref, { signal: state.abort.signal });
            await state.link.disarm('trim complete');
            const rub = detectRub(res.history);
            logLine(
                `trim ${res.trim.toFixed(4)} (implied mismatch ${(res.impliedMismatch * 100).toFixed(2)}%), ${rub.reason}`,
                'evt',
            );
            run.trim = res.trim;
            run.trimMismatch = res.impliedMismatch;
            showStep(
                4,
                steps.length,
                'Trim complete',
                `${res.suspicious ? 'Large trim. Check for a mechanical problem before trusting it. ' : ''}${rub.reason}`,
                res.trim.toFixed(4),
                res.suspicious ? 'warn' : 'ok',
            );
            setButtons({ go: 'Continue' });
            await waitAdvance();
        });
    }

    // 3. start recording
    add(async () => {
        showStep(null, steps.length, 'Press A on the jig', 'The tool waits for the recording line.');
        setButtons({ go: null });
        const evt = await waitJigLine('recording');
        run.logFile = evt.logFile;
        run.tStartHost = evt.tHost;
    });

    // 4. still hold, pre
    if (exp.holds !== false) {
        add(async () => {
            setButtons({ go: null });
            run.holdPre = await countdown(10, 'Hold still', 'Per-run gyro bias estimate. Do not skip.');
        });
    }

    // 6b. cable out, for anything that moves the robot away from the bench
    //
    // The jig runs off its LiPo and USB only overrides and charges it, so the
    // capture keeps running with the cable out. It has to come out here rather
    // than before the still hold: pulling a plug jostles the robot, and the
    // hold is where the per-run gyro bias comes from.
    const movesRobot = exp.motion && exp.motion !== 'none';
    if (movesRobot) {
        add(async () => {
            showStep(
                null,
                steps.length,
                'Unplug the USB cable from the jig',
                'The jig keeps recording on its battery. Leave the cable out until the motion is done, ' +
                    'and check nothing else is tethering the robot.',
                'Cable OUT',
                'warn',
            );
            await waitCable(false);
            logLine('cable out, robot free', 'evt');
        });
    }

    // 5. excitation
    add(async () => {
        if (!plan.program) {
            const passes = exp.planPasses?.(state.session.space.courseM);
            const detail = exp.manualPush
                ? `Motors disarmed. Push ${passes.recommendedPasses} passes along the straightedge, mark to mark, pausing 2 s at each end. Alternate direction.`
                : exp.steps[0];
            showStep(null, steps.length, exp.manualPush ? 'Push the robot' : 'Perform the experiment', detail);
            setButtons({ go: 'Done' });
            await waitAdvance();
            return;
        }
        if (plan.program.manual) {
            showStep(null, steps.length, 'Drive', `Operator drives for ${plan.program.durationS} s.`);
            setButtons({ go: null });
            await countdown(plan.program.durationS, 'Drive', 'Operator drives. Sticks are the command.');
            return;
        }
        showStep(
            null,
            steps.length,
            'Arm and play',
            `${plan.program.label}. Everyone clear. The SF switch is the failsafe.`,
        );
        setButtons({ go: 'Arm and play' });
        await waitAdvance();
        if (!state.link?.connected) throw new Error('transmitter not connected');
        await state.link.arm(plan.program.durationS + 10);
        const res = await play(state.link, plan.program, {
            signal: state.abort.signal,
            onProgress: (t, total, c) =>
                showStep(
                    null,
                    steps.length,
                    c.label ?? 'Playing',
                    `lin ${c.linear.toFixed(2)}  ang ${c.angular.toFixed(2)}`,
                    `${(total - t).toFixed(1)}`,
                ),
        });
        await state.link.disarm('program complete');
        run.commands = res.commands;
    });

    // 6. still hold, post
    if (exp.holds !== false) {
        add(async () => {
            setButtons({ go: null });
            run.holdPost = await countdown(10, 'Hold still', 'Bias drift bound for this run.');
        });
    }

    // 8b. cable back in, before B is pressed
    //
    // Order is not negotiable. The stop summary is a serial line and nothing
    // buffers it, so pressing B with the cable out loses the sample and drop
    // counts for good, and those are two of the gates. Plugging in first costs
    // nothing: the recording is still running and the tail is not used.
    if (movesRobot) {
        add(async () => {
            showStep(
                null,
                steps.length,
                'Plug the USB cable back into the jig',
                'Do this before pressing B. The stop summary only exists on the wire, so a stop with the ' +
                    'cable out loses the sample and dropped counts. Reconnects on its own.',
                'Cable IN',
                'warn',
            );
            await waitCable(true);
            logLine('cable back in', 'evt');
        });
    }

    // 7. stop recording
    add(async () => {
        showStep(null, steps.length, 'Press B on the jig', 'The tool waits for the stop summary.');
        setButtons({ go: null });
        const evt = await waitJigLine('stopped');
        run.samples = evt.samples;
        run.dropped = evt.dropped;
        run.tStopHost = evt.tHost;
        run.durationS = (run.tStopHost - run.tStartHost) / 1000;
    });

    // 8. encoder count, free because startRecording() zeroed it
    add(async () => {
        showStep(null, steps.length, 'Encoder check', 'Reading the count the run accumulated.');
        setButtons({ go: null });
        try {
            run.encoderCountAfter = await state.jig.readEncoderCount();
            logLine(`encoder count after run: ${run.encoderCountAfter}`, 'evt');
        } catch (err) {
            logLine(`encoder check failed: ${err.message}`);
        }
    });

    // 9. clock probe, post
    add(async () => {
        showStep(null, steps.length, 'Clock probe, post', '200 more round trips.');
        setButtons({ go: null });
        run.clockPost = await state.jig.probeClock(200, (i, n) =>
            showStep(null, steps.length, 'Clock probe, post', 'Probing', `${i}/${n}`),
        );
        run.skewPpm = JigLink.skewPpm(run.clockPre, run.clockPost);
    });

    // 10. hand measurements, where the experiment calls for them
    if (exp.measures) {
        add(async () => {
            showStep(null, steps.length, 'Measurements', 'Tape measure values for this run.');
            $('stepExtra').innerHTML = exp.measures
                .map((m) => `<label>${m.label} <input data-measure="${m.key}" type="number" step="0.001"></label>`)
                .join('');
            setButtons({ go: 'Recorded', skip: true });
            await waitAdvance();
            for (const el of $('stepExtra').querySelectorAll('[data-measure]')) {
                const v = Number(el.value);
                if (Number.isFinite(v)) run.measures[el.dataset.measure] = v;
            }
            $('stepExtra').innerHTML = '';
        });
    }

    // run it
    try {
        for (const [i, fn] of steps.entries()) {
            if (state.abort.signal.aborted) throw new Aborted();
            curStep = i + 1;
            await fn();
        }
    } catch (err) {
        await state.link?.disarm('run aborted').catch(() => {});
        if (!(err instanceof Aborted)) logLine(`run error: ${err.message}`);
        run.notes = (run.notes ? `${run.notes} ` : '') + (err instanceof Aborted ? 'aborted' : err.message);
    }

    try {
        // Inside the guard, not before it. applyGates reads a dozen fields off a
        // run that may have been abandoned half-populated, and a throw out here
        // would escape startRun entirely with the flag still set.
        applyGates(run, exp);
        await finishRun(run, exp);
    } finally {
        // finishRun clears this itself, but only after waiting for the operator
        // to save. A throw anywhere in there would otherwise leave the flag set,
        // and Start run, Clear runs and New session all refuse while it is,
        // so the afternoon stops until someone reloads the page.
        state.active = null;
        renderStatus();
    }
}

async function finishRun(run, exp) {
    const failed = run.gates.filter((g) => g.pass === false);
    const unknown = run.gates.filter((g) => g.pass === null);
    showStep(
        null,
        null,
        run.verdict === 'pass' ? 'Run passed' : run.verdict === 'discard' ? 'Discard this run' : 'Incomplete',
        failed.length
            ? failed.map((g) => `${g.name}: ${g.detail}`).join('  |  ')
            : unknown.map((g) => `${g.name}: ${g.detail}`).join('  |  ') || 'All gates clear.',
        run.logFile ?? '',
        run.verdict === 'pass' ? 'ok' : 'warn',
    );
    $('stepExtra').innerHTML =
        '<label>Notes <input id="runNotes" type="text" placeholder="anything odd"></label>' +
        (run.verdict === 'discard'
            ? '<label><input id="runOverride" type="checkbox"> Keep anyway (override)</label>'
            : '');
    setButtons({ go: 'Save run', abort: false });
    await waitAdvance();
    run.notes = ($('runNotes')?.value || run.notes || '').trim();
    if ($('runOverride')?.checked) {
        run.overridden = true;
        run.verdict = 'pass';
    }
    $('stepExtra').innerHTML = '';

    state.session.runs.push(run);
    save(state.session);
    state.active = null;
    renderStatus();
    renderRuns();
    renderProgress();
    renderBattery();
    showStep(null, null, 'Ready', 'Pick the next run.');
    setButtons({ go: 'Start run', abort: false });
}

// --- wiring -------------------------------------------------------------

function bind() {
    $('connectJig').onclick = connectJig;
    $('connectTx').onclick = connectTx;
    $('cmdList').onclick = cmdList;
    $('cmdTime').onclick = cmdTime;
    const stop = (reason) => {
        state.link?.disarm(reason);
        state.abort?.abort();
        state.advance?.reject(new Aborted());
    };
    $('estop').onclick = () => stop('e-stop');
    // Escape stops the run, not only the motors. trainer.js binds Escape too,
    // but only while armed, and a run can be mid-countdown with nothing moving.
    window.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') stop('escape key');
    });
    $('toggleMock').onclick = () => {
        state.mock = !state.mock;
        logLine(`mock ${state.mock ? 'on' : 'off'}: reconnect to apply`);
        $('mockPanel').hidden = !state.mock;
        renderStatus();
    };

    $('stepGo').onclick = () => {
        if (state.active) state.advance?.resolve();
        else startRun().catch((e) => logLine(`run failed: ${e.message}`));
    };
    $('stepSkip').onclick = () => state.advance?.resolve();
    $('stepAbort').onclick = () => {
        state.abort?.abort();
        state.advance?.reject(new Aborted());
        state.link?.disarm('operator abort');
    };

    for (const id of [
        'fOperator', 'fRobot', 'fFloor', 'fMount', 'fGuard', 'fWeapon',
        'fGyroRange', 'fAccelRange', 'fBoard', 'fHalfWidth', 'fCourse', 'fRateLimit', 'fNotes',
    ]) {
        $(id).addEventListener('change', () => {
            readForm();
            // Persisted on every edit, not only when a run completes. Reloading
            // the tab is the recovery for a wedged run, and it should cost the
            // in-flight run and nothing else. Retyping operator, robot, board
            // length and IMU ranges under time pressure is how they get typed
            // wrong, and every one of them lands in the exported sheet.
            save(state.session);
            renderSpace();
            if (!state.active) renderPlan();
        });
    }

    $('exportMd').onclick = () => {
        readForm();
        download(`${state.session.id}-session.md`, toMarkdown(state.session));
    };
    $('exportJson').onclick = () => {
        readForm();
        download(`${state.session.id}-session.json`, toJson(state.session));
    };

    $('clearRuns').onclick = clearRuns;
    $('newSession').onclick = resetSession;

    $('diagToggle').onclick = () => toggleDiag();

    $('mockA').onclick = () => state.mockJig?.pressA();
    $('mockB').onclick = () => state.mockJig?.pressB();
    $('mockCable').onclick = () => {
        const m = state.mockJig;
        if (!m) return;
        if (m.connected) m.unplug();
        else m.replug();
    };
    $('mockEncoder').onchange = (e) => {
        if (state.mockJig) state.mockJig.encoderConnected = e.target.checked;
    };
}

function init() {
    loadForm();
    readForm();
    bind();
    renderStatus();
    renderSpace();
    renderBattery();
    renderPlan();
    renderRuns();
    renderProgress();
    $('mockPanel').hidden = !state.mock;
    $('runner').hidden = false;
    setButtons({ go: 'Start run', abort: false });
    showStep(null, null, 'Ready', 'Connect the jig, pick a run.');
    if (!navigator.serial && !state.mock) {
        logLine('Web Serial unavailable. Use Chrome or Edge, served over http://localhost.');
    }

    // Everything here lives in a module, so a wedged run cannot be looked at
    // from the browser console without a handle to it. Diagnosing one by
    // reasoning about the source instead is slow and gets the answer wrong.
    //
    // `rescue()` is the last resort: it force-ends the run, keeping whatever
    // the coach had already collected, so the panel unlocks without a reload.
    window.jigDebug = {
        state,
        step: () => ({
            active: state.active,
            waiting: state.advance ? 'a button' : 'not a button',
            aborted: !!state.abort?.signal.aborted,
            recording: !!state.jig?.recording,
            latched: state.jig?._latched,
        }),
        rescue: () => {
            state.abort?.abort();
            state.advance?.reject(new Aborted());
            return 'aborted; the run panel should now offer Save run';
        },
    };
}

init();
