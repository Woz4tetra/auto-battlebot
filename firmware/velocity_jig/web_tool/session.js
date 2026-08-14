// Session and run records, gate evaluation, persistence, export.
//
// The point of the tool is that a run writes itself down. The jig already
// announces the filename, the sample count, and the dropped count without being
// asked, and the tool already knows the clock probes, the program it played, and
// when the still holds started and ended. Free-text notes are the only field a
// human types.
//
// Gates run the moment a run closes, while the robot is still on the floor and
// re-running is cheap. The 2026-07-03 AprilTag session shipped parameters fit
// from one surviving segment because bad data was not caught at capture time.

import { encoderStateFor } from './experiments.js';

const STORE_KEY = 'velocity_jig_session_v1';
export const IMU_ODR_HZ = 1660;

export const GATE_LIMITS = {
    clockResidualMs: 2.0,
    holdSecondsMin: 9.0,
    encoderAttachedMinCount: 100,
    sampleShortfallFrac: 0.9,
    biasDriftDegPerS: 0.05, // offline only; the tool cannot see samples during a run
    skewPpmMax: 200,
};

export function newSession(fields = {}) {
    return {
        id: fields.id ?? `S-${new Date().toISOString().slice(0, 16).replace(/[:T]/g, '')}`,
        startedAt: new Date().toISOString(),
        operator: fields.operator ?? '',
        robot: fields.robot ?? '',
        floorSurface: fields.floorSurface ?? '',
        encoderMountSetting: fields.encoderMountSetting ?? '',
        guardPlatesOn: false,
        weaponDisabled: false,
        imu: { gyroDpsPerLsb: 0.07, accelGPerLsb: 0.000244, gyroRangeDps: 2000, accelRangeG: 8 },
        space: { boardM: 2.5, usableM: 2.0, halfWidthM: 0.25, courseM: 2.0 },
        encoderRateLimit: null, // filled from E5
        dragVerdict: '', // filled from E12
        notes: '',
        runs: [],
    };
}

export function newRun(exp, variant, fields = {}) {
    return {
        id: fields.id ?? `R${Date.now().toString(36)}`,
        experimentId: exp.id,
        experimentName: exp.name,
        variant: variant?.id ?? 'main',
        variantLabel: variant?.label ?? '',
        encoder: encoderStateFor(exp, variant),
        logFile: null,
        samples: null,
        dropped: null,
        tStartHost: null,
        tStopHost: null,
        durationS: null,
        holdPre: null,
        holdPost: null,
        // Straight-line trim, as a commanded angular value. A trimmed run is a
        // two-input excitation and the fit has to be told so.
        trim: null,
        trimMismatch: null,
        clockPre: null,
        clockPost: null,
        skewPpm: null,
        encoderCountAfter: null,
        protocol: fields.protocol ?? null,
        commands: [],
        measures: {},
        gates: [],
        verdict: null,
        overridden: false,
        notes: '',
    };
}

/**
 * Everything the tool can check without reading the log file.
 *
 * Deliberately excluded: gyro bias drift between the pre and post still holds.
 * The console is blind during a recording, so the samples are not available
 * until the file is pulled. It stays on the offline checklist rather than being
 * faked here.
 */
export function evaluateGates(run, exp) {
    const g = [];
    const add = (name, pass, detail) => g.push({ name, pass, detail });

    if (run.dropped === null) {
        add('dropped', null, 'No stop summary seen.');
    } else {
        add('dropped', run.dropped === 0, `dropped=${run.dropped}`);
    }

    for (const [key, probe] of [
        ['clock pre', run.clockPre],
        ['clock post', run.clockPost],
    ]) {
        if (!probe) {
            add(key, null, 'not taken');
        } else {
            add(
                key,
                probe.residualMs <= GATE_LIMITS.clockResidualMs,
                `residual ${probe.residualMs.toFixed(2)} ms, kept ${probe.kept}/${probe.total}`,
            );
        }
    }

    if (run.skewPpm !== null && run.skewPpm !== undefined) {
        add(
            'skew',
            Math.abs(run.skewPpm) <= GATE_LIMITS.skewPpmMax,
            `${run.skewPpm.toFixed(1)} ppm`,
        );
    }

    // The encoder check is free: startRecording() zeroes the count and nothing
    // resets it afterward, so a STREAM right after the stop reads the run total.
    if (run.encoderCountAfter === null || run.encoderCountAfter === undefined) {
        add('encoder state', null, 'not read');
    } else if (run.encoder === 'detached') {
        add(
            'encoder state',
            run.encoderCountAfter === 0,
            `count=${run.encoderCountAfter}, expected frozen at 0`,
        );
    } else if (run.encoder === 'attached') {
        add(
            'encoder state',
            Math.abs(run.encoderCountAfter) > GATE_LIMITS.encoderAttachedMinCount,
            `count=${run.encoderCountAfter}, expected motion`,
        );
    } else {
        add('encoder state', null, `count=${run.encoderCountAfter}, either state accepted`);
    }

    if (run.samples !== null && run.durationS) {
        const expected = run.durationS * IMU_ODR_HZ;
        add(
            'sample count',
            run.samples >= expected * GATE_LIMITS.sampleShortfallFrac,
            `${run.samples} of ~${Math.round(expected)} expected`,
        );
    }

    if (exp?.holds !== false) {
        for (const [key, hold] of [
            ['still hold pre', run.holdPre],
            ['still hold post', run.holdPost],
        ]) {
            if (!hold) {
                add(key, null, 'not timed');
            } else {
                const s = (hold.end - hold.start) / 1000;
                add(key, s >= GATE_LIMITS.holdSecondsMin, `${s.toFixed(1)} s`);
            }
        }
    }

    return g;
}

/** A run passes only if no gate failed. Unknown gates block, they do not pass. */
export function verdictFor(gates) {
    if (gates.some((x) => x.pass === false)) return 'discard';
    if (gates.some((x) => x.pass === null)) return 'incomplete';
    return 'pass';
}

export function applyGates(run, exp) {
    run.gates = evaluateGates(run, exp);
    if (!run.overridden) run.verdict = verdictFor(run.gates);
    return run;
}

// --- persistence --------------------------------------------------------

export function save(session) {
    try {
        localStorage.setItem(STORE_KEY, JSON.stringify(session));
        return true;
    } catch {
        return false; // quota or private mode; export is still available
    }
}

export function load() {
    try {
        const raw = localStorage.getItem(STORE_KEY);
        return raw ? JSON.parse(raw) : null;
    } catch {
        return null;
    }
}

export function clearStored() {
    try {
        localStorage.removeItem(STORE_KEY);
    } catch {
        /* ignore */
    }
}

// --- export -------------------------------------------------------------

export function toJson(session) {
    return JSON.stringify(session, null, 2);
}

/**
 * The session sheet, filled in. Same shape as the runbook's paper version so a
 * reader who knows one knows the other.
 */
export function toMarkdown(session) {
    const L = [];
    const yn = (b) => (b ? 'yes' : 'no');
    L.push(`# Velocity jig session ${session.id}`);
    L.push('');
    L.push(`- Date: ${session.startedAt}`);
    L.push(`- Operator: ${session.operator || '(unrecorded)'}`);
    L.push(`- Robot: ${session.robot || '(unrecorded)'}`);
    L.push(`- Floor surface: ${session.floorSurface || '(unrecorded)'}`);
    L.push(`- Guard plates on: ${yn(session.guardPlatesOn)}`);
    L.push(`- Weapon disabled: ${yn(session.weaponDisabled)}`);
    L.push(`- Encoder mount setting: ${session.encoderMountSetting || '(unrecorded)'}`);
    L.push(
        `- Firmware ranges: gyro ${session.imu.gyroRangeDps} dps, accel ${session.imu.accelRangeG} g`,
    );
    L.push(
        `- Space: board ${session.space.boardM} m, usable ${session.space.usableM} m, half width ${session.space.halfWidthM} m`,
    );
    L.push(
        `- E5 encoder rate limit: ${session.encoderRateLimit ?? '(not measured)'} rad/s`,
    );
    L.push(`- E12 drag verdict: ${session.dragVerdict || '(not measured)'}`);
    L.push('');

    L.push('## Runs');
    L.push('');
    L.push(
        '| # | Log | Experiment | Enc | Samples | Drop | Clock pre | Clock post | Skew | Verdict |',
    );
    L.push('|---|---|---|---|---|---|---|---|---|---|');
    session.runs.forEach((r, i) => {
        const pre = r.clockPre ? `${r.clockPre.offsetMs.toFixed(1)} (${r.clockPre.residualMs.toFixed(2)})` : '';
        const post = r.clockPost ? `${r.clockPost.offsetMs.toFixed(1)} (${r.clockPost.residualMs.toFixed(2)})` : '';
        const name = r.variantLabel ? `${r.experimentId} ${r.variantLabel}` : r.experimentId;
        L.push(
            `| ${i + 1} | ${r.logFile ?? ''} | ${name} | ${r.encoder[0].toUpperCase()} | ${
                r.samples ?? ''
            } | ${r.dropped ?? ''} | ${pre} | ${post} | ${
                r.skewPpm !== null && r.skewPpm !== undefined ? r.skewPpm.toFixed(0) : ''
            } | ${r.verdict ?? ''} |`,
        );
    });
    L.push('');

    const failed = session.runs.filter((r) => r.verdict === 'discard');
    L.push('## Discarded runs');
    L.push('');
    if (!failed.length) {
        L.push('None.');
    } else {
        for (const r of failed) {
            const why = r.gates.filter((g) => g.pass === false).map((g) => `${g.name}: ${g.detail}`);
            L.push(`- ${r.logFile ?? r.id} (${r.experimentId}): ${why.join('; ') || 'manual'}`);
            if (r.notes) L.push(`  - ${r.notes}`);
        }
    }
    L.push('');

    const measured = session.runs.filter((r) => Object.keys(r.measures ?? {}).length);
    if (measured.length) {
        L.push('## Hand measurements');
        L.push('');
        for (const r of measured) {
            const pairs = Object.entries(r.measures).map(([k, v]) => `${k}=${v}`);
            L.push(`- ${r.logFile ?? r.id} (${r.experimentId}): ${pairs.join(', ')}`);
        }
        L.push('');
    }

    const trimmed = session.runs.filter((r) => r.trim);
    if (trimmed.length) {
        L.push('## Trimmed runs');
        L.push('');
        L.push('The trim is a commanded angular value held for the whole run, so these are');
        L.push('two-input excitations. Fit them with the angular command from the command log,');
        L.push('not as pure linear steps.');
        L.push('');
        for (const r of trimmed) {
            L.push(
                `- ${r.logFile ?? r.id} (${r.experimentId} ${r.variantLabel}): trim ${r.trim.toFixed(4)}, implied wheel mismatch ${(r.trimMismatch * 100).toFixed(2)}%`,
            );
        }
        L.push('');
    }

    const noted = session.runs.filter((r) => r.notes && r.verdict !== 'discard');
    if (noted.length) {
        L.push('## Run notes');
        L.push('');
        for (const r of noted) L.push(`- ${r.logFile ?? r.id} (${r.experimentId}): ${r.notes}`);
        L.push('');
    }

    L.push('## Anything that surprised you');
    L.push('');
    L.push(session.notes || '(nothing recorded)');
    L.push('');

    L.push('## Offline checks still owed');
    L.push('');
    L.push('These need the log files and cannot be checked at capture time:');
    L.push('');
    L.push('- Gyro bias drift between the pre and post still holds, limit 0.05 deg/s.');
    L.push('- Saturated gyro samples on every detached spin run.');
    L.push('- Yaw axis from the still-hold gravity vector, recomputed per run.');
    return L.join('\n');
}

export function download(filename, text) {
    const blob = new Blob([text], { type: 'text/plain' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = filename;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

/** Progress across the planned battery, for the session overview. */
export function progress(session, experiments) {
    const done = new Set(
        session.runs.filter((r) => r.verdict === 'pass').map((r) => `${r.experimentId}/${r.variant}`),
    );
    return experiments.map((e) => ({
        id: e.id,
        name: e.name,
        block: e.block,
        variants: e.variants.map((v) => ({
            id: v.id,
            label: v.label,
            done: done.has(`${e.id}/${v.id}`),
            count: session.runs.filter(
                (r) => r.experimentId === e.id && r.variant === v.id && r.verdict === 'pass',
            ).length,
        })),
    }));
}
