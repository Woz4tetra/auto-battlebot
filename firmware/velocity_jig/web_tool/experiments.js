// The runbook as data. Source of truth for procedure text:
// docs/experiments/kalman_filter/velocity_jig_runbook.md
//
// Every experiment carries its checklist, its encoder state, what it produces,
// and (where it is driven) a builder that turns the space budget into an actual
// command program. The tool exists so none of this gets copied onto paper.
//
// `variants` splits an experiment into the runs you actually record. E8 is two
// runs, forward and reverse. E12 is two runs, encoder attached and detached.
// The variant is what a Run record points at.

import * as ex from './excitation.js';
import { PLANT, encoderPasses } from './plant.js';

const V = (id, label, extra = {}) => ({ id, label, ...extra });

/** Shared run-card steps for any recorded driven run. */
export const RUN_CARD = [
    'Confirm encoder state matches the experiment.',
    'Clock probe, pre.',
    'Press A to start recording.',
    'Hold still 10 s. This is the per-run gyro bias estimate.',
    'Play the excitation.',
    'Hold still 10 s. This is the bias drift bound for the run.',
    'Press B to stop.',
    'Clock probe, post.',
];

export const EXPERIMENTS = [
    // ---- Block 0: bench ------------------------------------------------
    {
        id: 'E0',
        name: 'Firmware and range check',
        block: 0,
        encoder: 'either',
        durationMin: 15,
        motion: 'none',
        recorded: false,
        needsSpace: false,
        produces: 'axis map, saturation counting',
        steps: [
            'Flash current firmware: `pio run -t upload` from firmware/velocity_jig.',
            'Confirm LIST, TIME and STREAM all answer.',
            'Rotate the robot by hand about each axis and confirm the expected gyro row responds. Wiring check only.',
            'Do not write the yaw axis down. Analysis derives it from gravity in the still hold.',
            'Set IMU_GYRO_RANGE to 4000 dps before E11 and E22. At 2000 dps the spin clips at 34.9 rad/s.',
            'Confirm the log header records both scale factors.',
        ],
        gates: [
            { name: 'console', detail: 'All three commands answer.' },
            { name: 'gyro wiring', detail: 'Each axis responds to hand rotation.' },
            { name: 'range', detail: 'Gyro range set for the experiments in this session.' },
        ],
        variants: [V('main', 'Bench check')],
    },
    {
        id: 'E1',
        name: 'Clock probe verification',
        block: 0,
        encoder: 'either',
        durationMin: 20,
        motion: 'none',
        recorded: false,
        needsSpace: false,
        produces: 'offset and skew under 2 ms',
        steps: [
            'Run three probe sets of 200 with the jig idle.',
            'Run one probe set during a recording, to confirm TIME answers instead of BUSY.',
            'Compare offsets across sets and read the implied skew.',
        ],
        gates: [
            { name: 'residual', detail: 'Under 2 ms across all sets.' },
            { name: 'skew', detail: 'Near 30 ppm. A wild value means a bad pairing, not a bad crystal.' },
        ],
        clockOnly: true,
        variants: [V('idle', 'Idle probe sets'), V('during', 'Probe during a recording')],
    },
    {
        id: 'E2',
        name: 'Gyro bias and Allan variance',
        block: 0,
        encoder: 'detached',
        durationMin: 35,
        motion: 'none',
        recorded: true,
        needsSpace: false,
        holds: false,
        produces: 'bias, ARW, RRW',
        steps: [
            'Solid floor, away from foot traffic and HVAC vents. A flexing table shows up in the curve.',
            'Power the robot and let it sit 10 min so the IMU reaches thermal steady state.',
            'Record 30 min with no motion at all. Unattended.',
        ],
        gates: [{ name: 'curve', detail: 'Allan curve has a visible minimum.' }],
        variants: [V('main', '30 min still')],
    },
    {
        id: 'E3',
        name: 'Gyro scale factor',
        block: 0,
        encoder: 'detached',
        durationMin: 15,
        motion: 'by hand',
        recorded: true,
        needsSpace: false,
        holds: true,
        produces: 'scale correction',
        steps: [
            'Set the robot on a pivot, yaw axis vertical. Mark a reference line on the floor and on the robot.',
            'Rotate by hand through exactly 10 full turns, smoothly, taking about 20 s.',
            'Stop precisely on the index mark.',
        ],
        gates: [
            { name: 'scale', detail: 'Error under 1%.' },
            {
                name: 'symmetry',
                detail: 'CCW and CW agree. A direction-dependent error is an axis-sign problem, not scale.',
            },
        ],
        variants: [V('ccw', '10 turns counter-clockwise'), V('cw', '10 turns clockwise')],
    },
    {
        id: 'E4',
        name: 'Encoder scale',
        block: 0,
        encoder: 'attached',
        durationMin: 20,
        motion: 'pushed by hand, motors disarmed',
        recorded: true,
        needsSpace: true,
        holds: true,
        manualPush: true,
        produces: '`meters_per_count`',
        steps: [
            'Clamp a straightedge to the board and tape a measured line along it. Measure the line twice.',
            'Disarm the robot. This run is hand-pushed, so no command is involved.',
            'Push the robot along the straightedge, mark to mark, smoothly. Pause 2 s at each end.',
            'Alternate forward and back within one recording.',
        ],
        gates: [
            { name: 'standard error', detail: 'Under 0.3% across passes.' },
            { name: 'symmetry', detail: 'Forward and reverse scale agree. If not, the mount is not square.' },
        ],
        variants: [V('main', 'Push passes')],
        /**
         * Passes scale with the inverse square of course length, so a short
         * course is not the problem it looks like: at 3 mm endpoint error and a
         * 0.3% target, 2.000 m needs one pass. The runbook's twenty are
         * dominated by something other than tape error.
         */
        planPasses(courseM) {
            const n = encoderPasses(courseM);
            return {
                courseM,
                minimumPasses: n,
                recommendedPasses: Math.max(6, n * 2),
                note: `Statistics need ${n}; record more for a slope you can see by eye.`,
            };
        },
    },
    {
        id: 'E5',
        name: 'Lever arms',
        block: 0,
        encoder: 'attached',
        durationMin: 20,
        motion: 'driven, slow spins only',
        recorded: true,
        needsSpace: true,
        spaceKind: 'circle',
        holds: true,
        driven: true,
        produces: '`r_imu`, `r_enc_perp`, encoder rate limit',
        steps: [
            'Clear a 2 m circle.',
            'Command pure spins at four increasing yaw rates. Keep the top rate under 8 rad/s.',
            'This is the one spin experiment with the encoder attached. The wheel scrubs sideways the whole time.',
            'Stop immediately if the encoder wheel chatters, skips, or leaves a scrub mark, and note the limit.',
        ],
        gates: [
            { name: 'regressions', detail: 'Both fits linear, R^2 above 0.95.' },
            { name: 'wheel', detail: 'No chatter, skip, or scrub mark.' },
        ],
        variants: [V('ccw', 'Spins left'), V('cw', 'Spins right')],
        program: (cfg, variant) =>
            ex.spins({
                sign: variant.id === 'cw' ? -1 : 1,
                holdS: cfg.holdS ?? 4.0,
                angularCap: cfg.angularCap,
            }),
    },

    // ---- Block 2: driven battery ---------------------------------------
    {
        id: 'E6',
        name: 'Polarity check',
        block: 2,
        encoder: 'attached',
        durationMin: 2,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        produces: 'session abort gate',
        steps: [
            'Drive forward one course length. Confirm the encoder count increases.',
            'Turn 90 deg left. Confirm integrated gyro yaw increases.',
            'Drive back in reverse. Confirm the count decreases.',
        ],
        gates: [
            { name: 'linear sign', detail: 'Forward increases count.' },
            { name: 'angular sign', detail: 'Left increases yaw.' },
            { name: 'reverse sign', detail: 'Reverse decreases count.' },
        ],
        abortOnFail: true,
        variants: [V('main', 'Three-sign check')],
        program: (cfg) => {
            const segs = [];
            const hold = Math.max(0.6, ((cfg.budgetM ?? 2) * 0.5) / (0.3 * PLANT.vSsFwd));
            const push = (d, l, a, label) => {
                const t0 = segs.length ? segs[segs.length - 1].t1 : 0;
                segs.push({ t0, t1: t0 + d, linear: l, angular: a, label });
            };
            // The turn is sized to land near 90 deg, slowly enough that the
            // operator can see which way it went. A command of 0.25 is 15 rad/s
            // on this robot, which is most of a full rotation in half a second:
            // fast enough that left and right look the same, which is the one
            // thing this experiment exists to tell apart.
            const turnRate = Math.min(2.0, (cfg.angularCap ?? 0.13) * PLANT.wSs);
            const turnCmd = turnRate / PLANT.wSs;
            const turnS = Math.PI / 2 / turnRate + PLANT.delayS;
            push(hold, 0.3, 0, 'forward');
            push(1.5, 0, 0, 'coast');
            push(turnS, 0, turnCmd, 'turn left ~90 deg');
            push(1.5, 0, 0, 'coast');
            push(hold, -0.3, 0, 'reverse');
            push(1.5, 0, 0, 'coast');
            return {
                label: 'polarity',
                durationS: segs[segs.length - 1].t1,
                segments: segs,
                at(t) {
                    return segs.find((s) => t >= s.t0 && t < s.t1) ?? { linear: 0, angular: 0 };
                },
            };
        },
    },
    {
        id: 'E7',
        name: 'Linear deadzone staircase',
        block: 2,
        encoder: 'attached',
        durationMin: 1,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        produces: '`dz_lin_fwd`, `dz_lin_rev`',
        steps: [
            'Staircase 0.01 to 0.10 in steps of 0.01, holding each 1.5 s.',
            'The deadzone is the level where speed first clears 5 sigma of stationary noise.',
            'If the robot moves at 0.01, extend the staircase downward.',
        ],
        gates: [{ name: 'bracket', detail: 'At least three levels below the motion threshold and three above.' }],
        variants: [V('fwd', 'Forward'), V('rev', 'Reverse')],
        program: (cfg, variant) =>
            ex.staircase({ channel: 'linear', sign: variant.id === 'rev' ? -1 : 1, holdS: cfg.holdS ?? 1.5 }),
    },
    {
        id: 'E8',
        name: 'Linear steps',
        block: 2,
        encoder: 'attached',
        durationMin: 2,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        primary: true,
        produces: '`k_fwd`, `k_rev`, `tau_lin_a/d`, most delay edges',
        steps: [
            'Longest straight available. At 5.6 m/s a 1.0 step covers ground fast.',
            'Steps 0.25, 0.5, 0.75, 1.0, holding each, returning to zero and coasting fully between.',
            'The step must not be slew limited. A ramped command is why stage 2 could not fit forward accel tau.',
            'If the floor runs out before steady state at 1.0, record the shorter step and note it.',
        ],
        gates: [
            { name: 'steady state', detail: 'Each step reaches a visible plateau.' },
            { name: 'coast tail', detail: 'Return to zero produces a clean tail.' },
        ],
        variants: [V('fwd', 'Forward'), V('rev', 'Reverse')],
        program: (cfg, variant) =>
            ex.steps({
                channel: 'linear',
                holdS: cfg.holdS ?? 3 * PLANT.tauAccel,
                sign: variant.id === 'rev' ? -1 : 1,
                shuttle: cfg.shuttle ?? false,
                scale: cfg.scale ?? 1.0,
            }),
    },
    {
        id: 'E9',
        name: 'Coast tails',
        block: 2,
        encoder: 'attached',
        durationMin: 1,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        produces: '`tau_lin_d`',
        steps: [
            'Accelerate to a steady 0.6, hold, then drop the command to zero instantly.',
            'Let the robot coast to a complete stop before the next repetition.',
        ],
        gates: [{ name: 'tails', detail: 'Ten clean tails per direction, at least 100 ms of decay each.' }],
        variants: [V('fwd', 'Forward'), V('rev', 'Reverse')],
        program: (cfg, variant) =>
            ex.coastTails({
                amplitude: 0.6 * (cfg.scale ?? 1.0),
                holdS: cfg.holdS ?? 2.0,
                reps: cfg.reps ?? 10,
                sign: variant.id === 'rev' ? -1 : 1,
                shuttle: cfg.shuttle ?? false,
            }),
    },
    {
        id: 'E10',
        name: 'Angular deadzone staircase',
        block: 2,
        encoder: 'attached',
        durationMin: 1,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        spaceKind: 'circle',
        holds: true,
        driven: true,
        produces: '`dz_ang_l`, `dz_ang_r`',
        steps: [
            'Staircase 0.01 to 0.10 in steps of 0.01, holding each 1.5 s.',
            'Low rate throughout, so the encoder stays on.',
            'Confirm encoder linear drift stayed small. Large drift means the robot is walking, not pivoting.',
        ],
        gates: [{ name: 'pivot', detail: 'Encoder drift small enough that the robot pivoted rather than walked.' }],
        variants: [V('left', 'Left'), V('right', 'Right')],
        program: (cfg, variant) =>
            ex.staircase({ channel: 'angular', sign: variant.id === 'right' ? -1 : 1, holdS: cfg.holdS ?? 1.5 }),
    },
    {
        id: 'E11',
        name: 'Angular steps and max spin',
        block: 2,
        encoder: 'detached',
        durationMin: 2,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        spaceKind: 'circle',
        holds: true,
        driven: true,
        primary: true,
        produces: '`k_ang`, `tau_ang_a/d`',
        steps: [
            'Detach the encoder. Confirm the frozen count.',
            'Confirm the gyro range is 4000 dps. At 2000 dps the run clips and the result is worthless.',
            'Steps 0.25, 0.5, 0.75, 1.0, holding each, returning to zero between.',
            'Check the saturation count immediately after the run.',
        ],
        gates: [
            { name: 'saturation', detail: 'Zero saturated gyro samples.' },
            { name: 'steady state', detail: 'Visible plateau at 1.0.' },
            { name: 'geometric bound', detail: 'Fitted k_ang below 2 * k_fwd / track_width.' },
        ],
        variants: [V('left', 'Spin left'), V('right', 'Spin right')],
        program: (cfg, variant) =>
            ex.steps({
                channel: 'angular',
                holdS: cfg.holdS ?? 3 * PLANT.tauAccel,
                sign: variant.id === 'right' ? -1 : 1,
            }),
    },
    {
        id: 'E12',
        name: 'Encoder drag delta',
        block: 2,
        encoder: 'both',
        durationMin: 10,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        spaceKind: 'circle',
        holds: true,
        driven: true,
        produces: 'bias correction for all linear parameters',
        steps: [
            'Encoder attached: capped angular battery, steps 0.25 and 0.5 only, both directions, five repetitions.',
            'Stay under the rate limit found in E5.',
            'Detach the encoder. Change nothing else: same pack, same floor, within 10 minutes.',
            'Run the identical sequence detached.',
        ],
        gates: [
            { name: 'pairing', detail: 'Both halves on the same pack, within 10 minutes.' },
            { name: 'delta', detail: 'Compare k_ang and tau_ang_a. Record the verdict prominently.' },
        ],
        pairing: true,
        variants: [
            V('attached', 'Attached, capped', { encoder: 'attached' }),
            V('detached', 'Detached, same sequence', { encoder: 'detached' }),
        ],
        // Capped, and the cap applies to both halves. The attached and detached
        // runs have to be the same sequence or the delta means nothing.
        program: (cfg) =>
            ex.steps({
                channel: 'angular',
                amplitudes: ex.capAmplitudes([0.25, 0.5, -0.25, -0.5], cfg.angularCap),
                holdS: cfg.holdS ?? 1.5,
            }),
    },
    {
        id: 'E13',
        name: 'Coupling grid',
        block: 2,
        encoder: 'attached',
        durationMin: 1,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        capped: true,
        produces: '`c_sb`, `c_ad`',
        steps: [
            '4x4 grid: u_lin in {0.25, 0.5, 0.75, 1.0} crossed with u_ang in {0, 0.1, 0.2, 0.3}.',
            'Cap so yaw rate stays under the E5 limit.',
            'Hold each combination, returning to zero between.',
            'Remove the E5 lever-arm terms before fitting, or rotation contaminates the linear measurement.',
        ],
        gates: [
            { name: 'coverage', detail: 'All 16 cells recorded in bounds. Cells the floor cannot take are noted as missing, not skipped quietly.' },
        ],
        variants: [V('main', 'Full grid')],
        program: (cfg) =>
            ex.couplingGrid({
                holdS: cfg.holdS ?? 1.5,
                shuttle: cfg.shuttle ?? false,
                scale: cfg.scale ?? 1.0,
                angularCap: cfg.angularCap ?? 1.0,
            }),
    },
    {
        id: 'E14',
        name: 'Linear PRBS',
        block: 2,
        encoder: 'attached',
        durationMin: 0.5,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        holdout: true,
        produces: 'holdout validation',
        steps: [
            'PRBS on the linear channel, amplitude 0.6, bit period 60 ms, 30 s.',
            'A PRBS wanders. Start it in the middle of the space.',
            'Held out of the fit entirely. This is the honest validation set.',
        ],
        gates: [{ name: 'bounds', detail: 'Robot stayed in bounds for the whole sequence.' }],
        variants: [V('main', 'Linear PRBS')],
        program: (cfg) =>
            ex.prbs({
                channels: ['linear'],
                amplitude: 0.6 * (cfg.scale ?? 1.0),
                durationS: cfg.durationS ?? 30,
                seed: cfg.seed ?? 0xace1,
            }),
    },
    {
        id: 'E15',
        name: 'Angular PRBS',
        block: 2,
        encoder: 'both',
        durationMin: 1,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        spaceKind: 'circle',
        holds: true,
        driven: true,
        holdout: true,
        produces: 'holdout validation',
        steps: [
            'Attached, capped: amplitude limited to the E5 rate limit, 30 s.',
            'Detached, full: amplitude 0.6, 30 s. Confirm the frozen count before starting.',
        ],
        gates: [{ name: 'encoder state', detail: 'Matches the variant.' }],
        variants: [
            V('attached', 'Attached, capped', { encoder: 'attached' }),
            V('detached', 'Detached, full', { encoder: 'detached' }),
        ],
        program: (cfg, variant) =>
            ex.prbs({
                channels: ['angular'],
                amplitude: variant.id === 'attached' ? (cfg.angularCap ?? 0.2) : 0.6,
                durationS: cfg.durationS ?? 30,
                seed: cfg.seed ?? 0x1234,
            }),
    },
    {
        id: 'E16',
        name: 'Combined PRBS',
        block: 2,
        encoder: 'attached',
        durationMin: 0.5,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        capped: true,
        holdout: true,
        produces: 'coupling validation',
        steps: [
            'Independent PRBS on both channels, linear amplitude 0.6, angular capped, 30 s.',
            'Different seed each repetition.',
        ],
        gates: [{ name: 'bounds', detail: 'Robot stayed in bounds.' }],
        variants: [V('main', 'Both channels')],
        program: (cfg) =>
            ex.prbs({
                channels: ['linear', 'angular'],
                amplitude: 0.6 * (cfg.scale ?? 1.0),
                angularAmplitude: cfg.angularCap ?? 0.2,
                durationS: cfg.durationS ?? 30,
                seed: cfg.seed ?? 0x5eed,
            }),
    },
    {
        id: 'E17',
        name: 'Chirp',
        block: 2,
        encoder: 'attached',
        durationMin: 1,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        capped: true,
        produces: 'frequency-domain cross-check',
        steps: [
            'Logarithmic sweep 0.2 to 8 Hz, amplitude 0.4, on the linear channel, 30 s.',
            'Repeat on the angular channel, capped.',
            'A first-order model with tau 58 ms puts its corner near 2.7 Hz. The chirp should agree.',
        ],
        gates: [{ name: 'sweep', detail: 'Full sweep completed without the robot leaving the space.' }],
        variants: [V('linear', 'Linear channel'), V('angular', 'Angular channel')],
        program: (cfg, variant) =>
            ex.chirp({
                channel: variant.id,
                amplitude:
                    variant.id === 'angular' ? (cfg.angularCap ?? 0.2) : 0.4 * (cfg.scale ?? 1.0),
                durationS: cfg.durationS ?? 30,
            }),
    },
    {
        id: 'E18',
        name: 'Operator driving',
        block: 2,
        encoder: 'both',
        durationMin: 2,
        motion: 'driven by hand',
        recorded: true,
        needsSpace: true,
        holds: true,
        manualDrive: true,
        holdout: true,
        produces: 'realistic-distribution holdout',
        steps: [
            'Attached: drive as in a match but keep spins under the E5 rate limit. 60 s.',
            'Detached: drive with no rate restriction, including full spins. 60 s. Heading-only validation.',
            'Note who drove and roughly what they were doing.',
        ],
        gates: [{ name: 'notes', detail: 'Driver and intent recorded.' }],
        variants: [
            V('attached', 'Attached, capped', { encoder: 'attached' }),
            V('detached', 'Detached, unrestricted', { encoder: 'detached' }),
        ],
        program: (cfg) => ex.manual({ durationS: cfg.durationS ?? 60 }),
    },
    {
        id: 'E19',
        name: 'Closure runs',
        block: 2,
        encoder: 'attached',
        durationMin: 2,
        motion: 'driven by hand',
        recorded: true,
        needsSpace: true,
        spaceKind: 'loop',
        holds: true,
        manualDrive: true,
        produces: 'ground-truth drift check',
        steps: [
            'Mark a start point and heading with tape.',
            'Drive a loop that returns to the mark on the same heading, taking about 20 s.',
            'Stop precisely on the mark and measure the actual position error with a tape.',
            'Repeat twice more with different loop shapes.',
        ],
        gates: [
            {
                name: 'closure',
                detail: 'Error under 1% of path length. Record the path length, since a short loop means a tighter absolute tolerance.',
            },
        ],
        measures: [
            { key: 'pathLengthM', label: 'Path length (m)' },
            { key: 'closureErrorM', label: 'Closure error (m)' },
        ],
        variants: [V('a', 'Loop A'), V('b', 'Loop B'), V('c', 'Loop C')],
        program: (cfg) => ex.manual({ durationS: cfg.durationS ?? 25 }),
    },
    {
        id: 'E20',
        name: 'Pack state repeat',
        block: 2,
        encoder: 'attached',
        durationMin: 10,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        pairing: true,
        produces: 'voltage dependence of `k_fwd`',
        steps: [
            'Run E8 on a freshly charged pack. Record resting voltage.',
            'Run the robot hard for 3 min to draw the pack down.',
            'Run E8 again immediately. Record resting voltage.',
            'Compare fitted k_fwd between the two.',
        ],
        gates: [{ name: 'voltage', detail: 'Both resting voltages recorded and clearly different.' }],
        variants: [V('fresh', 'Fresh pack'), V('drawn', 'Drawn-down pack')],
        program: (cfg, variant) =>
            ex.steps({
                channel: 'linear',
                holdS: cfg.holdS ?? 3 * PLANT.tauAccel,
                sign: 1,
                shuttle: cfg.shuttle ?? false,
                scale: cfg.scale ?? 1.0,
            }),
    },

    // ---- Block 3: validation and extensions ----------------------------
    {
        id: 'E21',
        name: 'Joint jig and camera session',
        block: 3,
        encoder: 'both',
        durationMin: 45,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        holds: true,
        driven: true,
        produces: 'measured R, projection bias, A/B bed',
        location: 'the arena, in view of the ZED',
        steps: [
            'Set up in the arena with the camera in its match position.',
            'Record on the Jetson, live: SVO plus MCAP with perception outputs, filter state, and commands.',
            'Do not plan to re-derive perception by replaying the SVO on a desktop. Replay warps frames a few percent.',
            'Align three clocks: jig to host by TIME probe, host to Jetson by NTP or a shared command log.',
            'Attached, capped: run E8, E13, E14, E16, E18 inside the arena.',
            'Detached: run E11 and the detached E18.',
        ],
        gates: [{ name: 'live capture', detail: 'Recorded on the Jetson, not replayed later.' }],
        variants: [
            V('attached', 'Attached set', { encoder: 'attached' }),
            V('detached', 'Detached set', { encoder: 'detached' }),
        ],
    },
    {
        id: 'E22',
        name: 'Weapon-spinning delta',
        block: 3,
        encoder: 'detached',
        durationMin: 15,
        motion: 'driven',
        recorded: true,
        needsSpace: true,
        spaceKind: 'circle',
        holds: true,
        driven: true,
        liveWeapon: true,
        produces: 'angular model gap size',
        steps: [
            'Do not run this until every other experiment is complete and the plant model is fit.',
            'Full safety setup: containment, everyone clear, weapon spun up outside the drive path first.',
            'Encoder detached. Nothing fragile stays mounted.',
            'Repeat the E12 capped angular sequence with the weapon at match RPM.',
            'Compare k_ang and tau_ang_a against the weapon-stopped values from E12.',
        ],
        gates: [
            { name: 'prerequisites', detail: 'Every other experiment complete and fit.' },
            { name: 'containment', detail: 'Enclosure in place, everyone clear.' },
            { name: 'gyro range', detail: '4000 dps.' },
        ],
        variants: [V('main', 'Capped angular, weapon live')],
        // Identical to E12's sequence, cap included. The only difference between
        // the two runs is meant to be the weapon.
        program: (cfg) =>
            ex.steps({
                channel: 'angular',
                amplitudes: ex.capAmplitudes([0.25, 0.5, -0.25, -0.5], cfg.angularCap),
                holdS: cfg.holdS ?? 1.5,
            }),
    },
];

export const BLOCKS = {
    0: { title: 'Block 0: Bench setup', note: 'No driven motion. Once per hardware change.' },
    1: { title: 'Block 1: Encoder attach and detach', note: 'Whenever the encoder moves.' },
    2: { title: 'Block 2: Driven battery', note: 'The data the model is fit from. Three passes per session.' },
    3: { title: 'Block 3: Validation and extensions', note: 'After the model is fit.' },
};

/** The encoder swap procedure, which is Block 1 in the runbook. */
export const ENCODER_SWAP = {
    detach: [
        'Disarm and power down the drive.',
        'Unplug the encoder connector. Do not cut power to the jig; the IMU heartbeat should keep running.',
        'Remove the wheel assembly from its mount.',
        'Confirm a frozen count (the tool checks this automatically after the next run).',
    ],
    reattach: [
        'Mount the wheel assembly at the same position and preload. Note any adjustment.',
        'Plug in the connector.',
        'Confirm the count changes when you roll the robot by hand, and increases going forward.',
        'Spot check scale: one hand-pushed pass over the measured course, within 1%.',
    ],
    spotCheckFail: 'Redo E4 in full. Do not scale-correct by hand from a single pass.',
};

export function getExperiment(id) {
    return EXPERIMENTS.find((e) => e.id === id);
}

export function getVariant(exp, variantId) {
    return exp.variants.find((v) => v.id === variantId) ?? exp.variants[0];
}

/** Encoder state a specific run needs: the variant overrides the experiment. */
export function encoderStateFor(exp, variant) {
    return variant?.encoder ?? exp.encoder;
}

/** Build the command program for a run, or null if the run is not scripted. */
export function buildProgram(exp, variant, cfg) {
    if (!exp.program) return null;
    return exp.program(cfg, variant ?? exp.variants[0]);
}
