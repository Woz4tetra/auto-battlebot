// Current plant estimates and the space arithmetic built on them.
//
// The estimates come from stage 2 (docs/experiments/control_improvement/
// stage2_mr_stabs_mk2_calibration.md). They are provisional by construction:
// the experiments this tool runs exist to replace them. They are good enough to
// predict how much floor a run needs, which is all they are used for here.
//
// Everything downstream of a run is fit offline. Nothing in this file feeds a
// fit. It sizes excitations to the space available.

export const PLANT = {
    vSsFwd: 5.6, // m/s at command 1.0
    vSsRev: 4.84,
    wSs: 61.5, // rad/s, camera-derived and probably inflated (E11 checks)
    tauAccel: 0.058, // s
    tauDecel: 0.078, // s
    delayS: 0.06, // command to motion
    trackWidthM: 0.1, // beetleweight
    encoderRateLimit: 8.0, // rad/s, provisional until E5 measures it
};

/**
 * Floor a single step consumes, from command edge to standstill.
 *
 * Three pieces: the rise under a first-order lag, the coast through the
 * transport delay after the command drops, and the exponential decay.
 *
 *   d_rise  = a * v_ss * (T - tau_a * (1 - exp(-T/tau_a)))
 *   d_delay = a * v_ss * L_d
 *   d_decay = a * v_ss * tau_d
 *
 * Delay and decay use v_ss rather than the speed actually reached at the end of
 * a short hold. That overestimates by a few percent at 3 tau, which is the
 * direction a space budget should err.
 *
 * Every term is linear in amplitude, which is what makes the amplitude solver
 * below a division instead of a search.
 */
export function stepDistance(amplitude, holdS, reverse = false, p = PLANT) {
    const v = amplitude * (reverse ? p.vSsRev : p.vSsFwd);
    const rise = v * (holdS - p.tauAccel * (1 - Math.exp(-holdS / p.tauAccel)));
    const delay = v * p.delayS;
    const decay = v * p.tauDecel;
    return { rise, delay, decay, total: rise + delay + decay, peak: v };
}

/** Peak speed reached by a step of this amplitude and hold. */
export function stepPeakSpeed(amplitude, holdS, reverse = false, p = PLANT) {
    const v = amplitude * (reverse ? p.vSsRev : p.vSsFwd);
    return v * (1 - Math.exp(-holdS / p.tauAccel));
}

/**
 * Largest amplitude that fits the budget, capped at 1.0.
 *
 * Scaling amplitude is the last knob to reach, not the first. It shrinks the
 * signal-to-noise of the fit and, worse, hides any nonlinearity that only
 * shows up near full command. Shorten the dwell first.
 */
export function solveAmplitude(budgetM, holdS, reverse = false, p = PLANT) {
    const unit = stepDistance(1.0, holdS, reverse, p).total;
    if (unit <= 0) return 1;
    return Math.min(1, budgetM / unit);
}

/** Shortest hold, in tau, that still fits at full amplitude. Null if none does. */
export function solveHoldTaus(budgetM, reverse = false, p = PLANT) {
    for (const n of [5, 4, 3, 2.5, 2]) {
        if (stepDistance(1.0, n * p.tauAccel, reverse, p).total <= budgetM) return n;
    }
    return null;
}

/**
 * Lateral excursion of an open-loop differential-drive run.
 *
 * A wheel speed mismatch of `mismatch` (fraction) turns the robot at
 * w = mismatch * v / W, and the sideways offset after distance L is
 *
 *   y = 0.5 * w * L^2 / v = 0.5 * mismatch * L^2 / W
 *
 * Speed cancels. Driving slower does not help; driving shorter does, and the
 * benefit goes as L squared. This is why width usually binds before length on a
 * narrow board: 2% mismatch over 2.5 m on a 0.1 m track is 0.6 m of drift.
 */
export function lateralExcursion(lengthM, mismatch, trackWidthM = PLANT.trackWidthM) {
    return (0.5 * mismatch * lengthM * lengthM) / trackWidthM;
}

/** Mismatch that keeps a run inside a half-width. Use it to set a trim target. */
export function mismatchForHalfWidth(lengthM, halfWidthM, trackWidthM = PLANT.trackWidthM) {
    return (2 * halfWidthM * trackWidthM) / (lengthM * lengthM);
}

/**
 * Passes needed for E4's encoder scale at a given course length.
 *
 * Standard error of the fitted slope goes as the per-pass endpoint error `e`
 * over the course length, divided by sqrt(N):
 *
 *   N = (e / (D * s))^2
 *
 * At e = 3 mm and a 0.3% target, a 2.000 m course needs N = 0.25, so one pass.
 * The runbook's twenty passes are dominated by something other than tape error,
 * which means shortening the course costs almost nothing.
 */
export function encoderPasses(courseM, endpointErrorM = 0.003, targetFrac = 0.003) {
    return Math.max(1, Math.ceil((endpointErrorM / (courseM * targetFrac)) ** 2));
}

/** Usable run length from a board length: 0.25 m of margin at each end. */
export function usableLength(boardM) {
    return Math.max(0, boardM - 0.5);
}

/**
 * Space report for one planned run. `budgetM` is the usable straight length.
 * Returns what the run will actually do, so the coach can show it before arming.
 */
export function spaceReport(budgetM, holdS, requestedAmplitude = 1.0, reverse = false, p = PLANT) {
    const cap = solveAmplitude(budgetM, holdS, reverse, p);
    const amplitude = Math.min(requestedAmplitude, cap);
    const d = stepDistance(amplitude, holdS, reverse, p);
    return {
        amplitude,
        scaled: amplitude < requestedAmplitude - 1e-9,
        holdS,
        holdTaus: holdS / p.tauAccel,
        distanceM: d.total,
        breakdown: d,
        peakSpeed: stepPeakSpeed(amplitude, holdS, reverse, p),
        marginM: budgetM - d.total,
        fits: d.total <= budgetM,
    };
}
