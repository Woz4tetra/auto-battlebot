# Velocity jig runbook

Step-through procedures for every experiment feeding the Kalman filter plant model. Companion to
[`kalman_filter_plan.md`](kalman_filter_plan.md), which explains why each number matters. This
document is what you hold while standing at the robot.

Twenty-three experiments in four blocks. Blocks 0 and 1 are bench work with no driven motion. Block 2
is the driven battery. Block 3 is validation. Run them in order; later blocks assume the calibrations
from earlier ones.

## The encoder rule

The encoder is detachable, and that decides half the procedures here.

| Encoder | When | Why |
|---------|------|-----|
| **Attached** | Anything with a linear component | Encoder arc length is the only linear ground truth |
| **Detached** | High-rate spin in place | The trailing wheel scrubs sideways at speed and will not survive it |

Two consequences that are easy to miss:

1. **An attached encoder is part of the plant.** Its wheel drags on the floor, so parameters fit with
   it attached describe robot-plus-encoder, not the competition robot. E12 measures how large that
   error is. Until E12 is done, treat every fitted parameter as provisional.
2. **A detached encoder holds a frozen count.** The inputs are pulled up, so the count column stays
   constant instead of going missing. The diagnostics screen shows `AB 11` with a frozen `ENC`. Use
   that to confirm the detach, and treat a frozen count in a run that should have had linear motion as
   a failed run, not as zero motion.

Every reattach needs the spot check in [Encoder attach and detach](#encoder-attach-and-detach).
Remount geometry changes the effective wheel radius, and a 2% scale error is a 2% velocity error in
everything fit afterward.

## Safety

- **Weapon disabled for the first 22 experiments, E0 through E21.** Leave it installed but not
  spinning: its mass and inertia are part of the plant we are fitting, its angular momentum is not
  something this model covers. E22 covers the spinning case and has its own bar.
- **Guard plates on, always.** Their ground friction is part of the plant. Characterizing without them
  produces numbers that do not apply in a match.
- **Human driver's sticks centered.** Trainer mode adds stick input to the scripted command. A nudged
  stick contaminates the run at best.
- **Clear, bounded space.** Steps at full command cover several meters before stopping. Know where the
  robot ends up before you arm.
- **Arm explicitly, disarm on every exit.** The runner zeroes channels and disarms on normal exit,
  Ctrl-C, exception, and wall-clock timeout. Verify that behavior once at the start of a session with
  a deliberate Ctrl-C, before you trust it for the rest of the day.

## Kit

- Robot, guard plates fitted, weapon disabled
- Jig mounted, encoder and its detach hardware
- Charged packs, at least three, plus a voltmeter or a charger that reports resting voltage
- Tape measure, masking tape, marker
- A 3 m clear straight line, marked at both ends
- A pivot or turntable with an index mark, for E3
- Laptop with the transmitter connected and `venv` active
- The session sheet at the end of this document, printed or on screen

## How a run works

Every driven run follows the same card. Learn this once and the rest of the document is just which
excitation to play.

**Run card**

- [ ] 1. Confirm encoder state matches the experiment. Detached runs: verify `AB 11` and a frozen
      `ENC` on the diagnostics screen (button C).
- [ ] 2. Record pack resting voltage on the session sheet.
- [ ] 3. Clock probe, pre. 200 probes. Record the fitted offset.
- [ ] 4. Press A to start recording.
- [ ] 5. **Hold still 10 s.** Do not skip this. It is the per-run gyro bias estimate.
- [ ] 6. Play the excitation.
- [ ] 7. **Hold still 10 s.** This is the bias drift bound for the run.
- [ ] 8. Press B to stop. Record the filename, sample count, and `DROPPED` from the summary screen.
- [ ] 9. Clock probe, post. Record the offset and the implied skew.
- [ ] 10. Write the run down on the session sheet immediately. The jig names files `LOG-0`, `LOG-1`,
      and so on, with no metadata inside. An unlabeled log is a discarded log.

**Gates that abort a run**

| Observation | Action |
|-------------|--------|
| `DROPPED` nonzero | Discard the run. Lower `IMU_ODR`, raise `SD_SCK_MHZ_VAL`, or swap the card |
| Clock probe residuals over 2 ms | Stop recording data. Fix the USB path before continuing |
| Pre and post bias differ by more than 0.05 deg/s | Discard. The IMU is drifting, usually thermal |
| Robot hit a wall, tipped, or was touched | Discard. Note why on the sheet |
| Frozen encoder count on a linear run | Discard. Check the connector |

Discarding runs is cheap. The 2026-07-03 AprilTag session shipped parameters fit from a single
surviving segment because bad data was not caught at the time it was recorded.

---

# Block 0: Bench setup

No driven motion. Do this once per hardware change, not once per session.

## E0. Firmware and range check

**Encoder:** either. **Duration:** 15 min. **Robot motion:** none.

Confirms the jig records what we think it records before anything expensive depends on it.

- [ ] Flash current firmware: `pio run -t upload` from `firmware/velocity_jig`.
- [ ] Open the console. Confirm `LIST`, `TIME`, and `STREAM` all answer.
- [ ] Press C for diagnostics. Confirm one accel axis reads about 1.00 g at rest and the gyro rows
      sit near zero.
- [ ] Rotate the robot by hand about each axis and confirm the expected gyro row responds. This is a
      wiring check only. Do not write the yaw axis down; derive it from gravity instead (below).
- [ ] Check the configured ranges in `include/config.h`:
      `IMU_GYRO_RANGE` and `IMU_ACCEL_RANGE`.

**The range problem.** The default gyro range is +/-2000 dps, which clips at 34.9 rad/s. The stage 2
calibration put max yaw rate at 61.5 rad/s. The default accel range is +/-8 g, and an IMU mounted
50 mm off the yaw axis at 35 rad/s sees 6.2 g of centripetal acceleration before any real
acceleration happens.

- [ ] If the gyro is still at 2000 dps, switch it to the ISM330DHCX 4000 dps extension and the accel
      to 16 g before running E11 or E22. E1 through E10 are safe at the default ranges.
- [ ] Whatever the ranges, check every recording for saturation: count samples with `|raw| > 32000`
      per axis. A saturated channel biases every fit that uses it and does so silently.

**The yaw axis comes from gravity, not from a note.** At rest the accelerometer measures specific
force, so it reads +1 g on whichever axis points up. That is a signed vector: it gives both the
vertical axis and which end of it is up. The ISM330DHCX puts the gyro and accel on one right-handed
triad, so up plus the right-hand rule fixes positive yaw as counter-clockwise viewed from above. Every
run card already opens and closes with a 10 s still hold, so the reference is in every log. Analysis
takes it from there:

```
u = mean(accel over the still segment); u /= norm(u)
yaw_rate = dot(gyro, u)
```

This survives a tilted or rotated remount, where no single gyro axis is yaw anymore, and it is
recomputed per run so a remount cannot silently invalidate an earlier fit. Two limits: the reference
must come from a still segment, because at 50 mm off-axis and 35 rad/s centripetal acceleration
reaches 6.2 g and swamps gravity; and a 20 mg accel bias tilts `u` by ~20 mrad, which costs 2e-4 of
yaw-rate scale and leaks 2% of roll and pitch rate into yaw, both negligible during a flat spin.

Two sign conventions the accelerometer cannot reach, because they belong to other hardware: whether a
positive angular command produces positive measured yaw rate (E6), and whether increasing encoder
count means forward (E4).

**Pass:** all three console commands answer, gyro responds on hand rotation about each axis,
saturation counting available.

## E1. Clock probe verification

**Encoder:** either. **Duration:** 20 min. **Robot motion:** none.

The jig counts microseconds from RP2040 boot. The host that issues commands counts something else.
Cross-correlating commands against measured acceleration only ever recovers
`clock_offset + transport_delay`, so the offset has to be measured on its own or the ~60 ms actuation
latency comes out wrong by however far the clocks are apart.

- [ ] Run 200 `TIME` probes. Record round-trip times.
- [ ] Keep the lowest decile by round trip. Pair the midpoint of `(t_send, t_recv)` with the midpoint
      of the reply's `(rx_us, tx_us)`.
- [ ] Fit offset and skew across a 5 minute span.
- [ ] Repeat three times over 30 minutes. Check that residuals to the fitted line stay under 2 ms.
- [ ] Probe once while a dummy recording runs. `TIME` is answered during recording; everything else
      returns `BUSY`. Confirm the mid-run probe lands on the same line as the idle probes.

**Pass:** residuals under 2 ms across all three probe sets. Skew should land near 30 ppm, about
0.9 ms over a 30 s run.

**Fail:** if residuals exceed 2 ms, the USB path is not symmetric enough. Do not proceed to fitting
transport delay from jig data. Carry the stage 2 value of 59 ms as fixed and say so in the report.

## E2. Gyro bias and Allan variance

**Encoder:** detached (irrelevant, but keeps the count column obviously frozen). **Duration:** 35 min
recording, unattended. **Robot motion:** none.

- [ ] Place the robot on a solid floor, away from foot traffic and HVAC vents. A table that flexes
      when someone walks past will show up in the Allan curve.
- [ ] Power the robot and let it sit 10 min before recording, so the IMU reaches thermal steady state.
- [ ] Record 30 min with no motion at all.
- [ ] Compute: mean bias per axis, Allan deviation curve, angle random walk (deg/sqrt(hr)), rate
      random walk.

**Pass:** an Allan curve with a visible minimum. The tau at the minimum tells you how long a bias
estimate stays good, which sets how often the 10 s stationary windows in each run need to be taken.

**Produces:** the gyro noise terms that feed the heading uncertainty in the process noise model.

## E3. Gyro scale factor

**Encoder:** detached. **Duration:** 15 min. **Robot motion:** by hand.

- [ ] Set the robot on the pivot with the yaw axis vertical. Mark a reference line on the floor and on
      the robot.
- [ ] Start recording. Hold still 10 s.
- [ ] Rotate by hand through exactly 10 full turns counter-clockwise, smoothly, taking about 20 s.
      Stop precisely on the index mark.
- [ ] Hold still 10 s. Repeat clockwise in a second run.
- [ ] Integrate gyro yaw over each rotation and compare to 3600 deg.

**Pass:** scale error under 1%. Record the correction factor and apply it in every later analysis.

**Watch for:** a scale error that differs between directions. That points at a mounting or axis-sign
problem, not a scale factor. Sort it before continuing.

## E4. Encoder scale

**Encoder:** attached. **Duration:** 20 min. **Robot motion:** pushed by hand, motors disarmed.

- [ ] Tape a 3.000 m line on the floor. Mark both ends. Measure it twice.
- [ ] Disarm the robot. This run is hand-pushed, so no command is involved.
- [ ] Start recording, then push the robot along the line from mark to mark, smoothly, staying
      straight. Ten passes forward, ten back, all in one recording, pausing 2 s at each end.
- [ ] Fit meters per count by least squares through the origin across all 20 passes.

**Pass:** standard error under 0.3% across passes. Forward and reverse scale should agree; if they do
not, look for a mounting that is not square rather than accepting two numbers.

**Produces:** `meters_per_count`, the constant every linear measurement depends on.

## E5. Lever arms

**Encoder:** attached. **Duration:** 20 min. **Robot motion:** driven, slow spins only.

The IMU and the encoder wheel are both mounted somewhere other than the yaw axis, so a pure spin
produces signals that are not real translation. Measuring the offsets lets us remove them, which is
what makes the combined lin+ang runs in E13 usable instead of discarded.

- [ ] Clear a 2 m circle.
- [ ] Command pure spins at four increasing yaw rates. **Keep the top rate under 8 rad/s.** This is the
      one spin experiment with the encoder attached, and the wheel scrubs sideways the whole time.
      Hold each rate 4 s.
- [ ] Regress measured lateral acceleration against measured yaw rate squared. The slope is `r_imu`.
- [ ] Regress encoder rate against gyro yaw rate over the same segments. The slope is `r_enc_perp`.

**Pass:** both regressions linear with R^2 above 0.95. If `r_imu` disagrees with a tape measure from
the yaw axis to the IMU by more than 10 mm, something is wrong with the axis assignment from E0.

**Stop immediately if:** the encoder wheel chatters, skips, or leaves a scrub mark. Drop the top rate
and note the limit on the session sheet. That limit is the cap for E13 and E16 as well.

---

# Block 1: Encoder attach and detach

## Encoder attach and detach

Do this the same way every time or E4's scale factor stops being true.

**To detach:**

- [ ] Disarm and power down the drive.
- [ ] Unplug the encoder connector. Do not cut power to the jig; the IMU sampling heartbeat should keep
      running.
- [ ] Remove the wheel assembly if it is a separate mount.
- [ ] Press C and confirm `AB 11` with a frozen `ENC` count.

**To reattach:**

- [ ] Mount the wheel assembly at the same position and preload. If the mount has an adjustment, note
      its setting on the session sheet.
- [ ] Plug in the connector.
- [ ] Press C and confirm the count changes when you roll the robot by hand and that it increases when
      the robot moves forward.
- [ ] **Spot check the scale.** Record one hand-pushed 3.000 m pass and confirm the derived distance
      lands within 1% of 3.000 m. Two minutes, and it catches a remount that changed the effective
      radius.

**If the spot check fails by more than 1%:** redo E4 in full. Do not scale-correct by hand from a
single pass.

---

# Block 2: Driven battery

This is the data the model is fit from. Everything here needs `velocity_jig_drive.py`, which plays a
scripted command sequence over the trainer link and logs each command with a host timestamp. Phases
marked *manual fallback* can be driven by hand if the script is not ready, at the cost of a much
weaker fit.

Run the full block three times per session, and run three sessions on different days with different
packs. Session-to-session spread is a number we need, not an inconvenience.

## E6. Polarity check

**Encoder:** attached. **Duration:** 2 min. **Manual fallback:** yes.

Stage 2 discovered after the fact that both channels were anti-correlated with their command, because
the solved pose frame was inverted. Catch that at the start of the session instead of in the fit.

- [ ] Drive 1 m forward. Confirm encoder count increases.
- [ ] Turn 90 deg left. Confirm integrated gyro yaw increases.
- [ ] Drive 1 m in reverse. Confirm the count decreases.

**Pass:** all three signs as expected. **Fail: abort the session** and fix the sign convention. Every
run recorded after a sign mismatch is wasted.

## E7. Linear deadzone staircase

**Encoder:** attached. **Duration:** 1 min per run. **Manual fallback:** no.

- [ ] Clear 6 m of straight floor.
- [ ] Staircase from 0.01 to 0.10 in steps of 0.01, holding each 1.5 s, forward.
- [ ] Repeat in reverse.
- [ ] Take the median sustained speed at each level. The deadzone is the level where speed first clears
      5 sigma of the stationary noise.

**Why 0.01:** stage 2 reported the deadzone as 0.04 because that was the smallest step tested and the
robot was already moving there. The real value is somewhere below. This staircase finds it.

**Pass:** at least three levels below the motion threshold and three above. If the robot moves at 0.01,
extend the staircase downward.

## E8. Linear steps

**Encoder:** attached. **Duration:** 2 min per run. **Manual fallback:** no.

The most important driven experiment. Produces max speeds both directions, both accel time constants,
and most of the command edges the delay estimate pools over.

- [ ] Clear the longest straight run available. At 5.6 m/s a 1.0 step covers ground fast.
- [ ] Steps to 0.25, 0.5, 0.75, 1.0, holding 1.5 s, returning to zero between each and letting the
      robot fully coast to a stop before the next.
- [ ] Repeat in reverse.
- [ ] **The step must not be slew limited.** Stage 2 could not fit forward accel tau because
      `build_protocol` ramped every forward command increase and the fitter then excluded the ramp, so
      the hold started after the robot had already spun up. Confirm the runner's slew limiter is off
      for this phase before recording.

**Pass:** every step reaches a visible steady state, and every return to zero produces a clean coast
tail. If the robot runs out of floor before steady state at 1.0, record the shorter step anyway and
note it: max speed then comes from the lower steps plus the fitted first-order model, which is what
stage 2 had to do, but here at least the rise is fully sampled.

**Produces:** `k_fwd`, `k_rev`, `tau_lin_a`, `tau_lin_d`.

## E9. Coast tails

**Encoder:** attached. **Duration:** 1 min per run. **Manual fallback:** yes.

Covered by E8's returns to zero, but worth a dedicated run because decel was the better-measured
constant in stage 2 and it is what drives stopping overrun.

- [ ] Accelerate to a steady 0.6 forward, hold 2 s, then drop the command to zero instantly.
- [ ] Let the robot coast to a complete stop before the next repetition.
- [ ] Ten repetitions forward, ten reverse.

**Pass:** ten clean tails per direction, each with at least 100 ms of decay before the robot stops.

## E10. Angular deadzone staircase

**Encoder:** attached. **Duration:** 1 min per run. **Manual fallback:** no.

Low rate throughout, so the encoder stays on. Having it on is useful here: it catches unintended
linear motion during what should be a pure rotation.

- [ ] Staircase from 0.01 to 0.10 in steps of 0.01, holding each 1.5 s, left.
- [ ] Repeat right.
- [ ] Confirm from the encoder that linear drift stayed small. Large drift means the robot is walking
      rather than pivoting, and the angular fit will be contaminated.

## E11. Angular steps and max spin

**Encoder: DETACHED.** **Duration:** 2 min per run. **Manual fallback:** no.

The high-rate experiment. The encoder comes off for this one.

- [ ] Detach the encoder per the procedure above. Confirm `AB 11` and a frozen count.
- [ ] Confirm the gyro range is 4000 dps (E0). At 2000 dps this run clips and the result is worthless.
- [ ] Steps to 0.25, 0.5, 0.75, 1.0, holding 1.5 s, returning to zero between, spinning left.
- [ ] Repeat right.
- [ ] Check the saturation count immediately after the run.

**Pass:** zero saturated gyro samples, and a visible steady state at 1.0.

**Produces:** `k_ang`, `tau_ang_a`, `tau_ang_d`.

**Cross-checks to run before believing the result:**

- Compare fitted `k_ang` against `2 * k_fwd / track_width`. A fit above that geometric bound is wrong.
- Compare against the stage 2 value of 61.5 rad/s. That number came from camera keypoints and may have
  been inflated by yaw keypoint flips. If the gyro says the true rate is well below it, that is a
  finding about the perception pipeline and belongs in the report.

## E12. Encoder drag delta

**Encoder: both, back to back.** **Duration:** 10 min. **Manual fallback:** no.

The experiment that tells us how much the encoder contaminated everything else. Guard plates stay on
in a match; the encoder wheel does not. Its drag is a contaminant, not part of the plant we want.

The angular channel is the one we can measure both ways, since the gyro does not care whether the
encoder is attached.

- [ ] With the encoder **attached**, run a capped angular battery: steps to 0.25 and 0.5 only, holding
      1.5 s, both directions, five repetitions. Stay under the rate limit found in E5.
- [ ] Detach the encoder. Change nothing else, same pack, same floor, within 10 minutes.
- [ ] Run the identical sequence **detached**.
- [ ] Fit `k_ang` and `tau_ang_a` separately for each and compare.

**Interpretation:**

| Result | Meaning |
|--------|---------|
| Difference within the run-to-run spread from E11 | Encoder drag is negligible. Linear parameters stand as fit |
| `k_ang` lower attached, outside spread | Encoder adds real drag. Report linear parameters with a stated bias and estimate the correction from the ratio |
| `tau` longer attached | Added drag and inertia. Same treatment |

**This gate matters more than it looks.** If the delta is large, every linear parameter in E7 through
E9 carries a bias we cannot remove by measuring more carefully, only by correcting for. Record the
result prominently on the session sheet.

## E13. Coupling grid

**Encoder:** attached, rate capped per E5. **Duration:** 1 min per run. **Manual fallback:** no.

Steer-brake (turning costs forward speed) and angular droop (linear load slows the turn) were both
unmeasured in stage 2, because the combination never survived tag dropouts. With on-robot ground truth
they are straightforward.

- [ ] 4x4 grid of held command pairs: `u_lin` in {0.25, 0.5, 0.75, 1.0} crossed with `u_ang` in
      {0, 0.1, 0.2, 0.3}, capped so yaw rate stays under the E5 limit.
- [ ] Hold each combination 1.5 s, returning to zero between.
- [ ] Remove the lever-arm terms from E5 before fitting, or the rotation contaminates the linear
      measurement.

**Pass:** all 16 cells recorded with the robot in bounds. Cells the floor cannot accommodate get noted
as missing rather than quietly skipped.

**Expected outcome:** possibly nothing. If a coupling coefficient fits within its confidence interval
of zero, it does not go in the model. Three terms that are all real beat six where two are noise.

## E14. Linear PRBS

**Encoder:** attached. **Duration:** 30 s per run. **Manual fallback:** no.

Held out of the fit entirely. This is the honest validation set.

- [ ] PRBS on the linear channel, amplitude 0.6, bit period 60 ms, 30 s.
- [ ] Confirm the robot stays in bounds. A PRBS wanders; start it in the middle of the space.

**Why:** steps excite one band well. A 60 ms bit period covers roughly 0.2 to 8 Hz, which brackets the
58-78 ms time constants and the 33 ms frame period. If the model fit from steps also predicts PRBS
data, it generalizes.

## E15. Angular PRBS

**Encoder: two versions.** **Duration:** 30 s each. **Manual fallback:** no.

- [ ] **Attached, capped:** amplitude limited to the E5 rate limit, 30 s.
- [ ] **Detached, full:** amplitude 0.6, 30 s. Confirm the frozen count before starting.

The capped version validates the model in the regime where we also have linear truth. The full version
validates the angular model at rates the encoder cannot survive.

## E16. Combined PRBS

**Encoder:** attached, capped. **Duration:** 30 s per run. **Manual fallback:** no.

- [ ] Independent PRBS sequences on both channels, linear amplitude 0.6, angular capped, 30 s.

This is the validation set for the coupling terms from E13. Different seeds each repetition.

## E17. Chirp

**Encoder:** attached, capped. **Duration:** 30 s per run. **Manual fallback:** no.

- [ ] Logarithmic sweep 0.2 to 8 Hz, amplitude 0.4, on the linear channel, 30 s.
- [ ] Repeat on the angular channel, capped.

Gives a frequency response to cross-check the fitted time constants. A first-order model with tau of
58 ms puts its corner near 2.7 Hz; the chirp should show that corner where the step fit says it is.

## E18. Operator driving

**Encoder: two versions.** **Duration:** 60 s each. **Manual fallback:** yes, this phase is manual by
definition.

The holdout with a realistic command distribution. Scripted excitation covers the space evenly; a
driver does not, and the model has to work on what a driver actually does.

- [ ] **Attached:** drive as in a match but keep spins under the E5 rate limit. 60 s.
- [ ] **Detached:** drive with no rate restriction, including full spins. 60 s. Heading-only validation.

Note on the sheet who drove and roughly what they were doing. "Chasing a corner cone aggressively"
is useful context when a residual spikes.

## E19. Closure runs

**Encoder:** attached. **Duration:** 2 min. **Manual fallback:** yes.

The check on dead-reckoning ground truth itself.

- [ ] Mark a start point and heading with tape.
- [ ] Drive a loop of roughly 20 m that returns to the mark, same heading, taking about 20 s.
- [ ] Stop precisely on the mark. Measure the actual position error with a tape measure.
- [ ] Repeat twice more with different loop shapes.

**Pass:** closure error under 1% of path length, so under 200 mm on a 20 m loop.

**Fail:** a run above 1% gets excluded from fitting and investigated. The usual causes are wheel slip
(check the E8 flag rate), lift-off during hard accel, or gyro bias drift (compare the pre and post
stationary windows).

## E20. Pack state repeat

**Encoder:** attached. **Duration:** 10 min. **Manual fallback:** no.

- [ ] Run E8 on a freshly charged pack. Record resting voltage.
- [ ] Run the robot hard for 3 min to draw the pack down.
- [ ] Run E8 again immediately. Record resting voltage.
- [ ] Compare fitted `k_fwd` between the two.

**Why:** if max speed varies with pack state, a constant `k_fwd` is wrong at exactly the moment of a
match when it matters. The likely outcome is that we still ship a constant and fold the variation into
process noise, but that decision should follow a number.

---

# Block 3: Validation and extensions

## E21. Joint jig and camera session

**Encoder: two versions.** **Duration:** 45 min. **Location:** the arena, in view of the ZED.

One session produces the measurement noise model, the projection bias map, and the A/B test bed for
comparing the new filter against `RobotFrontBackSimpleFilter`.

- [ ] Set up in the arena with the camera in its match position.
- [ ] Record on the **Jetson, live**: SVO plus MCAP with perception outputs, filter state, and
      commands. Do not plan to re-derive perception by replaying the SVO on a desktop later. Desktop
      replay warps frames a few percent relative to the live path, which would show up as a fake
      measurement bias.
- [ ] Align three clocks: jig to host by `TIME` probe, host to Jetson by NTP or a shared command log.
- [ ] **Encoder attached, capped:** run E8, E13, E14, E16, E18 inside the arena.
- [ ] **Encoder detached:** run E11 and the detached E18.

**Produces:** measured R as a function of range and image position, the flat-plane projection bias,
the dropout distribution under real motion, and ground-truthed pose error for both filters.

## E22. Weapon-spinning delta

**Encoder: detached.** **Duration:** 15 min. **Safety bar: highest in this document.**

A spinning weapon carries angular momentum, so yaw response changes: the chassis resists yaw
acceleration about the weapon's axis, and precession couples the axes. The model does not cover it.
This experiment measures how big the error is, so we know whether it needs covering.

**Do not run this until every other experiment is complete and the plant model is fit.** It is the
only experiment with a live weapon, and it exists to quantify a known model gap, not to fit
parameters.

- [ ] Full safety setup: enclosure or containment, everyone clear, weapon spun up outside the robot's
      drive path first.
- [ ] Encoder detached. This is a spin experiment with a live weapon; nothing fragile stays mounted.
- [ ] Repeat the E12 capped angular sequence with the weapon at match RPM.
- [ ] Compare `k_ang` and `tau_ang_a` against the weapon-stopped values from E12.

**Interpretation:** a difference inside the run-to-run spread means the model gap is not worth
covering. A large difference means the angular process noise needs inflating whenever the weapon is
spinning, which the filter can key off the weapon command.

---

# Session sheet

Copy this per session. Fill it in as you go, not afterward.

```
SESSION ____________  DATE __________  ROBOT __________  OPERATOR __________
FLOOR SURFACE ______________________  GUARD PLATES ON [ ]  WEAPON DISABLED [ ]
ENCODER MOUNT SETTING ______________  FIRMWARE GYRO RANGE ______ ACCEL RANGE ______

E5 ENCODER RATE LIMIT: ________ rad/s     (cap for E13, E15 capped, E16, E17, E18 attached)
E12 DRAG DELTA:  k_ang attached ______  detached ______  verdict ________________

RUNS
 #  | LOG file | Experiment | Enc | Pack V | Samples | DROP | Clock pre | Clock post | Notes
----+----------+------------+-----+--------+---------+------+-----------+------------+-------
 1  |          |            | A/D |        |         |      |           |            |
 2  |          |            | A/D |        |         |      |           |            |
 3  |          |            | A/D |        |         |      |           |            |
 4  |          |            | A/D |        |         |      |           |            |
 5  |          |            | A/D |        |         |      |           |            |
 6  |          |            | A/D |        |         |      |           |            |
 7  |          |            | A/D |        |         |      |           |            |
 8  |          |            | A/D |        |         |      |           |            |
 9  |          |            | A/D |        |         |      |           |            |
10  |          |            | A/D |        |         |      |           |            |

ENCODER SPOT CHECKS (after every reattach)
 time ______  measured over 3.000 m: ________ m   error ______ %   pass [ ]
 time ______  measured over 3.000 m: ________ m   error ______ %   pass [ ]

DISCARDED RUNS AND WHY
________________________________________________________________________________
________________________________________________________________________________

ANYTHING THAT SURPRISED YOU
________________________________________________________________________________
```

# Quick reference

| # | Experiment | Encoder | Duration | Produces |
|---|-----------|---------|---------:|----------|
| E0 | Firmware and range check | either | 15 min | axis map, saturation counting |
| E1 | Clock probe verification | either | 20 min | offset and skew under 2 ms |
| E2 | Gyro bias and Allan variance | detached | 35 min | bias, ARW, RRW |
| E3 | Gyro scale factor | detached | 15 min | scale correction |
| E4 | Encoder scale | attached | 20 min | meters per count |
| E5 | Lever arms | attached, slow | 20 min | `r_imu`, `r_enc_perp`, rate limit |
| E6 | Polarity check | attached | 2 min | session abort gate |
| E7 | Linear deadzone | attached | 1 min | `dz_lin_fwd`, `dz_lin_rev` |
| E8 | Linear steps | attached | 2 min | `k_fwd`, `k_rev`, `tau_lin_a/d`, delay edges |
| E9 | Coast tails | attached | 1 min | `tau_lin_d` |
| E10 | Angular deadzone | attached | 1 min | `dz_ang_l`, `dz_ang_r` |
| E11 | Angular steps and max spin | **detached** | 2 min | `k_ang`, `tau_ang_a/d` |
| E12 | Encoder drag delta | both | 10 min | bias correction for all linear params |
| E13 | Coupling grid | attached, capped | 1 min | `c_sb`, `c_ad` |
| E14 | Linear PRBS | attached | 30 s | holdout validation |
| E15 | Angular PRBS | both | 60 s | holdout validation |
| E16 | Combined PRBS | attached, capped | 30 s | coupling validation |
| E17 | Chirp | attached, capped | 60 s | frequency-domain cross-check |
| E18 | Operator driving | both | 2 min | realistic-distribution holdout |
| E19 | Closure runs | attached | 2 min | ground-truth drift check |
| E20 | Pack state repeat | attached | 10 min | voltage dependence of `k_fwd` |
| E21 | Joint jig and camera | both | 45 min | R, projection bias, A/B bed |
| E22 | Weapon-spinning delta | detached | 15 min | angular model gap size |

Block 0 runs once per hardware change. Block 2 runs three times per session, three sessions total.
Blocks 0 and 1 take about two hours. One full pass of block 2 takes about 25 minutes of recording plus
the encoder swap for E11, E12, E15, and E18.

## Next steps

1. Run E0 and decide on the gyro and accel ranges before anything else is recorded.
2. Run E1. If the clock probe does not hold 2 ms, stop and fix it; the delay estimate depends on it.
3. Build `velocity_jig_drive.py` from `calib_lib/drive_protocol.py` with the camera path removed and
   the step-phase slew limiter off.
4. Run blocks 0 and 1, then a first pass of block 2, and fit before scheduling the remaining sessions.
   One fit on real data will change some of these procedures, and it is cheaper to find that out after
   one session than after three.
