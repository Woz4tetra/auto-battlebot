# Velocity jig runbook

Step-through procedures for every experiment feeding the Kalman filter plant model. Companion to
[`kalman_filter_plan.md`](kalman_filter_plan.md), which explains why each number matters. This
document is what you hold while standing at the robot.

Twenty-two experiments in three blocks. Block 0 is bench work with no driven motion. Block 2 is the
driven battery. Block 3 is validation. Run them in order; later blocks assume the calibrations from
earlier ones.

## The encoder rule

The encoder stays mounted for every experiment. Detaching it changed the angular response too little
to pay for the swap, and the swap was expensive: a remount changes the effective wheel radius, and a
2% scale error is a 2% velocity error in everything fit afterward.

Two consequences that are easy to miss:

1. **The encoder is part of the plant.** Its wheel drags on the floor, so every fitted parameter
   describes robot-plus-encoder. That is the configuration the filter runs in, so it is the right
   thing to fit, but say so when the numbers are quoted.
2. **Every angular battery is capped at the E5 rate limit.** The wheel scrubs sideways through any
   spin, so E5 runs early and its measured limit caps E11, E15, E18, E21, and E22. The cost is the
   top of the k_ang lever arm, and a capped fit has to state the cap.

A frozen count in a run that should have had linear motion is a failed run, not zero motion. Check
the connector.

## Safety

- **Weapon disabled for every experiment except E22.** Leave it installed but not
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
- Jig mounted, encoder mounted and its scale spot-checked
- Charged packs, at least three, plus a voltmeter or a charger that reports resting voltage
- Tape measure, masking tape, marker
- A 3 m clear straight line, marked at both ends
- A pivot or turntable with an index mark, for E3
- Laptop with the transmitter connected and `venv` active
- The session sheet at the end of this document, printed or on screen

## How a run works

The CLI drives the run and writes the metadata. Learn this once and the rest of the document is
just which excitation to play.

```bash
source scripts/activate_python.sh
python playground/calibration/velocity_jig_drive.py \
    --waveform lin_step_full --reps 3 \
    --name "garage floor, pack 3"
```

The session lands in `playground/calibration/out/<date>-<name>` unless `--out` says otherwise.
Running the same name again the same day appends to it, which is what you want: a session is one
battery on one floor, and it is also the unit the fitter holds out for leave-one-out validation.

**Run card.** Steps 1, 3, 5, 9 and 10 are automatic; the CLI prompts for the rest.

- [ ] 1. Clock probe, pre. 200 probes. Discard the run if the residual exceeds 2 ms.
- [ ] 2. **Press A** on the jig. The CLI waits for the `recording LOG-N.TXT` line.
- [ ] 3. **Hold still 10 s.** Do not skip this. It is the per-run gyro bias estimate.
- [ ] 4. **Unplug the USB cable.** The jig keeps logging on its own battery.
- [ ] 5. Arm and play the excitation. Ctrl-C aborts and disarms at any point.
- [ ] 6. **Hold still 10 s.** This is the bias drift bound for the run.
- [ ] 7. **Press B with the cable still out**, then press Enter at the prompt.
- [ ] 8. **Plug the cable back in.** The download starts as soon as the jig reappears.

      B first, cable second. Plugging in resets this board, and the reset lands wherever the
      firmware happens to be. On LOG-119 it landed mid-write: the log's last row came back
      truncated and the post-probe measured a fresh boot, which cost the skew correction.
      Pressing B first closes the file on the card before USB can do anything, so the reset
      has nothing left to interrupt.

      This costs the stop summary. It goes to the wire unbuffered, so with the cable out
      there is nothing to receive `n` and `dropped`. Both are reporting niceties: the gates
      read a missing count as unknown, and the log's own timestamps show the drops. A
      truncated recording cannot be recovered; a lost counter can be lived without.
- [ ] 9. Clock probe, post. The skew comes from the two probes together, and a reboot between
      them means there is no skew to have: the tool reports 0 ppm rather than the ratio of two
      different boot epochs.
- [ ] 10. The sidecar TOML and command CSV are written beside the downloaded log.

The session sheet is gone. Each log gets a `LOG-N.toml` next to it holding the waveform, its
parameters, both clock probes, the still-hold windows, and the gate results, so an unlabeled log
is no longer possible. Add anything the tool cannot know at the notes prompt, or type `discard`
there to reject a run by hand.

**Gates**

Capture-time gates run in the CLI and print a verdict at the end of each run:

| Observation | Action |
|-------------|--------|
| `dropped` nonzero | Discard. Lower `IMU_ODR`, raise `SD_SCK_MHZ_VAL`, or swap the card |
| Clock probe residual over 2 ms | Stop recording data. Fix the USB path before continuing |
| Clock skew over 200 ppm | Discard. One of the two probes is wrong |
| Gaps in the 1 kHz stream, or a log that stops short of a still hold | Discard. The recording was cut short |
| Measured command differs from what the mix could send | The driver's sticks were not centered. Discard |
| Robot hit a wall, tipped, or was touched | Discard by typing `discard` at the notes prompt |
| Post-probe uptime below the pre-probe | **Warning, not a discard.** The jig rebooted, usually at the replug |
| Truncated last row in the log | **Warning, not a discard.** A reset between the write and the flush. Every earlier row parsed |
| Clipped accelerometer | **Warning, not a discard.** Nothing in the plant model reads acceleration. Check where it lands on the plot |

A warning prints at the end of the run and is written to the sidecar's `warnings`, but the
verdict stays `pass`. A reboot costs the skew correction, worth 0.9 ms per 30 s at the RP2040's
30 ppm and measured at 2.77 ppm on this board, and it costs the `n`/`dropped` counts, which the
log's own timestamps show anyway. What it does not do by itself is damage the samples already on
the card: whether the reset truncated the run is checked directly, from the sample spacing and
the still-hold coverage. Fix the cable or the battery before the next run, because a reset that
lands during excitation instead of after it takes the run with it.

Since step 7 moved ahead of the replug, the reset arrives after the file is closed, so what it
leaves behind is at worst a truncated final row. The skew is still gone, because the post-probe
measures a board that restarted; the tool reports 0 ppm instead of the ratio of two boot epochs.
At the 2.77 ppm measured on this board, a 200 s run loses about 0.6 ms.

Operator pauses are cut out before any of this is counted. The robot is handled inside those
windows, so a set-down that clips the accelerometer is not an impact, and the seconds spent
waiting are not commanded time.

The rest need the log parsed and are applied by the fitter, which prints them per run and shows
them in the report's run-quality table: gyro bias drift over 0.05 deg/s, IMU saturation, encoder
slip, a frozen encoder count on a linear run, and a still hold detected far from where the CLI
commanded one.

Discarding runs is cheap. The 2026-07-03 AprilTag session shipped parameters fit from a single
surviving segment because bad data was not caught at the time it was recorded.

**Safety, every run**

Trainer mode *adds* to the human driver's sticks, so a run is only as safe as the person holding
the radio. Verify the disarm once per session with a deliberate Ctrl-C during the first run,
before trusting it for the rest of the day.

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

**Duration:** 35 min recording, unattended. **Robot motion:** none.

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

**Duration:** 15 min. **Robot motion:** by hand.

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

**Duration:** 20 min. **Robot motion:** pushed by hand, motors disarmed.

- [ ] Tape a 3.000 m line on the floor. Mark both ends. Measure it twice.
- [ ] Disarm the robot. This run is hand-pushed, so no command is involved.
- [ ] Start recording, then push the robot along the line from mark to mark, smoothly, staying
      straight. Ten passes forward, ten back, all in one recording, pausing 2 s at each end.
- [ ] Fit meters per count:

      ```bash
      python playground/calibration/fit_encoder_scale.py \
          --run 1.0 LOG-a.TXT LOG-b.TXT LOG-c.TXT \
          --run 2.0 LOG-d.TXT LOG-e.TXT LOG-f.TXT \
          --run 3.0 LOG-g.TXT LOG-h.TXT LOG-i.TXT \
          --plot playground/calibration/out/encoder_scale.png
      ```

      It reads the encoder count at each dwell and fits counts against distance, then prints
      the `[encoder]` block to paste into `playground/calibration/jig_calibration.toml`.

      **Use several distances, not one distance repeated.** A fixed error, the robot coasting
      past the mark or backlash taken up at the start, is a constant count offset. With one
      distance it is invisible and lands entirely in the scale; with three it shows up as an
      intercept and the differential slope cancels it. The 2026-08-20 measurement found
      +17 mm per pass that way, which a through-origin fit would have carried into the scale
      as a 2.1% error in every speed fitted afterwards.
- [ ] Paste `meters_per_count` into `playground/calibration/jig_calibration.toml`. The fit
      tools default to that path and refuse to run while it is still zero, since a zero scale
      makes every measured speed zero and produces parameters that look plausible.

**Pass:** standard error under 0.3% across passes. Forward and reverse scale should agree; if they do
not, look for a mounting that is not square rather than accepting two numbers.

**Produces:** `meters_per_count`, the constant every linear measurement depends on.

## E5. Lever arms

**Duration:** 20 min. **Robot motion:** driven, slow spins only.

The IMU and the encoder wheel are both mounted somewhere other than the yaw axis, so a pure spin
produces signals that are not real translation. Measuring the offsets lets us remove them, which is
what makes the combined lin+ang runs in E13 usable instead of discarded.

- [ ] Clear a 2 m circle.
- [ ] Command pure spins at four increasing yaw rates. **Keep the top rate under 8 rad/s.** The wheel
      scrubs sideways the whole time. Hold each rate 4 s.
- [ ] Regress measured lateral acceleration against measured yaw rate squared. The slope is `r_imu`.
- [ ] Regress encoder rate against gyro yaw rate over the same segments. The slope is `r_enc_perp`.

**Pass:** both regressions linear with R^2 above 0.95. If `r_imu` disagrees with a tape measure from
the yaw axis to the IMU by more than 10 mm, something is wrong with the axis assignment from E0.

**Stop immediately if:** the encoder wheel chatters, skips, or leaves a scrub mark. Drop the top rate
and note the limit on the session sheet. That limit caps every later angular run: E11, E13, E15,
E16, E18, E21, and E22.

---

# Block 2: Driven battery

This is the data the model is fit from. Everything here needs `velocity_jig_drive.py`, which plays a
scripted command sequence over the trainer link and logs each command with a host timestamp. Phases
marked *manual fallback* can be driven by hand if the script is not ready, at the cost of a much
weaker fit.

Run the full block three times per session, and run three sessions on different days with different
packs. Session-to-session spread is a number we need, not an inconvenience.

## E6. Polarity check

**Duration:** 2 min. **Manual fallback:** yes.

Stage 2 discovered after the fact that both channels were anti-correlated with their command, because
the solved pose frame was inverted. Catch that at the start of the session instead of in the fit.

The robot is tank steered, so the tool mixes each body command into a left and a right wheel command
before it goes out: forward is both channels positive, and a left turn is the right wheel leading. This
check is what confirms the mix landed the right way round on this robot.

- [ ] Drive 1 m forward. Confirm the encoder count increases, and that the robot tracks straight rather
      than arcing. A hard arc means the mix is wrong, not that the trim is off.
- [ ] Turn 90 deg left. Confirm integrated gyro yaw increases.
- [ ] Drive 1 m in reverse. Confirm the count decreases.

**Pass:** all three signs as expected. **Fail: abort the session** and fix the sign convention. Every
run recorded after a sign mismatch is wasted.

## E7. Linear deadzone staircase

**Duration:** 1 min per run. **Manual fallback:** no.

- [ ] Clear 6 m of straight floor.
- [ ] Staircase from 0.01 to 0.10 in steps of 0.01, holding each 2 s, forward.
- [ ] Repeat in reverse.
- [ ] Take the median sustained speed at each level. The deadzone is the level where speed first clears
      5 sigma of the stationary noise.

**Why 0.01:** stage 2 reported the deadzone as 0.04 because that was the smallest step tested and the
robot was already moving there. The real value is somewhere below. This staircase finds it.

**Pass:** at least three levels below the motion threshold and three above. If the robot moves at 0.01,
extend the staircase downward.

## E8. Linear steps

**Duration:** 2 min per run. **Manual fallback:** no.

The most important driven experiment. Produces max speeds both directions, both accel time constants,
and most of the command edges the delay estimate pools over.

- [ ] Clear the longest straight run available. Budget about 1.5 m for the 1.0 step alone.
- [ ] Steps to 0.25, 0.5, 0.75, 1.0, **holding at least 2 s**, returning to zero between each and
      letting the robot fully coast to a stop before the next.
- [ ] Repeat in reverse.
- [ ] **Two seconds is a floor, not a target.** Stage 2 held roughly a fifth of a second, which is
      under the rise, so there was no plateau to read a max speed off. The tool refuses to plan a
      shorter hold; it shortens the program by shuttling direction instead.
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

**Duration:** 1 min per run. **Manual fallback:** yes.

Covered by E8's returns to zero, but worth a dedicated run because decel was the better-measured
constant in stage 2 and it is what drives stopping overrun.

- [ ] Accelerate to a steady 0.6 forward, hold at least 2 s, then drop the command to zero instantly.
- [ ] Let the robot coast to a complete stop before the next repetition.
- [ ] Ten repetitions forward, ten reverse.

**Pass:** ten clean tails per direction, each with at least 100 ms of decay before the robot stops.

## E10. Angular deadzone staircase

**Duration:** 1 min per run. **Manual fallback:** no.

Low rate throughout, so the encoder stays on. Having it on is useful here: it catches unintended
linear motion during what should be a pure rotation.

- [ ] Staircase from 0.01 to 0.10 in steps of 0.01, holding each 2 s, left.
- [ ] Repeat right.
- [ ] Confirm from the encoder that linear drift stayed small. Large drift means the robot is walking
      rather than pivoting, and the angular fit will be contaminated.

## E11. Angular steps

**Duration:** 2 min per run. **Manual fallback:** no.

- [ ] Confirm the gyro range is 4000 dps (E0). At 2000 dps this run clips and the result is worthless.
- [ ] Steps to 0.25, 0.5, 0.75, 1.0 **scaled to the E5 rate limit**, holding at least 2 s, returning to
      zero between, spinning left.
- [ ] Repeat right.
- [ ] Watch the encoder wheel. It scrubs sideways for the whole battery.
- [ ] Check the saturation count immediately after the run.

**Pass:** zero saturated gyro samples, and a visible steady state at the top step.

**The cap costs the top of the lever arm.** This is no longer a max-spin experiment: the encoder stays
mounted, so the E5 limit binds here too. Quote the fitted `k_ang` with the cap it was fit under.

**Produces:** `k_ang`, `tau_ang_a`, `tau_ang_d`.

**Cross-checks to run before believing the result:**

- Compare fitted `k_ang` against `2 * k_fwd / track_width`. A fit above that geometric bound is wrong.
- Compare against the stage 2 value of 61.5 rad/s. That number came from camera keypoints and may have
  been inflated by yaw keypoint flips. If the gyro says the true rate is well below it, that is a
  finding about the perception pipeline and belongs in the report.

## E13. Coupling grid

**Rate capped per E5.** **Duration:** 1 min per run. **Manual fallback:** no.

Steer-brake (turning costs forward speed) and angular droop (linear load slows the turn) were both
unmeasured in stage 2, because the combination never survived tag dropouts. With on-robot ground truth
they are straightforward.

- [ ] 4x4 grid of held command pairs: `u_lin` in {0.25, 0.5, 0.75, 1.0} crossed with `u_ang` in
      {0, 0.1, 0.2, 0.3}, capped so yaw rate stays under the E5 limit.
- [ ] Hold each combination at least 2 s, returning to zero between.
- [ ] Remove the lever-arm terms from E5 before fitting, or the rotation contaminates the linear
      measurement.

**Pass:** all 16 cells recorded with the robot in bounds. Cells the floor cannot accommodate get noted
as missing rather than quietly skipped.

**Expected outcome:** possibly nothing. If a coupling coefficient fits within its confidence interval
of zero, it does not go in the model. Three terms that are all real beat six where two are noise.

## E14. Linear PRBS

**Duration:** 30 s per run. **Manual fallback:** no.

Held out of the fit entirely. This is the honest validation set.

- [ ] PRBS on the linear channel, amplitude 0.6, bit period 60 ms, 30 s.
- [ ] Confirm the robot stays in bounds. A PRBS wanders; start it in the middle of the space.

**Why:** steps excite one band well. A 60 ms bit period covers roughly 0.2 to 8 Hz, which brackets the
58-78 ms time constants and the 33 ms frame period. If the model fit from steps also predicts PRBS
data, it generalizes.

## E15. Angular PRBS

**Duration:** 30 s. **Manual fallback:** no.

- [ ] Amplitude limited to the E5 rate limit, 30 s. Clear the circle first: a PRBS at the cap spins the
      robot for the whole run.

Validates the angular model in the regime where we also have linear truth, which is the only regime we
run in now.

## E16. Combined PRBS

**Duration:** 30 s per run. **Manual fallback:** no.

- [ ] Independent PRBS sequences on both channels, linear amplitude 0.6, angular capped, 30 s.

This is the validation set for the coupling terms from E13. Different seeds each repetition.

## E17. Chirp

**Duration:** 30 s per run. **Manual fallback:** no.

- [ ] Logarithmic sweep 0.2 to 8 Hz, amplitude 0.4, on the linear channel, 30 s.
- [ ] Repeat on the angular channel, capped.

Gives a frequency response to cross-check the fitted time constants. A first-order model with tau of
58 ms puts its corner near 2.7 Hz; the chirp should show that corner where the step fit says it is.

## E18. Operator driving

**Duration:** 60 s each. **Manual fallback:** yes, this phase is manual by definition.

The holdout with a realistic command distribution. Scripted excitation covers the space evenly; a
driver does not, and the model has to work on what a driver actually does.

- [ ] Drive as in a match, but keep spins under the E5 rate limit. 60 s.
- [ ] A second run with a different driver beats one long run by one driver.

Note on the sheet who drove and roughly what they were doing. "Chasing a corner cone aggressively"
is useful context when a residual spikes.

## E19. Closure runs

**Duration:** 2 min. **Manual fallback:** yes.

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

**Duration:** 10 min. **Manual fallback:** no.

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

**Duration:** 45 min. **Location:** the arena, in view of the ZED.

One session produces the measurement noise model, the projection bias map, and the A/B test bed for
comparing the new filter against `RobotFrontBackSimpleFilter`.

- [ ] Set up in the arena with the camera in its match position.
- [ ] Record on the **Jetson, live**: SVO plus MCAP with perception outputs, filter state, and
      commands. Do not plan to re-derive perception by replaying the SVO on a desktop later. Desktop
      replay warps frames a few percent relative to the live path, which would show up as a fake
      measurement bias.
- [ ] Align three clocks: jig to host by `TIME` probe, host to Jetson by NTP or a shared command log.
- [ ] Run E8, E11, E13, E14, E16, E18 inside the arena, all capped as usual.

**Produces:** measured R as a function of range and image position, the flat-plane projection bias,
the dropout distribution under real motion, and ground-truthed pose error for both filters.

## E22. Weapon-spinning delta

**Duration:** 15 min. **Safety bar: highest in this document.**

A spinning weapon carries angular momentum, so yaw response changes: the chassis resists yaw
acceleration about the weapon's axis, and precession couples the axes. The model does not cover it.
This experiment measures how big the error is, so we know whether it needs covering.

**Do not run this until every other experiment is complete and the plant model is fit.** It is the
only experiment with a live weapon, and it exists to quantify a known model gap, not to fit
parameters.

- [ ] Full safety setup: enclosure or containment, everyone clear, weapon spun up outside the robot's
      drive path first.
- [ ] Nothing fragile stays mounted. The encoder wheel does, so the E5 cap stays on.
- [ ] Repeat the E11 capped angular sequence with the weapon at match RPM.
- [ ] Compare `k_ang` and `tau_ang_a` against the weapon-stopped values from E11.

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

E5 ENCODER RATE LIMIT: ________ rad/s     (caps E11, E13, E15, E16, E17, E18, E21, E22)

RUNS
 #  | LOG file | Experiment | Pack V | Samples | DROP | Clock pre | Clock post | Notes
----+----------+------------+--------+---------+------+-----------+------------+-------
 1  |          |            |        |         |      |           |            |
 2  |          |            |        |         |      |           |            |
 3  |          |            |        |         |      |           |            |
 4  |          |            |        |         |      |           |            |
 5  |          |            |        |         |      |           |            |
 6  |          |            |        |         |      |           |            |
 7  |          |            |        |         |      |           |            |
 8  |          |            |        |         |      |           |            |
 9  |          |            |        |         |      |           |            |
10  |          |            |        |         |      |           |            |

DISCARDED RUNS AND WHY
________________________________________________________________________________
________________________________________________________________________________

ANYTHING THAT SURPRISED YOU
________________________________________________________________________________
```

# Quick reference

| # | Experiment | Duration | Produces |
|---|-----------|---------:|----------|
| E0 | Firmware and range check | 15 min | axis map, saturation counting |
| E1 | Clock probe verification | 20 min | offset and skew under 2 ms |
| E2 | Gyro bias and Allan variance | 35 min | bias, ARW, RRW |
| E3 | Gyro scale factor | 15 min | scale correction |
| E4 | Encoder scale | 20 min | meters per count |
| E5 | Lever arms | 20 min | `r_imu`, `r_enc_perp`, rate limit |
| E6 | Polarity check | 2 min | session abort gate |
| E7 | Linear deadzone | 1 min | `dz_lin_fwd`, `dz_lin_rev` |
| E8 | Linear steps | 2 min | `k_fwd`, `k_rev`, `tau_lin_a/d`, delay edges |
| E9 | Coast tails | 1 min | `tau_lin_d` |
| E10 | Angular deadzone | 1 min | `dz_ang_l`, `dz_ang_r` |
| E11 | Angular steps | 2 min | `k_ang`, `tau_ang_a/d` |
| E13 | Coupling grid | 1 min | `c_sb`, `c_ad` |
| E14 | Linear PRBS | 30 s | holdout validation |
| E15 | Angular PRBS | 60 s | holdout validation |
| E16 | Combined PRBS | 30 s | coupling validation |
| E17 | Chirp | 60 s | frequency-domain cross-check |
| E18 | Operator driving | 2 min | realistic-distribution holdout |
| E19 | Closure runs | 2 min | ground-truth drift check |
| E20 | Pack state repeat | 10 min | voltage dependence of `k_fwd` |
| E21 | Joint jig and camera | 45 min | R, projection bias, A/B bed |
| E22 | Weapon-spinning delta | 15 min | angular model gap size |

Block 0 runs once per hardware change. Block 2 runs three times per session, three sessions total.
Block 0 takes about two hours. One full pass of block 2 takes about 30 minutes of recording: the 2 s
minimum hold makes every step battery longer than the runbook's first draft assumed.

## Next steps

1. Run E0 and decide on the gyro and accel ranges before anything else is recorded.
2. Run E1. If the clock probe does not hold 2 ms, stop and fix it; the delay estimate depends on it.
3. Record a first session with `velocity_jig_drive.py`, then fit and read the report's
   "what to collect next" table before planning the rest.
4. Run blocks 0 and 1, then a first pass of block 2, and fit before scheduling the remaining sessions.
   One fit on real data will change some of these procedures, and it is cheaper to find that out after
   one session than after three.
