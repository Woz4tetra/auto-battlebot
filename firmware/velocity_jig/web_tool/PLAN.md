# Velocity jig web tool: plan

A browser page that runs the session sheet from
[`velocity_jig_runbook.md`](../../../docs/experiments/kalman_filter/velocity_jig_runbook.md) so an
operator can get through 23 experiments in an afternoon without writing anything down by hand.

The jig already announces the two facts that are most annoying to transcribe. `startRecording()` prints
`recording LOG-7.TXT` when button A is pressed, and `stopRecording()` prints
`stopped, n=98304 dropped=0` when button B is pressed. Both arrive unprompted on USB serial. A page
holding the port sees them and binds them to whichever experiment is selected, which is the whole
premise of this tool: the log file gets labeled at the instant it is created, not from memory later.

## Goal

Cut per-run bookkeeping from about two minutes of writing to one number typed (pack voltage) plus one
button press. Evaluate the runbook's abort gates automatically at the moment they can still be acted
on, so a bad run is discarded and redone in the same session rather than discovered during the fit.

## Non-goals

| Out of scope | Owner |
|---|---|
| Fitting the plant model | Offline Python in `training/` |
| Bulk log transfer | Pull the SD card. `GET` over USB is for spot checks only |
| Live gyro bias checking during a recording | Not possible; see [Blind during recording](#blind-during-recording) |

Driving the robot **is** in scope. `velocity_jig_drive.py` does not exist yet, and building it here instead
of in Python is the better trade. See [Driving the robot](#driving-the-robot).

## Framework: none

Static files, vanilla ES modules, no build step, no package manager, no CDN.

- **Web Serial API** is native in Chromium, so the page talks to `/dev/ttyACM0` with no backend, no
  Python process, and nothing to keep alive between the browser and the hardware.
- A workshop may have no internet. Every byte is served locally.
- Copying one directory to the Jetson or a laptop is the entire install.

What that costs:

| Constraint | Detail |
|---|---|
| Browser | Chrome, Edge, or another Chromium. Firefox and Safari have no Web Serial |
| Origin | Must be served. `http://localhost:PORT` is a secure context; `file://` is not |
| Port access | Linux user must be in `dialout`, or the port picker shows nothing |
| Exclusivity | Web Serial takes the port. Close `pio device monitor` and `scripts/console` first |

## Console protocol

Read from `src/main.cpp`. Everything the tool relies on is already implemented.

| Direction | Line | Notes |
|---|---|---|
| Jig, unprompted | `recording <name>` | Button A. Binds the run |
| Jig, unprompted | `stopped, n=<samples> dropped=<overflow>` | Button B. Two gate inputs at once |
| `LIST` | `F <name> <size>` per file, then `END` | Idle only |
| `GET <name>` | `SIZE <n>`, then `n` raw bytes, then `\nEND\n` | Binary. Parser must switch modes |
| `DEL <name>` | `OK` or `ERR` | Idle only |
| `TIME` | `TIME <rx_us> <tx_us>` | Answered during recording as well |
| `STREAM` | `STREAM`, then `S t_us,count,gx,gy,gz,ax,ay,az` at 10 Hz, then `END` | Any input stops it |
| `BOOTSEL` | `REBOOTING to bootloader` | |
| anything else | `ERR unknown` | |

Three behaviors that constrain the design:

1. **Blind during recording.** The recording branch answers `BUSY` to everything except `TIME`. No
   `LIST`, no `STREAM`, no gyro readback while a run is in progress. Every gate other than the clock
   probe is evaluated at stop time or offline.
2. **`STREAM` blocks the console loop.** It cannot be interleaved with `TIME`. The tool serializes
   them and always ends a stream before probing.
3. **`STREAM` exits into a recording** if button A is pressed while it runs, printing `END` and then
   `recording <name>`. The parser must handle that sequence rather than treating `END` as terminal.

## Driving the robot

The OpenTX radio is a second USB CDC device (VID `0x0483`, PID `0x5740`) at 115200 baud speaking plain
ASCII, mirroring `src/transmitter/opentx_transmitter.cpp`:

```
telemetry on\r\n          prime, once at open
channels on\r\n           prime, once at open
trainer 0 <value>\r\n     linear,  value in [-500, 500]
trainer 1 <value>\r\n     angular, value in [-500, 500]
```

Web Serial holds two ports at once, so the page drives the robot and reads the jig in the same tab.
Raw commands are sent with no deadzone pre-compensation, since the physical deadzone is the thing E7
and E10 measure.

**The reason to build it here rather than as a Python script:** the command log and the clock probe end
up on the same clock by construction. `drive_protocol.py`'s docstring makes this argument for
`apriltag_track.py`, which shares `CLOCK_MONOTONIC` between commands and camera ground truth. A separate
Python driver would reintroduce exactly the host-to-host alignment problem the `TIME` probe exists to
solve, and would add a second unknown offset on top of it. One process, one `performance.now()`, one
offset to the jig.

Command timestamps are taken at the `write()` call and carry roughly one 1 ms USB frame of buffering
uncertainty. That is the same uncertainty `pyserial` has, and it is small against the ~60 ms transport
delay being measured.

### Timing

Send at 50 Hz. Jitter does not need to be small, because every command is timestamped and the fit is a
prediction-error method that takes non-uniform command times without complaint. OpenTX holds the last
value between updates, so a late frame is a slightly irregular command edge, not a dropout.

### Safety

The robot does 5.6 m/s. A browser tab driving it needs more care than a Python script, because tabs get
backgrounded.

| Hazard | Mitigation |
|---|---|
| Tab backgrounded, timers throttle to ~1 Hz, robot holds last command | Page Visibility API disarms immediately on hide |
| Screen sleeps mid-run | Screen Wake Lock held while armed |
| Command loop stalls | Watchdog disarms if a send misses its deadline by more than 100 ms |
| Tab closed or browser crashes | `beforeunload` disarm is best effort only. The real failsafe is below |
| Protocol runs long | Hard wall-clock timeout per protocol, then disarm |

**The human driver's SF arm switch is the failsafe, not the software.** Every mitigation above is a
convenience. Arming is a separate, explicit control in the UI from arming a recording, and the tool
refuses to send anything nonzero until it is set. Keep the driver's sticks centered: trainer mode adds
stick input to the command.

## Space budget

The workshop is shorter than the runbook assumes, and even the 2.4 m field is tight. Using stage 2's
numbers, a full-amplitude forward step needs more room than it looks.

With `v_ss = 5.60 m/s`, `tau_a = 0.058 s`, `tau_d = 0.078 s`, and `L_d = 0.060 s`, the distance for a
step held `T` seconds past the edge and then commanded to zero is:

```
d_rise  = v_ss * (T - tau_a * (1 - exp(-T / tau_a)))
d_delay = v_ss * L_d           robot keeps full speed through the transport delay
d_decay = v_ss * tau_d         first-order coast to rest
```

| Dwell | Settling | d_rise | d_delay | d_decay | Total |
|---|---|---:|---:|---:|---:|
| `3*tau_a` = 0.174 s | 95% | 0.67 m | 0.34 m | 0.44 m | **1.44 m** |
| `5*tau_a` = 0.290 s | 99% | 1.30 m | 0.34 m | 0.44 m | **2.07 m** |

Everything scales linearly with commanded amplitude `a`, since `v_ss = a * 5.60`. So the tool solves
for the amplitude that fits the space:

```
a = min(1, budget / d_total(1.0, T))
```

### Length is a live control

Usable length is a slider at the top of the run screen, not a setup constant. Everything downstream of
it recomputes on every change, with no reload and no re-entry:

| Recomputed | Shown as |
|---|---|
| Amplitude scale per experiment | The commanded fraction, and the peak speed it implies |
| Predicted distance | Against the budget, with the margin left over |
| Predicted lateral excursion | Against usable width, using the current trim residual |
| Binding constraint | Which of length or width runs out first |
| Experiment status | Green fits at full amplitude, amber fits reduced, red needs more room |
| E4 pass count | Solved from the reference distance actually available |
| Shuttle cycles and drift | Cycles that fit before the robot walks out of the budget |

Usable width is the second control, entered as board width minus robot width. See
[Width binds before length](#width-binds-before-length).

It runs backwards too. Enter a target instead of a length and the tool reports the length required, so
"what board do I need to measure `max_linear_speed`" has a number rather than a guess. The length can
change mid-session without invalidating anything already recorded, since each run stores the budget and
amplitude it actually used.

### Choosing a board

Assuming 0.25 m of placement slack at each end, so usable length is the board minus 0.5 m. Amplitude is
the fraction of full command that fits, capped at 1.0.

| Board | Usable | `a` at 3τ | `a` at 5τ | Max speed measurable |
|---|---:|---:|---:|---|
| 6 ft / 1.83 m | 1.33 m | 0.92 | 0.64 | no |
| 8 ft / 2.44 m | 1.94 m | **1.00** | 0.94 | no |
| 10 ft / 3.05 m | 2.55 m | 1.00 | **1.00** | no |
| 12 ft / 3.66 m | 3.16 m | 1.00 | 1.00 | marginal, ~0.19 s of steady state |
| 16 ft / 4.88 m | 4.38 m | 1.00 | 1.00 | yes, with room |

Reading it: **10 ft is the knee.** It buys full amplitude at 99% settling, which covers every time
constant, both deadzones, and all the broadband runs. Going from 8 ft to 10 ft upgrades settling from
94% to full. Going from 10 ft to 12 ft buys only a sliver of steady state, and measuring
`max_linear_speed` properly wants 16 ft. That last one is a single sprint, so it is the right thing to
borrow field time for rather than to buy lumber for.

Reverse runs need less room than forward (4.84 m/s against 5.60), so a forward and reverse shuttle is
always limited by the forward leg.

### Width binds before length

An open-loop straight run on a differential drive does not go straight. A yaw rate bias `w` at speed
`v` puts the robot off the centerline by:

```
y = 0.5 * w * L^2 / v
```

The bias comes from side-to-side mismatch. For a speed difference `dv` across a track width `W`,
`w = dv / W`. On a beetleweight with `W` near 0.1 m, a 2% mismatch at 5.6 m/s is `dv = 0.11 m/s` and
`w = 1.1 rad/s`, which over a 2.5 m run is 0.61 m of lateral excursion. Cutting the mismatch to 0.2%
still leaves 61 mm.

That number is an estimate from the geometry, not a measurement, and the first trim run replaces it.
The point stands either way: on a board narrow enough to carry into a workshop, **clearance runs out
before length does**, and the excursion grows with the square of run length while the space budget only
grows linearly.

So usable width is a second live control alongside length, and the tool reports whichever binds first.

### Straight-line trim

Most of the veer is removable, and removing it is a setup step the tool can do closed loop before any
recording starts.

`STREAM` gives live gyro at 10 Hz while the jig is idle. The tool drives at low linear amplitude, reads
mean yaw rate, and servos an angular trim until the yaw rate sits at zero. Nothing is recorded, the
robot never leaves walking speed, and the result is one number that makes the subsequent full-amplitude
runs go roughly where they are pointed.

Two things this must not do:

1. **The trim is a command, not a hidden offset.** It is written into the command log as commanded
   angular, because a linear step with a trimmed angular channel is a two-input excitation and the fit
   has to see both. Burying it in a calibration constant would put a systematic error into every linear
   parameter and leave no trace of where it came from.
2. **It does not replace E13.** The trim is the low-speed end of the same angular droop that `c_ad`
   describes, so the tool records each trim value as a data point rather than as a setting. Trim is
   also amplitude dependent, so it is fit as a fraction of linear command rather than a constant, and
   the residual veer at full amplitude is what the rail catches.

### If the robot drives on the board

The board's surface is not the arena's, and friction is not a nuisance here, it is half of what these
experiments measure. `tau_d` is friction braking almost entirely, the deadzones are the static friction
needed to break away, and `tau_a` is surface dependent whenever the robot is traction limited rather
than motor limited. Fitting those on plywood and deploying on the arena floor bakes in a bias that no
amount of careful measurement removes.

This is the same shape of problem as the encoder wheel in E12, and it takes the same fix: measure the
delta. Carry one short capped step through to the arena session in E21 and repeat it on the real floor,
then compare `tau_d` and the deadzones against the board fit. If they agree inside the run-to-run
spread, the board was free. If they do not, every board-fit parameter needs the correction.

A straightedge clamped along the board gives a rail, which is what makes a narrow board usable at all
given the excursion numbers above. The rail must be a **backstop, not a guide**. A robot riding against
it picks up contact friction, a lateral force, and a yaw moment, which contaminates the linear channel
the same way the encoder wheel does and suppresses exactly the heading behavior E13 is trying to
measure.

Contact is detectable rather than assumed. A rub shows up in the log as a yaw rate impulse correlated
with a linear deceleration, so it becomes a post-run gate: any run with rail contact during the
excitation is discarded, and the trim gets another pass. Trim first, rail second, and treat rail
contact as a failed run rather than a successful save.

For E4 the clean configuration is the straightedge alone, with the robot hand-pushed beside it rather
than driven on the board. There is no slip and no drive, so surface never enters, and the straightedge
is a pure measurement reference. Do E4 that way regardless of how much floor is available.

### Knobs, in the order worth spending

1. **Shorten the dwell to `3*tau_a`.** Costs 4 percentage points of settling and saves 0.63 m. A
   prediction-error fit recovers `tau` from the shape of the rise and does not need full settling, so
   this is nearly free.
2. **Shuttle instead of one-way.** A forward step followed by a reverse step retraces the same ground.
   Reverse is slower (4.84 m/s), so net drift is about 0.2 m per cycle rather than 1.44 m, and E8 needs
   both directions anyway. Same space, half the runs.
3. **Scale amplitude.** `tau_a` and `tau_d` are amplitude-independent in a first-order model, so they
   stay identifiable at reduced amplitude. What you lose is the top of the range, where saturation and
   any real nonlinearity live. Measure `max_linear_speed` with one sprint in the field and do the rest
   in the workshop.
4. **Zero-mean excitation.** A plain PRBS random-walks in position. A DC-corrected sequence, or one
   wrapped in a shuttle that reverses on a distance budget, stays near the start.

Angular experiments spin in place and need almost no room, so the tool sorts the remaining work by
space demand and flags which experiments actually require the field.

### E4 at less than 3 m

The runbook asks for 20 passes over 3.000 m. The reference distance is a parameter, not a constant. For
a per-pass placement error `e` and target relative standard error `s`, the passes needed are:

```
N = (e / (D * s))^2
```

At `e = 3 mm` and `s = 0.3%`, a 3.000 m reference needs `N = 0.11` and a 2.000 m reference needs
`N = 0.25`. Both round to one pass, so the runbook's 20 is already dominated by something other than
tape error, most likely wheel slip and lift. Shortening to 2 m costs almost nothing. The tool takes `D`
as input, computes `N`, and rounds up generously, because passes are cheap and a remount is not.

### Live protection

While a protocol runs, the tool integrates the commanded profile through the current plant estimate and
displays predicted distance from the start, aborting when it exceeds the budget. This is open loop and
partly circular, since the model is what we are fitting, so the safety integral uses worst-case
parameters (highest `v_ss`, longest `tau_d`, longest `L_d`) while planning uses best estimates. The
firmware `POS` follow-up below turns this into a closed loop.

## Automatic encoder detection

The runbook's abort gate is "frozen encoder count on a linear run". The tool checks this with no user
interaction, using a property of the firmware: `startRecording()` sets `g_encCount = 0` under
`noInterrupts()`, and nothing resets it afterward. So after button B, the free-running count equals the
net quadrature counts accumulated during that run.

A half-second `STREAM` immediately after the run stops reads it:

| Experiment requires | Count after run | Verdict |
|---|---|---|
| attached | `abs(count) > 100` | pass |
| attached | `abs(count) <= 100` | discard, encoder frozen or unplugged |
| detached | `count == 0` | pass |
| detached | `count != 0` | discard, encoder was left plugged in |

This works for spin-in-place runs with the encoder attached (E5, E10, E13), because the wheel sits off
the yaw axis and the lever arm turns it. The count moving is exactly the signal `r_enc_perp` is fit
from.

The diagnostics screen distinguishes a disconnected encoder by its pin levels (`AB 11`, both pulled
up), but `STREAM` rows carry only the count, not `AB`. Adding `AB` to the stream row is a three-line
firmware change and would let the tool check encoder state *before* a run instead of after. Listed
under [Firmware follow-ups](#firmware-follow-ups), not done here.

## Clock probe

Per probe `i`, using `performance.now()` in milliseconds:

```
RTT_i      = t_recv_i - t_send_i
host_mid_i = (t_send_i + t_recv_i) / 2
jig_mid_i  = (rx_us_i + tx_us_i) / 2000
offset_i   = host_mid_i - jig_mid_i
```

Keep the lowest decile of probes by `RTT`. USB Full Speed polls in 1 ms frames, so most of the spread
is host scheduling, and the fastest probes are the ones whose transport was most symmetric. Report the
median offset of the survivors and the RMS of their residuals about it.

Skew is not fit inside a burst. A 200-probe burst spans about two seconds, and 30 ppm of crystal drift
is 0.06 ms over that window, which is below the noise. It is fit across the run instead:

```
skew_ppm = (offset_post - offset_pre) / (jig_mid_post - jig_mid_pre) * 1e6
```

Expect roughly 30 ppm, which is about 0.9 ms over a 30 s run.

`performance.now()` may be coarsened to 100 us or worse without cross-origin isolation. That is well
under the 2 ms gate, so no special headers are needed.

## Gates

Evaluated automatically at stop time unless noted.

| Gate | Input | Discard when |
|---|---|---|
| Dropped samples | `stopped` line | `dropped != 0` |
| Clock residual | pre and post bursts | residual RMS > 2 ms in either |
| Encoder state | post-run `STREAM` count | mismatch per the table above |
| Sample count | `n` against duration times ODR | `n < 0.9 *` expected |
| Still holds | coach timestamps | either hold under 9 s |
| Gyro bias drift | log contents | **offline.** Pre and post bias differ by > 0.05 deg/s |

The bias-drift gate cannot run live, because the tool is blind during a recording. The tool records the
two still-hold windows as host time ranges so the offline script knows exactly which samples to average
instead of guessing from the file.

Every verdict is overridable with a required note. A discarded run stays in the export with its reason,
since knowing which runs were thrown away and why is part of the record.

## Screens

**Run.** The active experiment: encoder badge, duration, what it produces, condensed procedure. Arm
button, then a coach that walks the run card: hold still 10 s with a countdown and a beep, excitation,
hold still 10 s, press B. Gate results appear the moment the stop line arrives.

**Diagnostics.** Live `STREAM` at 10 Hz: encoder count, gyro in dps, accel in g, saturation flags at
`abs(raw) > 32000`. Computes the gravity unit vector `u = accel / norm(accel)` and shows the projected
yaw rate `dot(gyro, u)`, which is how E0 derives the yaw axis. Covers E0's hand-rotation wiring check
and the attach procedure's roll-the-robot check.

**Session.** Run table, progress against the 23 experiments, remaining time estimate, export.

**Console.** Collapsible raw serial for `LIST`, `DEL`, `BOOTSEL`, and debugging.

IMU scale factors are a session setting, defaulting to 0.070 dps/LSB and 0.000244 g/LSB for the
current +/-2000 dps and +/-8 g configuration, with presets for the +/-4000 dps and +/-16 g ranges E0
may switch to before E11 and E22.

## Workflow

1. `cd firmware/velocity_jig/web_tool && python3 -m http.server 8000`
2. Open `http://localhost:8000` in Chrome. Connect the jig, then connect the transmitter.
3. Set usable run length and usable width. Both are live controls, so change them whenever the space
   changes. The tool says which one binds.
4. Run the trim pass once per surface and pack. Low speed, not recorded, closed loop on live gyro.
5. Select the experiment. Check the encoder badge against what is physically plugged in. The tool shows
   the amplitude it will use and the predicted distance, and says so plainly if the experiment does not
   fit and needs the field.
6. Enter pack voltage. It carries over to the next run.
7. Click **Arm**. The pre-run clock probe fires.
8. Press **A** on the jig. The coach starts on the `recording` line.
9. Hold still through the countdown. The tool plays the excitation and disarms at the end of it. Hold
   still again.
10. Press **B**. The post probe fires, the encoder check runs, gates evaluate, the row is written.
11. Repeat. Export JSON and Markdown at the end.

Steps 8 and 10 stay manual button presses on the jig rather than software triggers, because the jig's
buttons are the only start and stop that cannot be lost to a USB problem.

## Data model

```
Session { id, startedAt, operator, robot, imu: {gyroDpsPerLsb, accelGPerLsb}, notes, runs: [Run] }

Run {
  id, experimentId, variant,            // variant: "attached" | "detached" for both-encoder experiments
  logFile, samples, dropped,
  tStartHost, tStopHost, durationS,
  holdPre: {start, end}, holdPost: {start, end},
  packVoltage,
  clockPre:  {offsetMs, residualMs, kept, total, atHostMs, atJigMs},
  clockPost: {...}, skewPpm,
  encoderExpected, encoderCountAfter,
  protocol: {name, params, spaceBudgetM, amplitudeScale, predictedPeakMs, predictedDistanceM},
  commands: [{tHost, linear, angular, trainerLin, trainerAng}],
  gates: [{name, pass, detail}],
  verdict: "pass" | "discard", overridden, notes
}
```

`commands` is the command log the fit is built from. Its timestamps and the clock probe's share one
`performance.now()` origin, so aligning commands to jig samples needs only the single jig offset.

Autosaved to `localStorage` on every change. Export writes JSON for the fit scripts and Markdown
matching the runbook's session sheet columns. Import restores a session to resume after a browser
restart.

## Failure modes

| Failure | Behavior |
|---|---|
| Browser refresh mid-session | `localStorage` restores. A run in progress is marked incomplete |
| Jig USB unplugged mid-run | **The jig keeps recording to SD.** It is not driven by the host. The tool flags the missing post probe and the run needs a manual sample count from the summary screen |
| Transmitter USB unplugged mid-run | Write throws, watchdog fires, tool marks the run discarded. OpenTX holds the last value, so **flip SF** |
| Port held by `pio device monitor` | Connect fails with a message naming the likely culprit |
| No `navigator.serial` | Feature-detected on load with setup instructions instead of a dead page |
| Stop line missed | Run stays open. Manual close with typed values, flagged as manually entered |
| Space budget exceeded mid-protocol | Disarm and mark discarded. Predicted overshoot is logged so the budget can be retuned |

## Files

```
firmware/velocity_jig/web_tool/
  PLAN.md          this document
  README.md        how to run it
  index.html
  app.css
  jig.js           Web Serial transport, line and binary framing, console commands, clock probe
  trainer.js       OpenTX link: second serial port, arming, watchdog, disarm paths
  excitation.js    protocol generators: step, staircase, PRBS, chirp, coupling grid, shuttle
  plant.js         current parameter estimates, distance and lateral integrals, amplitude solver
  trim.js          closed-loop straight-line trim against live STREAM gyro
  experiments.js   the 23 experiments as data: encoder state, duration, produces, gates, protocol
  session.js       run records, gate evaluation, persistence, JSON and Markdown export
  app.js           UI wiring, run coach, space budget panel
  mock.js          simulated jig and transmitter for testing without hardware
```

`mock.js` emits `recording` and `stopped` lines, answers `TIME` with a synthetic offset and skew, and
streams plausible `S` rows. It exists so the whole flow, including every gate path, can be verified
without the robot, and so the operator can rehearse the session before spending the afternoon on it.

## Build order

1. `jig.js` plus `mock.js`, verified against the mock: framing, `TIME` burst, `STREAM` start and stop.
2. `experiments.js` transcribed from the runbook quick reference table.
3. `session.js`: gates and export, tested against mock runs covering each discard path.
4. `app.js` and the markup: run coach, then diagnostics, then session view.
5. `plant.js` and `excitation.js` against the mock, checking predicted distance against the integral by
   hand for the two dwell cases in the space budget table.
6. `trainer.js` last, and bench it with the robot **up on blocks** before any wheel touches the floor.
   Verify every disarm path first: hide the tab, sleep the screen, kill the watchdog, close the tab.
7. `README.md` once the flow is settled.

## Firmware follow-ups

None is required. Each removes a manual step or a safety compromise, and none is done yet.

1. **`POS` command, answered during recording.** Replies `POS <t_us> <count>` by reading `g_encCount`
   under a brief `noInterrupts()`. Same pattern as `TIME`: touches neither the SD card nor the capture
   path. Polled at 20 to 50 Hz it turns the space budget from an open-loop integral into closed-loop
   distance feedback, so the excitation reverses on measured position instead of predicted position.
   This is the highest-value change of the three, and it is what makes a short workshop safe rather
   than merely calculated.
2. **Add `AB` pin levels to the `STREAM` row.** Lets the tool verify encoder state before a run rather
   than inferring it after, and turns a discarded run into a blocked one.
3. **Per-axis saturation counters in the log header.** E0 asks for a saturation count; today the tool
   can only flag it live in diagnostics, and offline analysis has to scan the whole file.

## Next steps

1. Measure the workshop, then decide on a board. 10 ft is the knee: it buys full amplitude at 99%
   settling and covers every parameter except `max_linear_speed`, which is one sprint and better
   borrowed from field time than bought in lumber.
2. Build in the order above, against `mock.js`. Bookkeeping first, driving last.
3. Add the firmware `POS` command. Everything about running in a short space gets safer with closed-loop
   distance, and it is roughly ten lines following the `TIME` pattern.
4. Dry-run the full 23-experiment flow on the mock and time it.
5. Bench the drive path with the robot on blocks, checking every disarm path, before any wheel touches
   the floor.
6. Run E0 and E1 with the tool on real hardware before committing the afternoon to it.
