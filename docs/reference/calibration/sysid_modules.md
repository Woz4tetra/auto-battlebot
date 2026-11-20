# Sysid modules

How the sysid code under `auto_battlebot/calibration/` and `playground/calibration/` works. The procedure that runs it is `docs/guides/sysid_guide.md`.

## Module reference

Design notes for the Python modules behind the sysid workflow, grouped by module.

Nothing under `calibration/sysid/` sends a command: the robot is driven by a person, and the package only watches. `test_sysid_package_never_sends_commands` fails if it imports a serial, socket, jig, MuJoCo or RL module.

### `calibration/jig/drive_protocol.py`

Trainer link to the OpenTX/EdgeTX radio: writes commands, reads back what was sent.

Mirrors the trainer protocol in `src/transmitter/opentx_transmitter.cpp`. The radio is a USB
CDC device (VID 0x0483, PID 0x5740) primed once with `telemetry on` + `channels on`, then
driven with `trainer <channel> <value>` at 50 Hz, value in [-500, 500].

**One port, both directions.** The same serial port that accepts `trainer` writes also
streams the radio's mixer output back once primed. Reading that back matters because
trainer mode *adds* to the human driver's sticks: the mixer output is the command the robot
actually received, not the one this process asked for. Two consequences:

- A hand-driven run has a real command log. Without the readback it has a column of zeros
  and is unusable for fitting.
- A scripted run gets a contamination check. If the driver's stick is not centered, measured
  diverges from commanded and the run can be flagged instead of quietly poisoning the fit.

Commands are sent RAW. The deployed transmitter applies `DifferentialDriveProcessor`'s
lifted and zero deadzones; this does not, because the physical deadzone is exactly what is
being measured and pre-compensating it here would measure the compensation.

SAFETY
- The robot moves fast. Run in a clear, bounded space with guard plates on.
- Keep the human driver's sticks centered: in trainer mode the radio ADDS stick input.
- Zero the channels and disarm on every exit path: normal return, Ctrl-C, exception, and a
  hard wall-clock timeout. `Armed()` is a context manager that does this.

#### `_SYNC`

Radio channel-stream framing: sync, phase byte (0 = channels 1..16, 1 = 17..32), length
byte, 16 little-endian int16, then a checksum equal to phase ^ length ^ every data byte.

#### `Measured`

One tick of the named channels, as the radio reported them.

A record rather than a tuple: this is unpacked at three call sites and the drive pair
reads the same as any other pair of floats, so a widening tuple is a silent reordering
waiting to happen.

#### `stream_measured`

Ticks from the radio alone, nothing commanded.

This is how a hand-driven run gets a command log: the tool sends nothing, the operator
drives, and the radio reports what it sent. `linear` and `angular` stay zero because
nothing was asked for; the measured channels are the whole record of the run, which is
why the weapon pair rides along with the drive pair here.

#### `MixConfig`

How a body command maps onto the two trainer channels, and how the readback is un-mixed.

| Field | Meaning |
| --- | --- |
| `channel_scale` | Full scale of the readback stream |
| `read_linear`, `read_angular`, `read_weapon`, `read_arm` | Slots into the radio's channel stream. The stream is 0-based, so slot 0 is what the radio's own screen calls CH1. On this model CH1 and CH2 are drive, CH3 is weapon throttle and CH5 is weapon arm. CH4 is unused and is not logged |
| `read_invert_a`, `read_invert_b` | Per-channel sign on the read path only. A radio with servo-reverse set on one drive channel reports it negated while the ESC on that side compensates, so the robot moves correctly and only the readback disagrees. Without the inversion a straight command un-mixes into a pure turn and the contamination gate fires on a good run. Find the signs with `velocity_jig_drive.py --check-radio` |

The defaults are `read_invert_a = False` and `read_invert_b = True`: `--check-radio` showed channel B coming back negated on this radio.

#### `PlayResult.SETTLE_TICKS`

`SETTLE_TICKS = 4`: ticks ignored after the command changes. The radio reports its mixer output on its own schedule, so `measured` lags `commanded` by a tick or two. At a step edge that lag makes the two differ by the full step height, which says nothing about the driver, so the contamination check skips those ticks.

### `calibration/jig/excitation.py`

Excitation waveforms for velocity jig runs, declared in TOML.

This replaces the web tool's `excitation.js` plus its hard-coded 21-entry experiment
catalog. The catalog was the runbook compiled into JavaScript, so changing an amplitude or
trying a new excitation meant editing code. Here a waveform is a TOML table:

    [waveform.lin_step_full]
    kind = "step"
    channel = "linear"
    role = "fit"
    amplitudes = [0.25, 0.5, 0.75, 1.0]
    hold_s = 2.0

`kind`, `channel` and `role` are also what the fit routes on, so a new waveform reaches the
right fit without touching the fitter.

Commands are normalized to [-1, 1] and dimensionless. Mapping to the radio's integer range
is the transmitter link's job, and deadzone compensation is deliberately not applied
anywhere: the physical deadzone is one of the things being measured.

#### `Program`

A playable excitation.

`segments` is present for piecewise programs and None for continuous ones (chirp, sine,
triangle, PRBS, manual), where there is no cell structure to slice by. `at(t)` is the
authority either way: the player samples it, and segments exist so the fit can label
samples by phase.

#### `MIN_COAST_S`

`MIN_COAST_S = 1.5` is the shortest coast between cells and the default `coast_s`. It lets the robot come to rest so each step's decay is measured from a standstill instead of from whatever the previous cell left behind.

#### `_build_coast`

Accelerate, hold, then drop to zero instantly and let the robot roll to a stop.

The drop must be instantaneous. A ramp down is a second input and the decel constant
fitted through it is the ramp's, not the robot's.

`reverse_cap` lowers the entry speed of the reverse repetitions only, same reason as in
`_build_step`: the tail has to decay from a straight-line steady state, and a robot that
spun up during the entry is scrubbing the encoder wheel sideways for the whole tail.

#### `_build_grid`

Cross linear against angular to separate the coupling terms.

The angular list must span both signs. Steer-brake and angular droop flip with the turn
direction while straight-line drift does not, so a grid that only turns one way cannot
tell them apart and will fold the drift into the droop coefficient.

#### `_build_chirp`

Swept sine, logarithmic by default.

A first-order model with tau 58 ms puts its corner near 2.7 Hz. A sweep through that
corner is the frequency-domain cross-check on a time-constant fitted from steps.

#### `_build_triangle`

Constant slew between the peaks, so unlike a sine it excites the rate limit evenly
rather than concentrating the fastest command changes at the zero crossings.

#### `_build_prbs`

Maximal-length 16-bit LFSR, taps 16/14/13/11. The sequence matches the retired web tool bit for
bit, so runs recorded before and after the rewrite stay comparable;
`tests/python/test_excitation.py::test_prbs_matches_web_tool_sequence` pins it. A 60 ms bit
period covers roughly 0.2 to 8 Hz, which brackets both the 58-78 ms plant time constants and
the 33 ms frame period. The combined linear/angular program draws a decorrelated second
sequence for the angular channel.

#### `_build_trim`

Low-amplitude straight drive used to measure the arc, not to fit anything.

Run before a session to find out whether a straight run will stay on the board, and to
seed the drift parameter before any fit exists. Driven open loop here; the closed-loop
correction lives in the CLI, which can see the jig's live stream.

### `calibration/jig/jig_fit.py`

#### Motion floors

Per-sample floors below which the measurement is noise, whatever the command says.

| Constant | Value | Meaning |
| --- | --- | --- |
| `MOTION_EPS_LIN` | 0.02 m/s | Linear speed floor |
| `MOTION_EPS_ANG` | 0.05 rad/s | Angular rate floor |
| `ONSET_SIGMA` | 5.0 | "Moving" means this many sigma above the stationary noise, per the plan |
| `MIN_HOLD_S` | 0.4 s | Shortest command hold worth fitting a steady state to |

`stage_a` passes `step_floor = 0.01` as `min_level` to `held_segments`. It sits above the radio's readback quantization and below the smallest commanded step in any ladder (0.1 linear after the reverse cap, 0.244 angular), so it drops rest cells and nothing that was actually driven.

#### `COAST_KINDS`

Kinds that return the channel to zero mid-run, which is where a decay constant is read.
A dedicated `coast` waveform is the one to add when a decel constant comes back weak;
before this routing existed, angular decay could only come from the drops at the end of
angular steps.

#### `fit_drift`

The drift fit uses both columns at once. Fitting them one at a time would let the first
absorb the second, the same trap the mirrored-cell decomposition avoids for droop.

#### `build_windows`

Cut windows out of every run and stack them into one batch.

Runs with a detached encoder carry no linear truth, so they are excluded from the position
objective. Their heading is still measured, which is what makes the angular parameters
fittable at rates the encoder cannot survive; `require_encoder=False` builds that set.

A window needs a known command for every sample it spans. That rules out the still holds at
both ends of every run and rules out an operator-driven run whose sticks were never logged:
with no command, the model predicts a robot that never moves and the residual is pure truth.

#### `residual_autocorrelation`

Lag-1 autocorrelation of the longest-horizon position error across window starts.

Strong autocorrelation says the model is missing a structural term. No amount of process
noise tuning fixes that honestly; it only widens the covariance to cover a bias.

### `calibration/jig/jig_link.py`

USB console client for the velocity jig (`firmware/velocity_jig`).

The jig is a Feather RP2040 Adalogger that rides on the robot and logs a wheel encoder plus
an ISM330DHCX IMU to microSD at 1 kHz. It speaks a line protocol over native USB CDC:

    LIST            -> "F <name> <size>" per file, then "END"
    GET <name>      -> "SIZE <n>", exactly n raw bytes, then "\nEND\n"
    DEL <name>      -> "OK" or "ERR"
    TIME            -> "TIME <rx_us> <tx_us>"
    STREAM          -> "STREAM", then "S t_us,count,gx,gy,gz,ax,ay,az" at 10 Hz until any
                       byte arrives, then "END"

Three firmware behaviors shape this client:

1. **Blind during recording.** Every command except `TIME` answers `BUSY`. `TIME` is the
   exception on purpose: it touches neither the SD card nor the capture path, so a mid-run
   probe measures skew across the run window instead of extrapolating into it.
2. **Recording starts and stops on physical buttons.** There is no `START`. Button A emits
   an unsolicited `recording <name>` and button B a `stopped, n=... dropped=...`, so the
   client latches those lines as they arrive rather than only while something is waiting.
   Operators press A during the clock probe burst, and an unlatched press hangs the CLI.
3. **`STREAM` stops on any byte** and can exit into a recording: pressing A mid-stream
   prints `END` and then `recording <name>`, so `END` is not necessarily terminal.

Baud is nominal. This is native USB CDC, so transfers run at USB Full Speed regardless.

#### `find_jig_port`

Locate the jig, never the transmitter.

Exact USB id match first. Falling back to "any ACM port that is not the radio" keeps the
tool usable before JIG_USB_IDS has been confirmed, but it warns through the caller so
the constant gets fixed rather than quietly relied on.

`JIG_USB_IDS` comes from the arduino-pico board variant and is not confirmed on hardware
(`--list-ports` shows the real ids). A wrong id falls back to the scan silently. The
`TRANSMITTER_USB_IDS` exclusion matters more: without it a first-ttyACM scan opens the
radio and times out waiting for `TIME`.

#### `find_port_by_serial`

Re-find a port after a replug.

Matching on the device path does not survive a replug: /dev/ttyACM0 can come back as
ttyACM1 if anything else enumerated in between. The RP2040's serial number is its flash
id, so it is stable across the unplug the run card asks for.

#### `wait_stopped`

Block until button B is pressed. Returns (samples, dropped).

Must be called with the cable plugged in: the stop summary goes to the wire
unbuffered, so a press while unplugged loses the counts for good.

#### `probe_clock`

Fit the host-to-jig clock offset from a burst of TIME round trips.

The jig counts microseconds since its own boot, so cross-correlating command edges
against measured acceleration only ever recovers `clock_offset + transport_delay`.
Measuring the offset separately is what makes the fitted transport delay a real number
rather than a real number plus however far apart the two clocks happen to be.

Only the fastest decile is kept. USB Full Speed polls in 1 ms frames, so most of the
spread is host scheduling; the fastest round trips are the ones whose transport was
most nearly symmetric, which is the assumption the midpoint pairing rests on.

#### `skew_ppm`

Crystal drift across the run, in parts per million. The RP2040 crystal runs around 30 ppm,
about 0.9 ms over a 30 s run: small, but the same order as the residual the run card gates
on, so worth removing.

Returns 0.0 when the jig span is not positive. The jig counts from its own boot, so a
post-probe at or below the pre-probe is a board that restarted, and dividing by that span
is arithmetic across two epochs. LOG-119 did this and produced -1048410 ppm, which reads
as a 105% clock error and would shift a 218 s run by 228 s if anything applied it.
`ClockFit.from_probes` already guarded against it. `skew_ppm` did not, so the number
reached the sidecar, where a reader taking `skew_ppm` directly would trust it.

### `calibration/jig/velocity_jig.py`

Velocity jig log reading, calibration, and dead-reckoned ground truth.

The jig is an RP2040 datalogger that rides on the robot and records a wheel encoder plus an
ISM330DHCX IMU at 1 kHz (`firmware/velocity_jig`). It writes plain text with a comment header:

    # auto-battlebot velocity jig
    # columns: t_us,count,gx,gy,gz,ax,ay,az  (raw int16 imu)
    # encoder: quadrature x4 (constant column if unplugged)
    # sample_rate_hz=1000 gyro_dps_per_lsb=0.070 accel_g_per_lsb=0.000244

Everything here turns those raw counts into the ground truth the plant fit consumes: yaw rate
about the measured gravity axis, forward speed from encoder arc length with the encoder wheel's
lever arm removed, and a dead-reckoned pose. It also carries the two things a log alone cannot
tell you: which host clock a sample belongs to (`ClockFit`), and which run it came from
(`load_session_dir`, reading the sidecar TOML that `velocity_jig_drive.py` writes next to
every downloaded log).

Three things get flagged rather than silently absorbed, because stage 2 shipped parameters
from silently degraded ground truth:

- Saturated gyro channels (`|raw| > 32000`), per axis, per run. This one disqualifies a run:
  a railed yaw axis makes the heading integral wrong from that sample on.
- Clipped accelerometer channels, past `ACCEL_SATURATION_G`. Reported, never gated. Nothing
  in the plant model reads acceleration, so a clip marks an impact worth looking at rather
  than data the fit would be wrong to use.
- Gyro bias drift between the pre-run and post-run still holds.

See `docs/experiments/control/kalman_filter/kalman_filter_plan.md` part 1.2 and the companion runbook.

#### `RAW_SATURATION`

int16 full scale is 32767, and `RAW_SATURATION` is 32000. Anything at or past it is either
clipped or one LSB away from it, and a clipped channel biases every fit downstream of it.

#### `ROLES`

Whether a run trains the fit or validates it. Declared per waveform rather than derived,
so a holdout stays a holdout even if someone later reuses the same excitation for fitting.

#### `saturation`

Per-axis saturation report, gyro first then accel.

`keep` restricts the report to the samples that describe the plant. Handling the
robot at an operator pause clips the accel for a sample or two, which says nothing
about the run, so callers that know where the pauses were mask them out.

The gyro keeps the raw rail test. Its ranges are chosen to sit just above the spin
rates being measured, so "near full scale" is the whole question there. The accel
runs wide of anything commanded, so what matters is the physical number.

#### `read_jig_log`

`time_us_64()` wraps after 584,000 years, so a decreasing timestamp means a corrupt row,
not a rollover. The reader keeps the monotone prefix and counts the rest as malformed.

`JigLog` fields: `t_boot_s` is seconds since RP2040 boot, `count_x4` is the quadrature count (x4, int64), and `gyro_raw_int16` / `accel_raw_int16` are `(N, 3)` int16 values as written.

#### `_bridge_blips`

Fill False runs no longer than `max_run`, leaving longer ones alone.

A held segment is one contiguous run, so without this a single sample over threshold
splits a 5 s hold into two 2.5 s halves and neither clears `min_duration`. That is not a
hypothetical: LOG-143's pre-hold peaked at 2.01 dps against the 2.0 bar with the encoder
dead still and gravity flat to 0.5%, and it was thrown out, while LOG-115 peaked at 1.90
and passed. The two are the same physical state.

Real motion lasts far longer than this window. A blip is a footfall on the floor, a bump
of the bench, or the sensor's own tail.

#### `find_still_segments`

Find stationary stretches at least `min_duration` long.

Stillness is tested three ways because each alone has a blind spot. The gyro catches
rotation. The accelerometer magnitude near 1 g catches acceleration but says nothing about
constant speed, so a robot cruising in a straight line looks perfectly still to it. The
encoder closes that hole, which matters because the deadzone staircases spend their first
several seconds barely moving and would otherwise get absorbed into the opening still hold.

The thresholds are loose on purpose. A hold that is not actually still shows up later as bias
disagreement between the pre and post holds.

#### `still_stats`

Gyro bias and the gravity ("up") unit vector over one still hold.

The yaw axis comes from gravity rather than from a note about how the jig is bolted on. The
accelerometer measures specific force, so at rest it reads +1 g on whichever axis points up,
and the ISM330DHCX puts gyro and accel on one right-handed triad. That makes positive yaw
counter-clockwise viewed from above with no further assumption, and it survives a tilted
remount that no single gyro axis would.

`StillStats` fields: `bias_radps` is the `(3,)` gyro bias in the IMU frame, `up_unit` the `(3,)` unit vector along measured gravity, and `gyro_noise_radps` the `(3,)` standard deviation.

#### `JigCalibration`

Bench calibration from runbook block 0, one set per hardware configuration.

`meters_per_count` (E4) and `gyro_scale` (E3) set the units of every fitted parameter.
`r_enc_perp` (E5) is the encoder wheel's lever arm, removed from every speed measurement,
which is what makes combined linear-plus-angular runs usable instead of discarded. `r_imu` is
kept for the report and for E5's own sanity check against a tape measure.

`from_toml` rejects a non-positive `meters_per_count`: zero would make every measured speed zero. Fit it with runbook E4 and `playground/calibration/fit_encoder_scale.py`.

| Field | Meaning |
| --- | --- |
| `r_enc_perp` | Encoder wheel offset perpendicular to the forward axis, m |
| `r_imu` | IMU offset from the yaw axis, m |
| `encoder_rate_limit` | Rate above which the encoder wheel scrubs, rad/s (E5) |

#### Command coverage

`load_run` counts a grid sample as commanded when it falls between the first command and 2.0 s after the last one. The still holds that bracket every run sit outside the command log, and so does an operator-driven run whose sticks were never logged. The coast for a couple of seconds past the last command is real data, because the runner zeroes and disarms on exit, so that much grace is kept and the rest is dropped.

`problems` rejects a run when the larger of the two hold gaps exceeds `max_hold_gap_s` (default 5.0 s, half of a ten second hold). Detection trims the edges of a still stretch, so a little uncovered time is normal; half of it missing is not.

#### `ClockProbe`

One `TIME` probe burst, as `velocity_jig_drive.py` records it.

The probe sends `TIME` many times, keeps the fastest decile by round trip, and reports
the median host-minus-jig offset (`offset_ms`, milliseconds) with the RMS about that median as `residual_ms`. Only
the fastest probes are kept because USB Full Speed polls in 1 ms frames: most of the
spread is host scheduling, and the fastest round trips are the ones whose transport was
most nearly symmetric.

#### `PauseWindow`

One operator stop, in host seconds.

The robot is picked up, walked back to its mark and set down inside this window, so the
motion there is the operator's rather than the plant's. Every gate and every duration
that describes the run has to skip it: setting the robot down clips the accelerometer,
and counting the wait as commanded time makes a 17 s excitation look like 54 s.

#### `WaveformSpec`

What a run played, in the terms the fit routes on.

This replaces the runbook experiment id. A fit that keys on "E8" needs editing every
time a new excitation is worth trying; a fit that keys on "a step on the linear
channel" does not. `params` is the catalog entry verbatim, which is what lets the
report say that an amplitude was declared but never actually reached.

#### `RunRecord`

The two drive channels exactly as the radio reported them. This is the measurement;
cmd_lin/cmd_ang are these read through a mix, so a mix set wrong can be corrected from
these without re-recording.

Units and sets: `hold_pre_host_s`, `hold_post_host_s` and `cmd_t_host_s` are host seconds. `encoder` is an `EncoderState` (`attached`, `detached` or `either`). `gate_note` is why the capture-time gates rejected the run, `None` when they passed.

`command_source` is a `CommandSource`. `measured` is the radio's own mixer output read back over the trainer link, which is what the robot actually received. `commanded` is what the CLI asked for. They differ when the driver's stick was not centered, and for a hand-driven run only the measured stream exists.

#### `_pause_windows`

Pause windows from the sidecar, or reconstructed from the command stream.

Sidecars written before the host windows were recorded carry only the program time and
the seconds held, neither of which places the pause on the log. The command gaps do, and
they are the same measurement the runner would have written.

#### `load_run_record`

Read one sidecar TOML and its command CSV.

`commands` picks which stream the fit sees. "measured" is the radio's own report, which
is what the robot actually received and the right default. "requested" is what the tool
asked for, and is the escape hatch for a run whose readback was mis-decoded: a wrong mix
or channel sign corrupts the measured stream while leaving the requested one exact.

#### `read_command_log`

Read a `LOG-N.cmd.csv` written by the drive CLI. Columns are named in a `# columns:` comment so the file can grow without breaking older readers. `t_host_s` shares its origin with the clock probes' `at_host_ms` (both `time.monotonic()` on the host), which is the identity `ClockFit` rests on.

#### `GroundTruth`

Dead-reckoned truth for one run, on the jig's 1 kHz sample grid.

Heading drift dominates position drift, so this is trustworthy over the 500 ms windows the
plant fit uses and not over a whole 30 s run. Validation is windowed for exactly that reason.

`t_s` is host seconds when a `ClockFit` was supplied, else jig seconds. `v_body_mps` is the body forward speed, `v_onset_mps` the same with only light smoothing for onset timing, `yaw_rate_radps` the yaw rate, and `theta_rad` the unwrapped heading.

#### `solve_ground_truth`

Unicycle dead reckoning from encoder arc length and gyro yaw.

    w(t)     = (gyro(t) . up - bias . up) * gyro_scale
    theta(t) = theta_0 + integral w dt              (trapezoidal, at the log rate)
    ds(t)    = (count(t) - count(t-1)) * meters_per_count
    v(t)     = ds/dt - w(t) * r_enc_perp            (encoder lever arm removed)
    p(t)     = p_0 + integral [v cos(theta), v sin(theta)] dt

Bias is taken from the pre-run still hold and, when a post-run hold is given, ramped linearly
to it across the run. A linear ramp is the right shape for thermal drift and it removes the
part of the heading error that a single-point bias leaves behind.

Encoder speed is smoothed over `smooth_s` before use. Differentiating a quantized count at
1 kHz otherwise produces a square wave whose amplitude is meters_per_count / dt, which is
much larger than the speeds being fit.

#### `RunQuality`

Quality is set per run in the sidecar. The post hold exists to bracket gyro bias drift, and
without it `solve_ground_truth` holds the pre-hold bias constant instead of ramping it.
Measured on this board that is worth very little: rate random walk is 1.862e-4
deg/s/sqrt(s) (E2, LOG-97), so 60 s of run moves the bias about 1.4e-3 deg/s and leaves
roughly 0.04 deg of heading error. Even the worst drift seen across 33 two-hold runs,
0.0155 deg/s, is only about 0.5 deg over 60 s. Waive it when a run is worth more than
that, and never on a run whose still holds failed because the robot was being handled.

#### `problems`

Gyro only, and zero tolerance. A railed yaw axis is unrecoverable: the true rate is
unknown, so the heading integral past that sample is wrong and stays wrong.

The accelerometer used to gate here too and no longer does. It cost more runs than
it saved, and it discarded them for the procedure rather than for the data: on
2026-08-22 the clip that discarded LOG-108 landed 1.29 s before the first command
(unplugging the USB cable) and the one that discarded LOG-110 landed 18.8 s after
the last (picking the robot up to plug it back in). See `accel_clip_samples`.

#### `notices`

Things worth looking at that do not disqualify the run.

Separate from `problems` on purpose. A reason to open the plot is not a reason to
throw the recording away, and folding the two together is what made an unplugged
cable look identical to a wall strike.

#### `Run`

One run resampled onto a uniform grid, with commands aligned to it.

Fits should not depend on the log rate, and the command log is asynchronous, so both get put
on one grid here. Everything downstream (per-phase fits, window prediction, residuals) reads
these arrays and nothing else.

`t` is the uniform grid in host seconds. `v_onset` is lightly smoothed and used for onset timing only. `command_known` is a bool mask of samples where the command is known (see `load_run`). `phase_label` is the per-sample phase label from the protocol segments.

#### `_hold_uncovered_s`

Seconds of a commanded still hold that the log does not show as still.

Overlap, not edge alignment. The jig starts recording when the operator presses A, which
is before the CLI starts its hold countdown, so the detected still segment routinely
begins earlier than the commanded one and ends later. An edge comparison reads that
normal offset as a fault.

What actually matters is whether the robot was motionless for the window the fit will
take its gyro bias from. A commanded hold that the log shows as moving means the jig was
not recording when the CLI thought it was, and every downstream alignment is off.

#### `load_run`

Read a run's log, solve ground truth, and resample it with its commands onto one grid.

`fit_hz` of 200 keeps five samples per 10 ms and is far above the 2.7 Hz corner of the plant,
while cutting the 1 kHz log down to something a windowed simulation fit can iterate over.

Every stretch the operator spent handling the robot (a pause) is dropped before anything is measured. A pause is not a still hold, not commanded time, and not a place where a clipped accelerometer says anything about the plant.

### `calibration/sysid/coverage.py`

The live coverage session: telemetry and tags in, labeled motions and plant coverage out.

``CoverageSession`` is pure and deterministic: a source feeds it decoded messages in log order
(``feed_telemetry``, ``feed_tags``, ``feed_field``), ``advance`` processes everything older than
``LAG_S``, and ``snapshot`` returns the page's JSON. It does no IO; the refit runs through a
``RefitEngine`` (``worker.py``), inline for tests and replay or in a process when live.

Pipeline per 10 ms grid sample, once the tags around it have arrived:

1. Telemetry is put on the host clock (``telemetry.LiveClock``). Commands are held to the grid;
   the gyro is interpolated. The clock map rests on the fastest deliveries, so the minimum
   radio latency stays in it and every telemetry stamp reads that much late. The fit's delay
   absorbs it for the commands; for the gyro it is a millisecond or two against the tags.
   The Jetson's ring poller stamps every row of one poll with that poll's arrival and marks all
   but a caught-up poll's newest row ``backfill``. Only each poll's newest row reaches the
   clock, so its fit stays at radio latency instead of the 100 ms a poll's oldest row waits.
2. ``maneuvers.ManeuverSegmenter`` labels the sticks.
3. Every ``model.WINDOW_S`` the window closes: gates, three sub-windows from the tag poses,
   Fisher information at the current estimate, and a status.
4. A refit runs every ``REFIT_S`` of driving when new windows closed.

Gates, first match wins:

- jetson: ``espnow_active`` or ``autonomy_enabled`` anywhere in the window. The sysid profile
  never commands, so this is a banner as well as a gate.
- link: telemetry missing for over ``GAP_S``, counted from ``telemetry_seq`` (frames lost) or
  from ``timestamp_ms`` (frames late). A frame or two lost is held over and costs nothing.
- tags: accepted tag fixes cover under ``TAG_COVERAGE_MIN`` of the camera frames.
- flip: the robot is upside down, or the running median of ``PITCH_MEDIAN_N`` tag poses is
  pitched more than ``PITCH_LIMIT_DEG`` off the rest pose.
- wall: the axle center within ``ROBOT_LENGTH_M`` of a rail (``sysid_windows.near_rail``), or,
  once the fit has settled, a tag residual over ``CONTACT_SIGMA`` (marked by the refit).
- disarmed: ``armed`` false.
- idle: no motor command over ``IDLE_CMD`` and no tag speed.

Every fifth usable window is held out: it scores the closed-loop prediction and never enters
the fit.

#### `PITCH_MEDIAN_N`

The pitch gate tests a running median of 7 tag poses. Single-frame tag pitch spikes reach 8 deg
on a flat spin (frame-to-frame sd 1.7 deg), and a real nose lift lasts far longer than 7 frames.

### `calibration/sysid/maneuvers.py`

What the driver did with the sticks: holds, edges, creeps, arcs, spins.

Two consumers:

- ``ManeuverSegmenter`` labels the stick stream as it arrives for the live coverage monitor. It
  works on the uniform 10 ms grid the monitor builds, with sticks as fractions, throttle forward
  positive and steer left positive.
- ``menu_from_grid`` is the offline session summary that
  ``playground/calibration/check_sysid_recordings.py`` prints against its ``MENU_TARGETS``. It
  moved here unchanged when the live monitor became its second caller.

Live definitions, ported from the prototype page:

- Plateau: a maximal run where a channel stays within ``PLATEAU_TOL`` of every other sample in
  it, the jitter of a held thumb.
- Hold: a plateau of ``HOLD_S`` or longer away from zero. A throttle hold with the steer mean
  under ``ARC_STEER`` is straight, with more it is an arc; a steer hold with the throttle mean
  under ``ARC_STEER`` is a spin.
- Edge: the move between two long plateaus (rest counts at any length) that differ by at least
  ``EDGE_MIN``. Clean when it took ``SNAP_S`` or less. Only clean edges count toward delay and
  lag; a slower one is a slew.
- Creep: a monotonic climb out of zero past ``CREEP_PAST`` that took ``CREEP_MIN_S`` or longer.

#### `BINS`

The coverage map bins are stick fractions, finer near zero where the deadzones are.

#### `EDGE_ROWS`

The transition table has rows for the yaw loop. The loop rows are steer edges with the flip
switch down, where the firmware yaw loop answers instead of the open-loop drivetrain.

### `calibration/sysid/model.py`

The fit behind the sysid monitor: window residuals, Fisher information, refits, suggestions.

Each 1.5 s fit window is cut into three 0.5 s sub-windows. A sub-window starts from the tag
state estimate at its start (``pose_stream.state_at`` plus the gyro) and runs the plant open loop
on the recorded motor commands for 0.5 s, the horizon the Kalman plan scores. Its residuals are:

- tag position, along and across the camera view, at each tag fix, over the per-fix sigma
  floored at ``POS_FLOOR_M`` for model error,
- tag heading at each fix, likewise floored at ``YAW_FLOOR_RAD``,
- gyro yaw rate every 20 ms over ``SIGMA_GYRO_RADPS``.

Heading has to be in there: with velocity and yaw rate alone the two drift terms never separated
in the prototype.

The fit input is the motor command (``firmware_mixer.motor_to_plant`` of ``left_cmd``/
``right_cmd``), so the same 16 drivetrain parameters come out with the yaw loop on or off and
the loop's feedback stays out of the Fisher matrix. Delay is one of the 16: commands are shifted
by a fractional, linearly interpolated delay, which keeps the residual smooth in it.

Predictions for the page (holdout, ghosts, suggestions) instead run the sticks through
``firmware_mixer.closed_loop_rollout``, so they score the plant the Jetson would command.

Uncertainty is the Cramer-Rao bound ``s^2 (J^T J + prior)^-1`` with ``s^2`` the fit's residual
variance per row when that exceeds 1, so a model that cannot reach the tag noise reports wider
bars instead of tighter ones.

#### `ParamSpec`

Each fit parameter has a prior (the `PlantParams` defaults, which are the stage 2 values), a
prior sigma, a ring target and a finite-difference step. The ring targets are:

- 4 ms for delay
- 6 ms for each lag
- 3% of the prior for each gain
- 0.008 for each deadzone
- 0.03 for each coupling term
- 0.12 rad/s for each drift term

#### `NUISANCE_SD`

Prior sigmas on each sub-window's start state come from the local tag estimate: x, y, theta, v.
The start state is a nuisance: each sub-window refines it, and the information it could fake
is projected out of the Jacobian.

#### `jacobian`

(r, J) with J shaped (S, R + NUISANCE, NP); columns outside ``free`` are zero.

Forward differences at the refined start states, stepped away from a bound, then projected
so the start states absorb what they can: data rows become ``J - jn A^-1 jn^T J`` and the
prior rows ``-A^-1 jn^T J / sd``. ``J^T J`` of the result is the Schur complement, the
information left once the start states are marginalized.

#### `expected_subwindows`

Simulate a motion from rest and cut it into the sub-windows a session would record.

Returns the sub-windows (measurements equal to the prediction, so their residual is zero
and only the Jacobian matters) and how far the robot travels.

### `calibration/sysid/pose_stream.py`

Causal robot pose from the AprilTag stream, for the live sysid monitor.

The offline path (``tag_pose_smoother.smooth_session``) runs a two-pass RTS smoother over the
whole session. The live monitor only needs two things from the tags, and neither needs the
future past a short lag:

- one body pose per detection, for the fit residual and the gates. Each frame goes through the
  offline ``build_candidates`` and ``select_ippe``; ``select_ippe`` looks ahead up to its
  ``neighbour_window_s`` on close calls, so a detection is final once it is that much older
  than the newest frame.
- the state each fit window starts from. ``state_at`` fits a weighted quadratic in time to the
  poses within ``STATE_HALF_SPAN_S`` either side of the start, which is a fixed-lag smoother of
  that one instant. Velocity is its slope projected on the heading; yaw rate comes from the
  gyro, which the caller passes in.

Frames follow ``tag_pose_smoother``: the output frame is the recorded field frame turned so z
points up, the body frame is FLU at the axle midpoint, and yaw is counter-clockwise.

### `calibration/sysid/server.py`

The sysid monitor's page: one static HTML file and a websocket of JSON snapshots.

Served by the Python process itself, on the laptop, because the page shows that process's
state, not the app's. ``GET /`` returns ``static/index.html``; ``/ws`` sends a ``hello`` with
the parameter list, then every snapshot ``publish`` hands it.

### `calibration/sysid/telemetry.py`

Robot telemetry for the sysid monitor: the frame schema, the plant inputs, the robot clock.

``/robot/espnow_telemetry`` is the source. The column list is read from the JSON schema the
recorder writes (``mcap_write.ESPNOW_TELEMETRY_JSON_SCHEMA``), so a field added there shows up
here with no edit and a renamed one fails loudly where it is used instead of reading as NaN.
Integers load as nullable ``Int64`` (``telemetry_seq`` is <NA> in recordings older than it),
numbers as float with NaN for null, booleans as bool.

Without ESP-NOW telemetry the source is ``/robot/esp32_diagnostics``, the robot's telemetry
ring polled over WiFi (older recordings: its event stream), with the same drive fields.
``from_diagnostics`` renames its ``seq`` to ``telemetry_seq`` so drops count the same way; the
columns it lacks (``espnow_active``, ``autonomy_enabled``) read as False. Its ``backfill`` rows
waited in the ring, so their ``host_receive_ns`` is late and the live clock skips them.

Plant inputs:

- fit input: ``left_cmd``/``right_cmd``, what the ESCs got after the deadband and the yaw loop,
  through ``firmware_mixer.motor_to_plant``.
- prediction input and stick display: ``a_percent``/``b_percent``, the arbitrated sticks before
  the mixer. ``throttle = -a / 100`` (forward positive) and ``steer = -b / 100`` (left positive),
  the plant's sign convention.

### `calibration/sysid/worker.py`

Refits for the sysid monitor, off the session's thread.

``RefitState`` keeps every usable window's sub-windows and turns a ``RefitJob`` into a
``RefitReply``: a warm-started fit, the Fisher matrix over every fit window, the contact gate,
the holdout score, every third time the model ladder, and the ranked suggestions.

Two engines run it. ``InlineEngine`` calls it on ``submit``, which keeps tests and replays
deterministic. ``ProcessEngine`` runs it in a spawned process so a live session keeps labeling
while a refit takes a few seconds.

`MAX_FIT_WINDOWS = 300`: the fit thins to this many windows. The Fisher matrix still uses all of them.

### `control/firmware_mixer.py`

#### Module conventions

`mix_motor_outputs` (`firmware/mr_stabs_mk2/src/main.cpp`) turns the arbitrated sticks into
motor commands. With the flip switch down it runs `yaw_control::YawController` on every BNO055
sample and its output replaces the steer stick; otherwise the steer stick passes through a 1%
deadband. `mujoco_sim/firmware.py` mirrors both line for line as scalar Python stepping one
robot. This module is the same logic over a batch dimension, so the sysid fit can roll out
hundreds of windows at once, plus `closed_loop_rollout`, which runs `plant.integrate_step`
behind the mixer with the heading and gyro read back from the simulated state.

Conventions follow the firmware: percents in [-100, 100], `a_percent` positive drives
tail-first (stick back), a positive steer output speeds up the left wheel and turns the robot
clockwise, headings in degrees growing clockwise, rates in degrees per second
clockwise-positive. The plant is counter-clockwise-positive in radians; `heading_from_theta`
and `gyro_from_w` convert.

Upside-down driving is not mirrored: the sysid gates drop inverted windows before they get here.

#### `motor_to_plant`

The mixer writes left = -a + steer and right = -a - steer. Forward is positive linear, and a
counter-clockwise turn (right wheel faster) is positive angular, as in `plant.py`.

#### `closed_loop_rollout`

`a_percent`, `b_percent` and `auto_steer` are (B, N) on a `dt` grid starting at the window
start. `history` is the plant command (linear, angular) on the same grid for the samples before
the start, (B, M), most recent last, so the transport delay reads real commands for its first
`delay_s`. Returns per-sample x, y, theta, v, w and the motor commands.

The command in effect `delay_s` before each step is sampled linearly between grid samples and
held for the step, the same sampling as `plant.delayed_commands` and `plant.simulate`. It is
zero before the history starts.

#### `from_telemetry`

Seed the loop from the telemetry sample at a window start.

The setpoint is carried over as an offset from the recorded heading, so it lands in the
simulated heading frame whatever the BNO055's absolute heading is. The integral is what
is left of the recorded output once the feedforward and proportional terms come off;
the firmware does not report it, but it reports everything else in that sum.

#### `BatchedMixer`

`mix_motor_outputs` plus `YawController`, stepped once per BNO055 sample.

The firmware runs the yaw loop only when a fresh IMU sample lands (every 10 ms), and the
steer output it computes holds until the next one. Stepping the whole mixer at that period
keeps the loop exact and costs at most one control loop of stick latency in passthrough.

### `control/plant.py`

#### `delayed_commands`

Shift a uniformly sampled command timeline back by `delay_s`.

The shift is fractional and interpolated linearly between samples. At the fit grid rate the
command is already a staircase sampled finely, so interpolating between two samples of it is
a sub-grid-step approximation to the true edge time, which is what makes the delay profile
smooth enough to search on a 2 ms grid.

#### `subsample`

Thin the window set to at most `limit` windows, evenly across the run.

Windows overlap heavily at a 50 ms stride, so thinning costs far less information than
the count suggests, and it is the knob that keeps a delay profile search over 60 grid
points to minutes instead of an hour.

### `mujoco_sim/actuator.py`

#### `PlantParams`

Everything the fit can move. Mass properties are fixed from CAD and are not here. Units are not in
the field names because fitted `fit.json` files store them by name.

| Field | Meaning |
| --- | --- |
| `delay_s` | Firmware command to motion. The radio part is measured separately. |
| `deadzone_left`, `deadzone_right` | Fraction of full command. |
| `lr_gain_ratio` | Left gain over right gain. |
| `joint_frictionloss` | N m at the wheel. |
| `joint_damping` | N m s / rad at the wheel. |

### `mujoco_sim/closed_loop.py`

`ClosedLoopSim` drives Mr Stabs Mk2 on CPU MuJoCo from body-axis stick commands in an arena. The
command chain is the robot's:

- Linear and angular stick fractions become the CRSF percents the firmware reads. Channel 0 carries
  linear and the firmware negates it (`a = -linear`). The transmitter reverses the angular channel
  (`b = -angular`).
- The firmware mixer and its heading hold turn those into per-motor percents, and the drivetrain
  delay holds them.
- The command-tape shaping (deadzone, throttle curve, left/right gain, pack voltage) turns them into
  wheel volts.
- Motor polarity is +1. The sim has no ESC wiring to undo, so a positive per-motor command drives
  forward.

The flip switch decides when the firmware treats the robot as inverted and negates the throttle
before the mixer (`FlipSwitch`). In the power-on DOWN position it follows the IMU's upside-down
reading with its hysteresis, so a flipped robot keeps driving wedge-first and the heading hold
restarts on the flip. `firmware_is_upside_down` is the firmware's `is_upside_down`.

Arena walls and wall blocks are static geoms, and hazards that move (the house bot) are mocap
cylinders, so wall contact is rigid-body contact and not a position clamp.

#### `MovingBlock`

A robot ours can hit, placed every tick by `ClosedLoopSim.set_moving_blocks`. It is a cylinder of
`radius`, or with `box` set, a box of full size `BoxSize(length_m, width_m, height_m)` with its
length along the robot's heading. That is the shape the RL environment gives the opponent
(`warp_nav.OPP_SIZE_M`).

### `mujoco_sim/firmware.py`

Mirrors `mix_motor_outputs` (`firmware/mr_stabs_mk2/src/main.cpp`) and
`yaw_control::YawController` (`firmware/mr_stabs_mk2/lib/yaw_control/`) line for line.
`test_firmware_yaw_mirror_matches_native_golden_fixture` replays a fixture generated from the native
`yaw_control.cpp`. The fit never uses this module: it replays the logged per-motor commands, which
already come out of the mixer.

Two firmware generations are mirrored, because `validate.py` replays recordings made before the
yaw-rate loop:

- `legacy=True`: heading hold with `PidV1` every control loop, quirks included. The integral
  accumulates the raw error and multiplies by dt only on output, and the tolerance check returns
  before the derivative term updates its previous error. The stick passes through while turning
  and for a 0.25 s coast after.
- `legacy=False` (default): the current firmware. In auto steer, `YawController` runs once per
  BNO055 sample (100 Hz): the stick commands a yaw rate, heading hold commands the rate back to the
  held heading, and an inner loop on the gyro sets the differential. Under 5% throttle with the
  stick centered it idles. It runs upside down too. Outside auto steer the stick passes through
  with a 1% deadband.

The BNO055 heading (`orientation_x`) is degrees in [0, 360) and grows clockwise seen from above.
`heading_from_yaw` converts the sim's counterclockwise yaw into it, and rates are clockwise-positive
to match. This sign assumption is pinned by a recording (plan validation check 5). The firmware
checks its own gyro sign at runtime.

`UPDOWN_THRESHOLD_UP_Z` mirrors `RIGHT_SIDE_UP_THRESHOLD` and `UPSIDE_DOWN_THRESHOLD` in
`updown_sensor.h`: the sensed gravity z must pass -1.0 m/s^2 to read right side up and +1.0 m/s^2 to
read upside down, and holds in between. As a chassis up-axis z component that is a +-0.102 band.
`test_updown_thresholds_match_firmware_header` reads the header.

#### `FirmwareMixer.resume`

Seeds the integral with the logged `pid_output` minus its feedforward and proportional terms. Seeding
it with the whole output double-counts the feedforward in a turn. The measurement is in
`docs/experiments/sysid/2026-10-15_mujoco_open_loop_heading_hold.md`.

### `mujoco_sim/fit.py`

`WindowErrors` holds position and heading errors shaped (N, H) for N windows and H horizons, NaN
where no measurement landed near a horizon. `pitch_rms_rad` is (N,) over the whole window and NaN
for flat windows. `position_sigma_m` and `heading_sigma_rad` are (N, H): the sigma each error was
scored against, the measurement's own or the window's.

A `Simulator` takes a window batch and one parameter set per world (`len == len(batch)`) and
returns the recorded chassis qpos (N, S, 7) plus the recording interval.

`run_stage` scores the starting point first, so a stage never hands back parameters worse than the
ones it got (`test_stage_never_returns_worse_than_its_start`).

### `mujoco_sim/greybox.py`

Fits the grey-box plant (`control/plant.py`) on the same windows as MuJoCo. This is plan validation
check 1: the MuJoCo model has to beat or match it on held-out sessions. Both models are scored by
the same `fit.window_errors` against the same raw measurements, so the comparison is like for like.
The grey-box plant takes linear and angular commands. They come from the per-motor tape through
`motor_to_plant`, the inverse of the mixer (`u_lin = (l + r) / 2`, `u_ang = (r - l) / 2`).
`plant.simulate`'s `traj[k]` is the state after step k, so `predict` prepends the start and sample
k sits at `k * PLANT_DT`.

### `mujoco_sim/mass_properties.py`

Loads `simulation/assets/robots/mr_stabs_mk2/mass_properties.toml` into frozen dataclasses.

#### `Body`

| Field | Meaning |
| --- | --- |
| `com` | (3,) center of mass, m |
| `fullinertia` | (6,) Ixx Iyy Izz Ixy Ixz Iyz about the COM, kg m^2 |

#### `TagMount`

| Field | Meaning |
| --- | --- |
| `upside_down` | The tag faces the floor when the robot is upright |
| `translation` | (3,) tag center in the body frame |
| `rotation` | (3, 3) `R_body_tag` |

#### `Weapon`

| Field | Meaning |
| --- | --- |
| `center` | (3,) point on the spin axis at the disk's mid-plane, body frame |
| `radius` | Largest disk radius about the axis |
| `half_width` | Half the disk thickness along the axis |

#### `MassProperties`

| Field | Meaning |
| --- | --- |
| `axle_x` | Axle of the driven wheels, body x |
| `follower_axle_x` | Axle of the belt-coupled wheels, or `None` when there are none. `total_mass` counts each wheel twice when it is set |

### `mujoco_sim/mjcf.py`

`GEAR_SOLREF` is the gear-coupling stiffness: a time constant of two timesteps, the minimum MuJoCo
accepts, at the 1 ms `FIT_TIMESTEP_S`. Without a collision hull the skid sits at the bottom-front
of the chassis COM box (`SKID_FALLBACK_X_M`, `CHASSIS_BOX_BOTTOM_Z_M`).

### `mujoco_sim/onshape_export.py`

URDF `rpy` is extrinsic roll-pitch-yaw about fixed x, y, z, which is scipy's lowercase `"xyz"`.
`LinkInertial.com_link` is (3,) and `inertia_about_com_link` is (3, 3), both in the link frame.
`onshape_inertia_to_tensor` reads Onshape's "Mass moments of inertia" panel (g mm^2). The panel
shows the tensor elements themselves, products already carrying the minus sign, so subtracting the
wheels from it reproduces the chassis product of inertia the URDF parts give, in sign and size.

#### `TagFrame`

A tag's marker frame in the body frame.

`rotation` and `translation` form T_body_tag, mapping points in the OpenCV marker frame
(origin at the tag center, x right and y up in the printed image, z out of the face; the
object points solvePnP IPPE_SQUARE expects) into the body frame.

`black_square_side_m` is the length OpenCV calls the marker length. `tilt_from_body_z_deg` is the
angle between the tag normal and the body's +z (or -z when underneath). `decodes_mirrored` is set
when the CAD face only decodes mirrored.

The CAD face is rasterized with the lightest faces painted first so the black pattern lands on top
of any overlapping plate.

### `mujoco_sim/rollout.py`

#### `CollisionSet`

Loaded from `collision.toml`.

| Field | Meaning |
| --- | --- |
| `pieces` | Convex OBJ pieces in the body frame |
| `skid_point` | Lowest nose point, body frame; `None` on four wheels |
| `rest_pitch_rad` | Nose-down pitch with both wheels and the skid on the floor |

#### `rotor_diaginertia`

Radial inertias are 0.6 of the axial. A thin disc's 0.5 compiles (MuJoCo 3.14, checked 2026-11-19), so the triangle-inequality limit is not why. The radial value reaches chassis yaw: at 0.5, `test_yaw_loop_stops_the_full_reverse_spin[1.02]` in `tests/python/test_mujoco_sim.py` spins to 88° of reverse yaw against its 60° bound, so the fitted plants and the yaw-loop gains assume 0.6.

#### `NCONMAX` and `nconmax_for`

Contacts and constraint rows per world. The hull pieces, skid and both treads rarely give more than
8 contacts; the 24-contact floor covers a robot on its side. The broadphase pairs the floor plane
with every robot geom, so a model with more pieces (Mrs Buff Mk3: 30 pieces and four treads) needs
room for that many candidate pairs: `nconmax_for` returns at least `2 * ngeom`. MuJoCo Warp drops
contacts on overflow, so an undersized buffer sinks the robot into the floor. `njmax_for` allows
`ROWS_PER_CONTACT` = 6 rows per contact (condim 6 on the treads), plus equalities and joint
friction.

#### `ParamArrays`

Shapes, with W worlds: `geom_friction` (W, ngeom, 3), `geom_solref` (W, ngeom, 2), `geom_solimp`
(W, ngeom, 5), `dof_damping` and `dof_frictionloss` (W, nv), `body_inertia` (W, nbody, 3),
`actuator_gainprm` and `actuator_biasprm` (W, nu, 10), `actuator_forcerange` (W, nu, 2). `coast`
is (W,), 1.0 where a zero command freewheels. `__post_init__` rejects any other rank or trailing
size.

#### `InitialState`

Per-world planar pose and velocity in the field frame. `pitch_nose_down_rad` is positive nose
down. `initial_qpos_qvel` composes the orientation as q = q_yaw * q_pitch (intrinsic z then y).
The free-joint angular velocity is in the body frame, so for a pitched body a pure field yaw rate
has components on body x and z.

#### `WarpRollout`

Tapes are (W, T, 2) volts and a (W, T, 2) zero-command mask; `set_tapes` rejects other shapes.
`run` resets every world to its initial state, replays the captured step graph T times and returns
(W, S, 7) chassis qpos sampled every `record_every` steps (S = ceil(T / record_every)).

### `mujoco_sim/session.py`

Load smooth_tag_poses.py session bundles into rollout windows for the MuJoCo fit.

A bundle is the directory playground/calibration/smooth_tag_poses.py writes per recording:
smoothed.csv (uniform-grid smoothed pose), measurements.csv (raw tag detections), commands.csv
(ESP32 events on the app clock), windows.csv (gated windows with initial states) and
session.json (noise floor, clock fit, ...).

The fit's loss reads the raw measurements, never the smoothed poses; the smoothed state only
supplies each window's initial conditions.

`TAPE_PAD_S` is the tape history kept before each window start so the delay shift never runs off
the front; it must exceed the `delay_s` upper bound in `fit.PARAM_SPECS`.

#### `WindowBatch`

`session_index` indexes `sessions`. `kind` is one of `WINDOW_KINDS` ("flat", "nose_lift").
`u_raw` is (N, pad + steps, 2), the per-motor command in [-1, 1], forward positive; `vbat` is
(N, pad + steps). `firmware` is the logged signals on the same grid, in `FIRMWARE_COLUMNS` order
(a_percent, b_percent, pid_output, left_cmd, right_cmd in percent, orientation_x in degrees), for
the closed-loop check. `polarity` is +1 per window if a positive logged motor command drives
forward, -1 otherwise (see `motor_polarity`); `__post_init__` rejects other values.

`meas_sigma_xy` and `meas_sigma_yaw` hold one sigma per measurement per window. Tag fixes carry
the session noise floor; a YOLO fix carries its own, larger one. An empty list means every
measurement uses the window's `noise_*`.

`tapes` returns `Tapes(volts, zero)`, each (N, steps, 2), for one parameter set per window or one
shared. It applies delay, deadzone, throttle curve, left/right ratio and vbat.

`build_batch` recomputes each window's initial wheel speeds with `rolling_wheel_speeds`, the
no-slip relation in the MJCF's convention (positive rolls forward), rather than reading them from
windows.csv.

### `mujoco_sim/step_response.py`

`StickTape.linear_frac_fwd` and `angular_frac_ccw` are body-axis stick fractions on a uniform
`STICK_TICK_S` grid from the step's start: forward positive, counterclockwise positive
(`ClosedLoopSim.step`). `fit_armature` clamps both measured and simulated lifts at
`FLIP_LIFT_DEG` before differencing, so a flipped run's unbounded measured lift cannot dominate
the RMS.

### `mujoco_sim/validate.py`

#### `compare_models`

Returns `ModelComparison(errors, qpos, record_dt)`. `errors` is the JSON-safe report; `qpos` and
`record_dt` feed the nose-lift check and stay out of it.

#### `YAW_LOOP_SINCE_NS`

The yaw-rate loop replaced heading hold on 2026-10-05 00:00 UTC.

#### `firmware_closed_loop`

The recordings used by `firmware_closed_loop` predate the 2026-10-05 heading hold rework.

### `perception/tag_pose_smoother.py`

The smoother internals and their tuning are in `docs/experiments/sysid/2026-10-25_sysid_tag_smoother.md`. These are the data contracts and invariants the code relies on.

#### Data layout

- `Candidates` holds N detections, two IPPE solutions each. `t_up_body` is (N, 2, 4, 4); slot 1 repeats slot 0 where `n_solutions` is 1. `tilt_from_rest_rad` is the attitude error from rest. `__post_init__` rejects shape and value mismatches.
- `IppeChoice.solution_index` is 0 or 1 per detection. `close_call_decided_by_neighbours` marks detections whose two tilts were within the margin and were decided by neighbouring frames. `RawPoses.yaw_wrapped` is in [-pi, pi).
- `TagMount.tag_faces_floor_when_upright` is the tag that faces the floor when the robot is upright. The `[tags.N]` table key stays `upside_down` and defaults to the sign of the tag normal's body z. A table gives either `translation_m` and `rotation` (R_body_tag) or a 4x4 `t_body_tag`.
- `up_from_field` returns `UpFrame(t_up_field, flipped)`. `flipped` is true when the camera sits at negative field z, so field z points into the floor.
- `Knots` is shared by every axis of one smoother. `kind[k, a]` is a `KnotKind` stored as int8 and says what axis `a` measures at knot `k`: nothing, its position, or its rate. `variance` is the measurement variance. `t_since_t0_s` is seconds from `t0_ns` and must be non-decreasing.
- `SmootherRun` arrays lead with the batch B: `m_pred` is (B, K, A, 3), `p_pred` is (B, K, A, 3, 3), `outcome` is (B, K) holding `Outcome` codes. `nis` is the joint normalised innovation squared, NaN without a measurement. `innovation` is (B, K, A), normalised per axis, NaN if unused. `Outcome.ACCEPTED_AFTER_LOST_TRACK` is a fix that failed the gate while the filter had lost track, taken with inflated covariance.
- `CrossValidation` keys by group name: `axes_by_group` lists the axes sharing one q, `q_candidates_by_group` the q values tried, `heldout_score_by_group` the mean clipped squared normalised held-out error.
- `ImuYawRate.app_clock_stamp_ns` is a uniform grid on the app clock. `ccw_rate_rad_s` is counter-clockwise positive under the assumed heading sign.
- `SessionSmoothing.detections_df` has one row per detection (the measurements.csv columns plus extras). `smoothed_grid_df` has the smoothed.csv columns plus pitch and roll rates. `noise_calibration_segments` are detection index ranges the noise model came from. `yolo_fixes` has the YOLO position fixes on frames the tag missed (stamp, x, y, yaw, sigmas, rejected), or None.
- `_planar_knots` returns `KnotBuild(knots, source_index)`. `source_index` is the detection index per knot, or `KnotSource.IMU` or `KnotSource.EXTRA`. `_AxisFit.gate_exempt_after_recheck` marks knots the smoothed re-check let through the gate.
- `FilterOptions.angular` is per axis and wraps the innovation to [-pi, pi). `prior_std` is per axis, (position, rate, acceleration).

#### Gating and branches

- An angle measurement is unwrapped onto the branch nearest a reference: the prediction normally, but after a gap longer than `branch_gap_s` since the axis's last fix, the last fixed angle carried forward at its fixed rate. Across a dropout mid-spin the acceleration state is too noisy to pick the branch. At 8 rad/s a 0.3 s gap already leaves a whole turn to chance. `test_angle_branch_after_dropout_mid_spin` pins it.
- Rate knots (the IMU) skip the gate. A wrapped angle fix cannot tell a rate from that rate plus a whole turn per frame, so a jolt across a dropout can leave the rate aliased while every fix still passes. Gating the IMU against that state would reject the one measurement that can pull it back. `test_imu_rate_pulls_an_aliased_yaw_rate_back` pins it.
- `smooth_with_recheck` gives gated position fixes a second look against the smoothed track. The forward gate sees only the past, so a hard reversal or punch that the process noise under-predicts fails it although the fix is good. The smoothed state at a rejected knot already excludes that fix and knows what came next, so a fix within the same chi-square gate of it (smoothed covariance plus measurement noise) is let through on the next pass. An outlier stays far from the smoothed track and stays rejected. It returns the final run and the exempted knots. `test_recheck_readmits_fix_at_reversal` pins it.
- `interpolate_smoothed` is exact for the model: it predicts from the filtered state at the knot before, then applies the RTS gain against the smoothed and predicted states at the knot after. Times outside the knot span clamp to the end knots. `test_interpolate_smoothed_is_exact_at_knots` pins it.

#### Tilt

- Tilt is fed unwrapped and never wraps its innovations (`_TILT_OPTIONS.angular` is all false). YOLO-derived tilt is moved onto the branch of the unwrapped tag fix before each stamp with `unwrap_onto`, so no innovation sees a whole turn.
- The unwrapped state can sit a whole turn off, so the reported `pitch` and `roll` are wrapped on output.

#### YOLO fixes

- `_fuse_yolo` sorts the fixes by stamp. The `KnotSource.EXTRA` knots come out of `_planar_knots` in stamp order, so the outcome at each of them lines up with the same row of the fixes table. `test_yolo_fix_outcomes_align_with_fix_rows` pins it.

#### `FilterOptions`

| Field | Meaning |
| --- | --- |
| `angular` | Per axis: wrap the innovation to [-pi, pi) |
| `prior_std` | Per axis: (position, rate, acceleration) std |
| `reacquire_s` | Accept a gated position fix after this long without one (default 0.3 s) |
| `branch_gap_s` | Gap above which the innovation reference is the state carried forward at its rate instead of the current estimate (default 0.1 s; see `_ForwardState._innovation`). `_with_rate` sets it to infinity so yaw unwraps onto the prediction's branch always |

#### `SmootherOptions`

| Field | Meaning |
| --- | --- |
| `still_yaw_rate` | rad/s below which the tag counts as "not turning" for the IMU rate noise (default 0.15) |
| `spin_rate` | rad/s above which the tag counts as "a spin" for the IMU agreement check (default 3.0) |

### `perception/sysid_windows.py`

Fit windows for the drivetrain sysid: gating and initial state (plan "Data pipeline" 3 and 4).

Input is the smoothed grid from ``tag_pose_smoother.smooth_session``. Three per-sample gates run
first and cut the session into spans:

- **near_rail**: the axle center within ``robot_length_m`` of a box rail, where wall contact is
  possible.
- **esp32_gap**: inside a gap in the ESP32 stream longer than ``esp32_max_gap_s``, or outside the
  stream altogether, since the fit has no command tape there.
- **flip**: the upright/inverted state changes, which the fit cannot start or end across.

Each span shorter than ``window_s`` is dropped; the rest are cut into ``floor(L / window_s)``
equal windows, so every window is between ``window_s`` and ``2 * window_s`` long (1 to 2 s by
default). A window is then dropped if accepted tag detections cover less than
``coverage_min`` of its camera frames. A kept window is ``nose_lift`` when its pitch leaves the
rest pitch by more than ``pitch_limit_deg`` anywhere, otherwise ``flat``.

Initial state per window is the smoothed state at its first sample. Wheel speeds come from the
no-slip relation in the body frame, ``(v -/+ w * track_half_width) / wheel_radius`` for left
and right, with v the forward speed along body x and w the yaw rate about body z. A wheel
spinning to roll the robot forward is positive, which is positive rotation about body +y. On an
inverted robot body z points down, so the body-frame yaw rate is minus the field one: a
counter-clockwise turn seen from above is clockwise about body z, and the left wheel reads
positive while the right reads negative.

#### `WindowOptions`

| Field | Meaning |
| --- | --- |
| `box_half_extent_m` | Half the 1.52 m test box side (default 0.76 m) |


### `recording/esp32_clock.py`

Put ESP32 events on the app clock and measure the delays around them.

The app stamps every ESP32 event with its receive time, which carries WiFi jitter: a few
milliseconds typically and tens of milliseconds when the link retries. The robot's
``timestamp_ms`` has no jitter but runs on its own crystal. ``fit_robot_clock`` maps one onto
the other with a line (offset and drift). Receive delay is never negative, so the line is the
lower envelope of (robot time, receive time): it rests on the fastest deliveries, which a least
squares line would not. On the synthetic session in ``test_tag_pose_smoother.py`` a least-squares
line sits about 4 ms into the jitter and the envelope does not. The minimum transport latency stays
inside the offset; nothing in the data can separate it.

A reboot resets ``timestamp_ms``, so each run of increasing robot time gets its own line.

``cross_correlation_delay`` estimates the lag between two signals on the app clock. It serves the
two checks in the plan: stick channels against the ESP32's ``a_percent``/``b_percent`` (the radio
link delay) and BNO055 yaw rate against tag yaw rate.

#### `_lower_envelope_line`

(intercept, slope) of the line under every point that minimises the summed gap.

Minimising sum(y - a - b x) subject to y >= a + b x is a two-variable linear program whose
optimum is the lower-hull edge spanning mean(x) (Moon, Skelly and Towsley 1999, skew
estimation for network delay measurements).

#### `fit_robot_clock`

The line is fitted to receive time minus robot time, so the slope is the drift and stays near
zero. Fitting host time against robot time directly loses the ppm-level slope to rounding.

#### `radio_link_delay`

How long a stick move takes to show up in the ESP32's ``a_percent``/``b_percent``.

Both signals are held between samples (the stick is logged only on change) and correlated
on their first differences, so the slow drift of a held stick does not dominate the peak.

#### `_smooth`

Gaussian blur that treats NaN as zero. Widens single-sample steps into a peak the
parabolic refinement can fit.

### `recording/sysid_io.py`

#### `field_transform_constancy`

How far the camera pose wanders from its median over the session.

The tripod does not move, so anything above a millimetre or two means a field re-init or a
bumped tripod, and the session needs a look before its poses are used.

## AprilTag pose selection

- Rest pitch: the floor-up vector in the body frame while the robot rests with a tag showing. Upright it leans back by the rest pitch (the robot sits nose down on its wedge tip). Upside down it is body -z, since the resting pitch on the top face is unmeasured.
- IPPE disambiguation: the index of the IPPE solution to trust, from solutions sorted by reprojection error. A solution that puts the camera under the floor is dropped when the other does not. Of the rest, the one whose implied floor up lies closest to the reference wins, because the other IPPE solution is the planar flip, which tilts the robot off the floor. The reference is `floor_up_camera` when known. Without it the reference is camera -z, right for an overhead camera and up to the tripod tilt off otherwise, so it only decides when the two solutions differ by more than 15 degrees against it. Otherwise, or when the two lie within 2 degrees of a known floor, the lowest reprojection error wins.
- Floor normal learning: the floor's up direction in the fixed camera frame is learned from the tags themselves. Every frame gives two candidates, one per IPPE solution. The right ones all agree, since the robot drives on one floor; the flipped ones move with the robot's position and heading. Until one direction has the support of most frames and no rival cluster does, `estimate()` stays empty and selection falls back to reprojection error. After that it tracks slowly, so a wheelie or a flip does not drag it.

## Synthetic sysid session

``tests/python/test_tag_pose_smoother.py`` builds one session with known truth for the readers,
clock fit, IPPE choice, smoother and window gating.

- The robot trajectory has stills, straight holds, a reversal, spins both ways, an arc and a nose
  lift, seen by a tilted pinhole camera.
- Each frame's tag corners get pixel noise and go through
  ``cv2.solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)`` for the two IPPE solutions, as the app does.
- ESP32 events run on a drifting robot clock and arrive with WiFi jitter.
- The tag mounts follow the CAD export: tag 76 on top, tag 41 underneath, both tilted about 9.6
  degrees.
- The recorded field frame puts z into the floor, as ``FiducialFieldFilter`` does, so the fixture
  applies a flip about x to match.

## Sysid script behavior

### `playground/calibration/analyze_apriltag_mcap.py`

`analyze_apriltag_mcap.py`:

Solve field-plane robot poses from an apriltag_track.py image recording.

```
Replays the recorded camera images, re-runs the AprilTag detection (apriltag_detect, the same code
the live preview used), and solves the (t, x, y, yaw, visible) truth CSV that fit_plant_calib.py
consumes.
Two solves, mirroring capture:
  1. The floor extrinsic from the /floor/image lock burst (board visible).
  2. The robot tag pose in that floor frame, per /camera/image frame.
```

Because the images are recorded raw, the floor lock, detector tuning, intrinsics, and yaw offset can all be corrected here and re-run without re-driving the robot.

```
Transmitter data recorded alongside the frames is carried through to the truth CSV as extra columns,
so
the commanded input stays aligned with the solved ground-truth pose:
  - a read-only stick capture (radio connected, no --drive) adds tx_sample_t, ch0..chN, joined per
    frame by timestamp.
  - an apriltag_track.py --drive excitation adds the active scripted command (cmd_lin, cmd_ang,
    trainer_lin, trainer_ang, label), zero-order-held onto each frame time. fit_plant_calib.py fits
    the plant directly from this single CSV, so no separate command log is needed.
```

```
Usage:
    source scripts/activate_python.sh
    python playground/calibration/analyze_apriltag_mcap.py         playground/calibration/out/apriltag_track.mcap         --out playground/calibration/out/truth_log.csv
    # Re-solve with a corrected yaw offset (overrides the value recorded at capture):
    python playground/calibration/analyze_apriltag_mcap.py rec.mcap         --yaw-offset-deg 90 --plot fit.png
    # Also emit a Foxglove overlay (camera frames + solved pose markers via TF) to eyeball the fit:
    python playground/calibration/analyze_apriltag_mcap.py rec.mcap --overlay-mcap overlay.mcap
```

- `ROBOT_DETECT_SCALE`: Upsample factor for robot-tag detection. The tag is ~40 px at 1080p and drops out during fast motion; detecting on a 2x image recovers borderline frames (the floor board is large, solved 1x).

### `playground/calibration/check_sysid_recordings.py`

`check_sysid_recordings.py`:

Check Mr Stabs Mk2 drive recordings before they go into the MuJoCo plant fit.

Reads each MCAP once and reports, per recording, whether it can feed smooth_tag_poses.py and fit_mujoco_plant.py, and what is missing if it cannot. Nothing here fits anything: it is the look you take the morning after a session, before deciding whether to drive again.

Checks, each PASS / WARN / FAIL:

- file: the MCAP has a summary. A missing one means it was copied while still recording
  (re-download it) or the app died mid-write.
- camera: frame rate from /camera/frame_meta (60 fps target) and the longest gap.
- video: /camera/video against /camera/frame_meta. The ZED X One records its footage into the
  MCAP, not an SVO, so this is the only copy to replay.
- field: a field -> camera transform exists and stays put. The tripod does not move, so more
  than 5 mm or 0.5 deg of wander means a re-init or a bumped tripod partway through.
- robot tags: share of camera frames with a robot AprilTag, overall and while the sticks are
  off center, and the tag's pixel size. The smoother's window gate is 70% coverage.
  Recordings from the match profile carry no tags. They need /camera/video replayed through the
  sysid keypoint model first (config/playback/mr_stabs_mk2_sysid_playback.toml).
- our-robot track: share of frames with an OUR_ROBOT marker (YOLO path, match profile).
- robot telemetry: /robot/espnow_telemetry, or /robot/esp32_diagnostics before 2026-10-25.
  Events against the 100 Hz telemetry rate, gaps, pack voltage. The fit's command tape is the
  per-motor left_cmd/right_cmd from this stream, so without it there is no command tape.
- sticks: the maneuver menu, measured from the radio's stick channels, or from the telemetry's
  a_percent/b_percent when the radio was not on USB. Seconds of straight holds per level and
  direction, creep, spins, arcs, reversals, punches, and hard stops.
- SVO (ZED 2i only, with --svo-dir): an SVO whose frame stamps overlap this recording, and how
  many frames it holds.

```
    python playground/calibration/check_sysid_recordings.py data/recordings/*2026-10-11*.mcap \
        --svo-dir data/svo --plots playground/calibration/out/mujoco_sysid/checks
```

Exit status is 1 if any recording has a FAIL, so it can gate a script.

- `LINEAR_CH`: Stick channels, from the sysid and zed_box profiles: CH1 linear, CH2 angular, +/-1024 full scale.
- `CAMERA_GAP_FAIL_S`: A stall this long splits the session for the smoother anyway /camera/video messages per /camera/frame_meta.
- `TAG_MIN_PX`: plan: at least ~40 px on the far side.
- `PERCENT_TO_CHANNEL`: Telemetry a_percent/b_percent to radio channel units. CH1 runs opposite to a_percent and CH2 with b_percent (radio delay correlations, mujoco_sysid/s01_2026-10-15/session.json).
- `MENU_TARGETS`: Session totals worth aiming for before calling a recording day enough. Rough: enough repeats per stage of the staged fit to hold a session out and still fit on the rest.
- `check_video`: Over 100% is normal: the encoder starts at camera open, a second or so before the first pipeline tick writes frame_meta. Under it means the encoder dropped frames.

`check_field`:

Each field init redefines the field frame, so a session is a run of locks. The smoother wants one; report how many there were, how long the longest held, and whether the camera wandered within it (a bumped tripod).

- `check_track`: Informational when tags exist: the fit reads tags, this is only the live keypoint path.

`sticks_from_telemetry`:

The handset sticks as the robot saw them, in the radio channel frame's columns. With autonomy off the telemetry's arbitrated a_percent/b_percent are the handset's.

`stick_centers`:

Resting value of each stick channel: the value held longest. A trimmed or worn gimbal rests a few counts off zero, which would otherwise read as constant creep.

### `playground/calibration/detect_robot_yolo.py`

`detect_robot_yolo.py`:

Run the YOLO pose engine over a sysid recording's video and write our robot's detections.

One row per ``MR_STABS_MK2`` detection: the box, its confidence, and the front and back keypoints in pixels. A frame can have several, because the model sometimes labels the floor marker board as the robot; ``smooth_tag_poses.py --yolo`` drops those with size gates and keeps the best survivor per frame, projects it onto the field, and uses it where the tag drops out. Kept as its own stage so the detections can be inspected, and so the smoother does not need TensorRT.

```
    source scripts/activate_python.sh
    python playground/calibration/detect_robot_yolo.py data/recordings/<session>.mcap \
        --out playground/calibration/out/mujoco_sysid/<session>/yolo.csv
```

### `playground/calibration/find_flip_accel.py`

`find_flip_accel.py`:

Find the max forward acceleration a robot tolerates before it flips (trainer link ONLY, no tracking).

A torque-happy rammer (e.g. Mr Stabs) backflips when you slam the throttle from rest: it is the acceleration (how fast the command rises), not the top speed, that pitches it over. This ramps the forward command 0 -> target at an escalating slew rate and stops at the rate that flips it. Feed the result is the safe forward command slew rate, so scripted excitation and the deployed controller slew-limit every motion to stay upright, instead of blindly capping speed.

No camera, no MCAP, no AprilTag. The measurement is operator-in-the-loop: you watch the robot and answer whether it flipped. Ctrl-C during a ramp aborts that trial and zeroes the link.

```
SAFETY
- Guard plates ON, competition battery, surface matched to the NHRL arena floor (plate friction is
  part of the plant, see am32_tuning.md).
- Clear, bounded lane several metres long: each trial ramps to full speed and coasts.
- Driver sticks CENTERED: in trainer mode the radio ADDS stick input to the command.
- The link is zeroed and disarmed on every exit (normal, Ctrl-C, exception, per-trial hard timeout).
```

```
Usage:
    source scripts/activate_python.sh
    python playground/calibration/find_flip_accel.py --robot mr_stabs_mk2
```

- `main.in_ms2`: d(speed)/dt = d(command)/dt * v_max, since speed ~= command * v_max (deadzone/lag aside).

### `playground/calibration/fit_encoder_scale.py`

`fit_encoder_scale.py`:

Fit metres per encoder count from hand-pushed runs over measured distances (runbook E4).

The encoder scale is the constant every linear measurement depends on, so it is measured rather than inferred from a top speed. Push the robot with the motors disarmed between two marked points, dwelling at each so the stops are unambiguous, and this reads the count at each dwell and fits counts against distance through the origin.

```
    source scripts/activate_python.sh
    python playground/calibration/fit_encoder_scale.py \
        --run 0.5  LOG-38.TXT LOG-39.TXT LOG-40.TXT \
        --run 0.75 LOG-42.TXT LOG-43.TXT LOG-44.TXT \
        --run 1.0  LOG-45.TXT LOG-46.TXT LOG-47.TXT \
        --plot out/encoder_scale.png
```

Several distances beat one distance repeated. A fixed error (the marks, or the robot not quite stopping on them) is a constant offset in counts, so it bends a counts-against-distance line away from the origin. With one distance that offset is invisible and lands entirely in the scale; with three it shows up as an intercept, which this reports.

An out-and-back is worth more than a one-way push, because returning to the start makes the closure error visible, and closure is the only check a single pass affords.

`find_dwells`:

Stretches where the count barely moves, which is where the marks are.

The tolerance is in counts rather than a velocity threshold: a hand push is slow, so a velocity bar tight enough to exclude the push would also split every dwell in two.

`Fit`:

Both fits, and which one to believe.

Through the origin is the model you want to be true: zero distance, zero counts. The two-parameter fit exists to test it. When the intercept is real, the through-origin slope is pulled up by a fixed per-pass error and overstates the scale, while the differential slope is honest because a constant offset cancels out of it.

The per-pass `far_dwell_spread_counts` is the spread among the candidate far dwells. It makes wheel creep visible.

`fit_scale`:

Counts against distance, through the origin, plus an intercept as a diagnostic.

Through the origin is the model: zero distance is zero counts. The intercept is fitted separately only to check that assumption. An intercept far from zero means a fixed error per pass, most likely the marks or where the robot actually came to rest, and it inflates the scale fitted from short pushes.

### `playground/calibration/fit_gyro_noise.py`

`fit_gyro_noise.py`:

Fit the gyro noise model from a long stationary recording (runbook E2).

A bias estimate taken from a 10 s still hold is only good for as long as the bias stays put. The Allan deviation curve says how long that is: it falls while averaging still helps, bottoms out where bias instability takes over, and climbs once the bias itself is wandering. The tau at the minimum is the answer to "how often do the still holds need retaking", and the two slopes on either side are the process-noise terms the filter needs for heading.

```
    source scripts/activate_python.sh
    python playground/calibration/fit_gyro_noise.py \
        playground/calibration/out/gyro_bias/LOG-97.TXT \
        --plot playground/calibration/out/gyro_noise.png
```

Runs on one log, so a 30 min recording of a robot doing nothing is the whole experiment. Longer is better: the curve is only trustworthy out to about a tenth of the record, because past that there are too few independent clusters left to average.

Three axes plus yaw. Yaw is taken along measured gravity rather than off whichever gyro channel happens to point up, matching E0 and fit_gyro_scale.py: at rest the accelerometer reads +1 g on the axis pointing up, so the unit vector along it is the yaw axis whatever the mount looks like. That survives a tilted remount where a fixed channel would not.

The estimators follow IEEE 952:

```
    N  angle random walk    sigma = N / sqrt(tau)     slope -1/2, read at tau = 1 s
    B  bias instability     sigma = 0.664 * B         slope 0, the floor of the curve
    K  rate random walk     sigma = K * sqrt(tau / 3) slope +1/2, read at tau = 3 s
```

Nothing here reads the encoder or the command log, so this is the one experiment that needs no radio and no clear floor.

- `MAX_CLUSTER_FRACTION`: Clusters longer than this fraction of the record average too few independent samples to mean anything: at tau = T/3 there are three clusters, and the curve turns into noise about its own noise. IEEE 952 draws confidence bands instead; truncating is the blunt version and is enough to keep a reader from over-reading the right-hand tail.

`overlapping_allan`:

Overlapping Allan deviation of a rate signal.

```
Works on the integrated angle, which turns the three-sample second difference into two
array slices and makes the whole sweep a handful of passes over the data instead of a
Python loop per cluster.
```

```
    sigma^2(tau) = sum (theta[k+2m] - 2 theta[k+m] + theta[k])^2 / (2 tau^2 (n - 2m))
```

Overlapping rather than plain: every offset within a cluster contributes, which buys roughly sqrt(3) tighter confidence for the same record at no extra cost here.

`_read_at`:

```
Value of the slope-`slope` asymptote at tau = `target`.
```

```
Reading a single point off a noisy curve throws away most of the record, so this fits the
line of the known slope through `band` and evaluates it at the target. `band` defaults to
the decade around the target, which keeps the fit inside the region where that slope
actually holds instead of dragging in the part of the curve governed by a different term.
Pass a band explicitly when the asymptote lives somewhere other than near the target,
which is the case for rate random walk on any record short enough that the +1/2 region
has not opened up by tau = 3 s.
```

- `RRW_RISE_RATIO`: A record only measures rate random walk if its curve has actually turned up. Requiring the far end to sit this far above the floor rejects a curve that merely stopped falling, where fitting a +1/2 line would report the noise floor dressed up as a drift term. `AllanCurve.rrw_deg_per_s_sqrt_s` is `None` when the curve never turned up inside the record. That is a statement about the record's length, not about the gyro.

`yaw_rate`:

Gyro projected onto measured gravity, in deg/s.

The whole record is stationary, so the mean accelerometer vector is gravity and there is no centripetal term to swamp it. A 20 mg accel bias tilts this by about 20 mrad, which leaks 2% of the roll and pitch rates into yaw; on a robot sitting still that is 2% of nothing.

- `make_plot`: The +1/2 guide is anchored in the region that actually rises, and only drawn over that region. Drawn across the whole axis from a tau = 3 s anchor it would sit a decade under the data and imply a drift term this record cannot see.

### `playground/calibration/fit_gyro_scale.py`

`fit_gyro_scale.py`:

Fit the gyro scale factor from rotations through a known angle (runbook E3).

Integrating the gyro over a rotation of known size and comparing to that size gives one number: how much the gyro over- or under-reports. Every heading estimate is multiplied by it, so an unmeasured 2% here is 2% of heading error that no filter tuning removes.

```
    source scripts/activate_python.sh
    python playground/calibration/fit_gyro_scale.py \
        --run 3600 LOG-a.TXT \
        --run -3600 LOG-b.TXT \
        --plot playground/calibration/out/gyro_scale.png
```

`--run` takes the true rotation in degrees, signed: positive counter-clockwise viewed from above. Run both directions. A scale that differs between them is not a scale factor at all, it is a mounting or axis-sign problem, and averaging the two would bury it.

The yaw axis comes from gravity, not from a note about how the jig is bolted on. At rest the accelerometer reads +1 g on whichever axis points up, and the IMU puts gyro and accel on one right-handed triad, so up plus the right-hand rule fixes positive yaw as counter-clockwise from above. That survives a tilted remount, which no single gyro axis would.

Bias dominates the error budget on a slow rotation. A 0.05 deg/s bias over a 20 s turn is 1 deg, so the bias is taken from the still hold that opens the run and ramped to the one that closes it. A run with only one still hold gets a constant bias and says so.

- `TURN_EPS_DPS`: Below this the robot counts as stationary for the purpose of finding where the turn starts and stops. Well above the bias, well below a hand rotation.
- `main`: Through the origin over signed angles, so both directions constrain one number.

### `playground/calibration/fit_mujoco_plant.py`

`fit_mujoco_plant.py`:

Fit the Mr Stabs Mk2 MuJoCo plant to smoothed sysid sessions, then validate on held-out ones.

Inputs are session bundles from smooth_tag_poses.py (playground/calibration/out/mujoco_sysid/ <session>/). Hold out whole sessions, never windows from a training session.

```
    # fit on three sessions, validate on a fourth
    python playground/calibration/fit_mujoco_plant.py fit \
        out/mujoco_sysid/s01 out/mujoco_sysid/s02 out/mujoco_sysid/s03 \
        --holdout out/mujoco_sysid/s04 --restarts 4
```

```
    # re-validate an existing fit
    python playground/calibration/fit_mujoco_plant.py validate out/mujoco_sysid/fit_<ts> \
        out/mujoco_sysid/s04
```

Validation writes ``validation.json`` and, under ``plots/``, the loss curves, the restarts' parameters, held-out rollouts against the measured robot, and the error per horizon (see ``auto_battlebot/mujoco_sim/plots.py``). ``validate --name X`` writes ``validation_X.json`` and ``plots_X/`` instead, so one fit can be scored on several holdouts side by side.

`fit` prints its total rollout-steps up front. 1.64e9 steps take about 27 minutes on the RTX 4080 laptop.

### `playground/calibration/fit_plant_calib.py`

`fit_plant_calib.py`:

Fit the full tank drivetrain plant from a calibration session.

```
Input is one CSV from analyze_apriltag_mcap.py run on an apriltag_track.py --drive recording. That
single file carries both halves the fit needs, recorded in one process on one absolute
CLOCK_MONOTONIC clock (no
time alignment is done here):
  - the solved ground-truth pose per frame   (t, x, y, yaw, visible)
  - the scripted command active at that frame (cmd_lin, cmd_ang, trainer_lin, trainer_ang, label)
```

```
It extends playground/control_stage0/fit_plant.py: the same first-order AR(1) identification (v[k+1]
= a*v[k] + b*cmd[k], tau = -dt/ln(a), gain = b/(1-a)) and lag-by-cross-correlation idea, but driven
by clean AprilTag truth and the labelled excitation protocol instead of noisy MCAP perception poses.
That lets us pull out what the MCAP fit could not: per-direction deadzone, forward/reverse gain
asymmetry, separate accel vs coast (decel) time constants, the steer-brake coupling, and the
actuation lag.
```

Outputs: a printed report, the `[our_robot]` block to paste into simulation/kinematic_sim.toml, a recommended `lifted_deadzone_percent` for config/main.toml, and a validation plot (--plot).

```
Usage:
    source scripts/activate_python.sh
    python playground/calibration/fit_plant_calib.py         playground/calibration/out/truth_log.csv         --plot playground/calibration/out/fit.png
```

`load_session`:

Load one CSV from analyze_apriltag_mcap.py (a --drive recording) into a fitting Session.

The CSV carries both halves the fit needs: the solved pose per frame (x, y, yaw, visible) and the scripted command active at that frame (cmd_lin, cmd_ang, label, zero-order-held onto the frame time). Commands are read from every row (the full timeline, including frames where the tag was not visible); pose is read only from visible frames.

- `load_session`: Commands and poses come from one recording on one absolute CLOCK_MONOTONIC clock, so no time alignment is needed. The sync twitches at the protocol's start and end are a visual cross-check in `--plot`.
- `load_session`: Commands span the full timeline. Frames before the first issued command have empty cells; treat the command there as zero (no command) and the label as unset (matches no protocol phase).

`fit_deadzone`:

Smallest staircase command level (same sign) whose sustained motion exceeds eps.

Works per command level, not per sample: the staircase holds each command for ~0.8 s, so the median |velocity| over that hold is compared to the threshold. This rejects single-sample noise spikes that a per-sample test would mistake for motion onset.

- `_rise_tau`: The leading rise; ignore later re-entries from steady-state noise.
- `_drive_segments`: Steady-state speed from in-frame samples only; interpolated steady samples would bias vss (and, through it, both the gain and the rise-tau fit).

`fit_drive_segments`:

Per held-drive segment: steady-state gain and rise tau, bucketed by command sign.

Returns (gains_fwd, gains_rev, taus_fwd, taus_rev). Gain is Vss / effective-command (deadzone removed), which is a steady-state quantity and so immune to the smoothing/noise tau trade-off.

- `_decay_segments`: The coasting portion must be measured, not interpolated.

`fit_steer_brake`:

Fractional forward-speed loss per unit |angular command| during the steer-brake grid.

```
One steady-state forward-speed sample per grid cell (last half of each drive segment), so the
rising transient does not bias the slope. speed/speed0 = 1 - coeff*|ang|.
```

`_latency_stack`:

Pool every clean command edge and stack the acceleration that follows it to time the transport lag.

The dedicated latency battery is not trusted on its own: its aggressive bipolar steps flip the robot or drive it out of frame, leaving the truth pose (and so the velocity) mostly interpolated across dropouts. Instead every clean command edge in the whole session is pooled. Linear edges are timed against forward acceleration (dv/dt), angular edges against yaw acceleration (dw/dt); the transport delay is the same physical path (Crossfire + ESC + mechanical) for both, so they stack together. Any edge whose response window overlaps a truth gap is dropped, which is what removes the flipped latency segments and leans the estimate on the many in-frame angular-step onsets.

Each edge contributes its post-edge acceleration, sign-flipped to the step direction so accels and decels add rather than cancel, and scaled by the channel's own accel spread so m/s^2 and rad/s^2 are comparable. Stacking many onsets averages down the per-frame perception noise that made the single latency-battery correlation swing wildly between runs. Lag is the peak of the averaged acceleration pulse, which for a delay + first-order plant sits exactly at the transport delay: dv/dt is zero until motion starts then jumps to its maximum right at the delay. The peak is far more robust to noise here than a first-threshold crossing at this frame rate and onset count.

- `_context`: The deadzone test compares a per-level MEDIAN over a ~0.8 s hold, which averages down the noise by ~sqrt(N); use a gentler threshold than the per-sample motion floor so the onset is not pushed high.
- `_panel_latency`: Zoom to the averaged pulse: individual onsets are ~10x noisier and would flatten it off- scale.

### `playground/calibration/jig_report.py`

- `TARGET_REL`: Relative spread below which a parameter counts as measured. 10% is the point where the parameter stops being the thing limiting the model.

### `playground/calibration/make_print_tags.py`

`make_print_tags.py`:

Generate print-ready AprilTag PDFs for drivetrain calibration: a floor grid and a robot tag.

The PDF export (pixels-per-mm -> DPI, so a 100%-scale print is dimensionally exact) is adapted from true_battlebot scripts/tags/tag_tools.py. The tags themselves are rendered with cv2.aruco (DICT_APRILTAG_36h11) so the printed geometry exactly matches the GridBoard model apriltag_track.py uses for pose. cv2.aruco's ids match the canonical AprilRobotics tag36h11 family (verified), so these are real AprilTags.

Sizing keeps the camera resolution in mind. apriltag_track.py runs the OAK-1 W at 1080p (1920x1080 @ 60 fps); the script prints the estimated marker size in pixels at that resolution for the given mount height. Face-on, 36h11 needs ~18 px edge to decode, so aim well above that (>= ~30 px). If your mount is high and the estimate is small, increase --floor-marker-size / --robot-tag-size, lower the mount, or step up to 4K.

```
Usage:
    source scripts/activate_python.sh
    python playground/calibration/make_print_tags.py --out-dir playground/calibration/print
```

The floor grid is normally the manufactured board (3x5 of 65 mm markers, 15 mm gaps, ids 160..174 numbered right-to-left per row); this script reproduces it only if you need a reprint. The robot tag (id 20, off the floor's 160..174 range) is the piece you actually print here. Defaults match apriltag_track.py. Print at 100% (no "fit to page"), tape the tag flat on the robot top.

`save_pdf`:

Save a grayscale image as a PDF whose page is dimensionally exact when printed at 100%.

The tags are pure black/white, so the image is stored bilevel (mode "1"): PDF keeps that lossless (no JPEG softening of the marker edges), and the file stays small.

- `main`: Camera sizing estimate (defaults: OAK-1 W wide lens at 1080p).

### `playground/calibration/make_robot_tag_3d.py`

`make_robot_tag_3d.py`:

Generate a multi-material 3D-print file for the robot's AprilTag (Bambu Lab AMS).

This is the 3D-print counterpart of make_print_tags.py for the tag that mounts on the robot top. Instead of a paper tag you tape down, it emits a printable plate: a white base with the black AprilTag cells raised on top as a second material. Two watertight objects share one coordinate frame, so in Bambu Studio you assign white to the base and black to the tag and slice.

The black geometry comes straight from the cv2.aruco DICT_APRILTAG_36h11 bit pattern, so the printed tag is geometrically identical to what apriltag_track.py detects (id and orientation verified by re- detecting a top-down raster before writing). A white margin around the tag gives the quiet zone the detector needs.

The 3MF is written directly (a zip of XML) so it needs no lxml: two named, pre-colored objects that import into Bambu Studio already split into a white base and a black tag. STL pairs are written too.

```
Outputs, written to --out-dir:
    robot_tag.3mf         two objects (white base + black tag), pre-colored
    robot_tag_white.stl   base only
    robot_tag_black.stl   tag only
```

```
Usage:
    source scripts/activate_python.sh
    python playground/calibration/make_robot_tag_3d.py --out-dir playground/calibration/print_3d
```

Default: id 0, 130 mm tag, 2 mm white base, 0.6 mm raised black, 16 mm white margin (a ~1-cell quiet zone). The plate is 162 x 162 mm, well within a 256 mm Bambu bed. Bambu Studio: import robot_tag.3mf, confirm the tag object is black filament and the base is white, then slice. Use 0.1-0.2 mm layers and a base thickness that is a clean multiple of the layer height. Note its physical edge (--tag-size) and heading offset for apriltag_track.py --tag-size / --yaw-offset-deg.

`black_cell_boxes`:

Boxes for the black cells, raised on top of the base. origin is the tag's (x0, y0) corner in mm.

Consecutive black cells in a row are merged into one box to keep the mesh small and free of internal coplanar faces.

### `playground/calibration/plot_jig_runs.py`

`yaw_rate_dps(log)`: yaw about the measured gravity axis, in deg/s. `JigLog.gyro_rad_s` is rad/s, so this converts. The panel axis says deg/s, and a robot spinning at 700 deg/s plotted as 12 reads as a robot going perfectly straight. Yaw comes from gravity rather than a named axis so a tilted or rotated remount does not silently invalidate the plot. It falls back to the z row when there is no still segment to measure gravity from.

### `playground/calibration/plot_sysid_bundle.py`

`plot_sysid_bundle.py`:

Plot a smoothed sysid bundle so each smoothing check can be judged by eye.

Reads a bundle written by ``smooth_tag_poses.py`` and writes, under ``<bundle>/plots/``:

- ``overview.png``: the whole session on one time axis. Which source fixed the pose at each
  moment (tag, YOLO, neither); the smoothed yaw rate against the robot IMU's on a log scale,
  with the 50 rad/s line no real drive crosses; pitch against the flat band the window gate
  uses; x and y against the box the near-rail gate assumes; and the fit windows, by kind.
- ``floor.png``: top-down, every tag and YOLO fix, the smoothed path, the box, and where each
  fit window starts.
- ``dropouts.png``: one top-down panel per tag dropout longer than ``--min-gap-s`` in which the
  robot stayed upright, 0.5 s either side. ``--compare`` overlays a second bundle of the same
  recording (for example one smoothed without ``--yolo``).

```
    python playground/calibration/plot_sysid_bundle.py \
        playground/calibration/out/mujoco_sysid/<session>_yolo \
        --compare playground/calibration/out/mujoco_sysid/<session>_tag
```

- `_imu_yaw_rate`: Heading is clockwise.

### `playground/calibration/render_mujoco_overlay.py`

- `Scene.__init__`: Video stamps sit this far off the tag image clock (the YOLO calibration measures it).

### `playground/calibration/smooth_tag_poses.py`

- `main`: Position and heading only; the 2D keypoints carry no height or tilt.
- `main`: The video frames YOLO filled in count as camera frames for window coverage, so a dropout the fixes cover reads as covered and a rejected fix still counts against it.

### `playground/calibration/sysid_coverage.py`

- `run`: Unpaced replays advance on recording time, so a fast replay closes the same windows as a live run would.

## Workarounds

- **`mujoco_sim/onshape_export.py` frames and mass.** The export's root frame is the Onshape assembly frame: the axle runs along `X_asm` and the robot faces `-Y_asm`. The body frame used everywhere else (C++ filter, MJCF, smoother) is FLU at the axle midpoint, so `x = -Y_asm`, `y = +X_asm`, `z = +Z_asm`. The export carries no mass for the motors, ESCs, flight controller, batteries or switch (Onshape mass overrides do not survive it), so only the wheels are lumped from it. The chassis is the Onshape whole-assembly total minus the wheels; see `chassis_from_assembly`.
- **`mujoco_sim/rollout.py` kernel compile.** `_compile_kernels_before_capture` runs one step before `wp.ScopedCapture` so kernel compilation happens outside the CUDA graph. Unverified: same Warp requirement as the RL environment, not run without it.
- **`check_sysid_recordings.py` catches `Exception` on read.** The mcap reader raises a mix of exception types on a torn file, so the check reports any of them as an unreadable file.
- **`plot_jig_runs.py` drive channels.** The two drive channels the radio reported are logged directly since the format change. Older runs stored only the mixed body units, so the plot reconstructs the channels by inverting the tank un-mix that produced them.
- **`render_match_prediction_camera.py` video source and tail.** Only recordings before 2026-09 name a separate SVO in `svo_path`; video now lives on `/camera/video` inside the recording, so there is no path to follow. A JPEG stream tail can yield one truncated frame, and the renderer skips it rather than hand ffmpeg a short packet.

## Workarounds in tests

- `test_sysid_coverage.py` compares the mirror's steer output to the reported one within `HEADING_QUANTIZATION_TOL` (1.0). The reported heading is quantized to 1/16 degree and the heading hold sees that. Unverified: the telemetry encoder was not traced.
- `test_tag_pose_smoother.py` allows `MAX_CLEAN_FIXES_AT_GATE` (3) rejected clean fixes. The fused IMU's sigma is its spin residual (about 0.26 rad/s in the scenario), so a few clean fixes at the blackout edge and mid-spin sit right at the innovation gate.
