# Velocity jig session tool

Runs the velocity jig experiment battery from a browser tab. It drives the robot, takes the clock
probes, times the still holds, checks the gates, and writes the session sheet, so the afternoon goes
into running experiments instead of filling in a form.

Procedure comes from `docs/experiments/kalman_filter/velocity_jig_runbook.md`. Design and the
reasoning behind it are in `PLAN.md`.

## Run it

```bash
cd firmware/velocity_jig/web_tool
python3 -m http.server 8000
```

Open `http://localhost:8000` in Chrome or Edge.

`file://` does not work. Web Serial needs a secure context, and `http://localhost` counts as one while
a bare file path does not.

## Rehearse without hardware

```
http://localhost:8000/?mock=1
```

Mock mode substitutes a simulated jig and transmitter that answer the real console protocol, including
`BUSY` during a recording and `STREAM` stopping on any input. The mock carries a deliberate -1234.567 ms
clock offset and 30 ppm skew, so the probe has something real to recover. The offset is negative because
the jig boots before the page opens, so its uptime leads the browser's. Buttons A and B appear in the
Mock jig panel.

Run the whole battery in mock once before the session. It takes a few minutes and it is the cheapest
way to find out that a step needs more floor than the workshop has.

## Connections

Two serial ports, connected separately from the header:

- **Jig**: the RP2040. 115200 baud. Console commands, clock probes, live stream.
- **TX**: the OpenTX transmitter in trainer mode. Filtered to USB VID 0x0483 / PID 0x5740.

The browser drives instead of a separate Python script so that command timestamps and clock probes
share one `performance.now()` origin. A separate driver process would stack a second unknown host
offset on top of the one the probe is trying to measure, and command-to-motion delay is about 60 ms,
which is the same size as the error that would introduce.

## Safety

**The driver's SF arm switch is the failsafe.** Everything below reduces how often it gets used. None
of it replaces it.

Automatic disarm fires on: tab hidden, window blur, Escape, page unload, a 100 ms watchdog on the
setpoint, and a hard timeout sized to the program. Tab-hidden is the one that matters in practice, since
a background tab has its timers throttled to 1 Hz while the last command keeps being obeyed.

Before any wheel touches the floor, bench the drive path with the robot **up on blocks** and confirm
every disarm path individually.

Weapon stays installed but disabled for the whole battery except E22, which is last and has its own
containment requirements.

## Space

Set board length, half width, and measured course in the Space panel. All three are live: change them
whenever the space changes and every prediction updates.

The tool fits each experiment to the space in this order:

1. Shorten the dwell, down to three time constants.
2. Shuttle, so successive repetitions alternate direction and the robot stays near where it started.
3. Scale amplitude, last, because it costs signal-to-noise and hides nonlinearity near full command.

Anything it had to do is stated in the run panel. If a run still does not fit, it says so rather than
running a truncated version quietly.

Width usually binds before length. Lateral drift grows with the square of run length while the space
budget grows linearly, so on a narrow board a 2% wheel mismatch walks the robot off the side before it
runs out of straight. The tool offers a closed-loop trim pass on the live gyro for those runs. The trim
is logged as a commanded angular value, never applied as a hidden offset, because a trimmed linear step
is a two-input excitation and a fit that does not know about the second input will blame the first.

## Gates

Gates run the moment a run closes, while the robot is still on the floor and re-running is cheap:

| Gate | Limit |
|---|---|
| Dropped samples | zero |
| Clock probe residual, pre and post | under 2 ms |
| Clock skew across the run | under 200 ppm |
| Encoder state | matches what the experiment needs |
| Sample count | at least 90% of duration times 1660 Hz |
| Still holds | at least 9 s each |

A failed gate marks the run discarded. Discarded runs stay in the export with their reason, since
knowing what was thrown away and why is part of the record. Every verdict is overridable.

Three checks cannot run here and are listed under "Offline checks still owed" in the exported sheet:
gyro bias drift between the two still holds, saturated gyro samples, and the per-run yaw axis. The
console is blind during a recording, so none of them can be evaluated until the log file is pulled.

## Export

**Export sheet** writes the runbook's session sheet as Markdown. **Export JSON** writes everything,
including the command log with per-command host timestamps, which is what the fit consumes.

Session state persists in `localStorage`, so a reloaded tab keeps its runs. Export anyway before
closing the tab.

## Clearing results

Three ways to throw work away, in increasing order of how much goes:

- **×** on a run row deletes that one record. Use it for a run that was botched in a way the gates
  cannot see, an operator error rather than bad data, when you would rather redo it than carry a
  discard through the sheet.
- **Clear runs** deletes every run record and keeps the setup: operator, robot, board length, IMU
  ranges, notes, and the session id. This is the one to use after rehearsing on the mock, so the
  practice runs do not reach the sheet.
- **New session** clears the runs and the setup fields and issues a new session id.

None of these touch the jig. The log files stay on the SD card, and the tool only forgets the rows
pointing at them. Clearing is refused mid-run. Export first if you want a copy, because clearing
overwrites the saved state and there is no undo.

## Diagnostics

The Diagnostics panel streams at 10 Hz and shows encoder count, gyro in dps, accel in g, saturation
flags, and the gyro projected onto gravity. That projection is the yaw rate, and it is how E0's wiring
check and the encoder reattach check get done. It works on a tilted or rotated mount, where no single
gyro axis is yaw anymore.

Two console commands sit next to the stream button:

- **LIST** shows what is on the card with sizes and a total, for confirming a run's file landed and
  for watching space over the afternoon.
- **TIME** runs a 50-probe clock check on demand and reports offset, residual, kept count, and median
  RTT. Run it before starting. If the residual is already over the 2 ms gate here, every run of the
  afternoon fails that gate too, and it is worth fixing before spending the time.

Diagnostics and a run cannot share the port, since the jig stops streaming on any input. Starting a run
stops the stream, and so do LIST and TIME. Both refuse to run mid-recording, because the jig answers
`BUSY` to everything but `TIME` while it is recording.

Either connection chip disconnects when clicked. Disconnecting the transmitter disarms it first, and
disconnecting either one mid-run aborts the run rather than leaving the coach waiting on a line that
can no longer arrive.

## Known limits

- **Two experiments saturate a 2000 dps gyro.** The range clips at 34.9 rad/s. E11 detached max spin
  reaches 61.5 and E15 detached reaches 36.9. Both are running at the amplitude their step text calls
  for, so the fix is a firmware change and not a smaller command: set `IMU_GYRO_RANGE` to 4000 dps and
  reflash before either one. The tool shows the clip threshold in the run panel and warns when a
  planned run crosses it. Every other experiment is capped at the E5 rate limit and stays inside 2000
  dps.
- **The space budget is open loop.** Predicted distance comes from integrating the stage-2 plant
  estimates, not from measured position. A `POS` command answered during recording would turn it into
  closed-loop distance feedback, which is the single change that would most improve running in a short
  space. See "Firmware follow-ups" in `PLAN.md`.
- **Encoder state is verified after the run, not before.** The count is zeroed at recording start and
  nothing resets it, so a short stream right after stop reads the run's net count. Adding A/B pin levels
  to the stream row would turn a discarded run into a blocked one.
