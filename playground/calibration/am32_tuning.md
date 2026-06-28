# AM32 ESC tuning for Mrs Buff MK3

Mrs Buff runs stock AM32 config, which is fine for most NHRL contestants but is not tuned for an
autonomous rammer. The priorities here are, in order:

1. Maximum acceleration and momentum at contact.
2. Never quit when pushing against an opponent or a wall.
3. Reliable motion at low command for fine aim (small deadzone).
4. Survive a ~3-minute match thermally.

The AM32 wiki (https://wiki.am32.ca/guides/ESC-Settings-Explained.html) writes its recommendations for
crawlers: slow, precise, hold-on-a-hill. That is the opposite of what a rammer wants. Use the wiki for what
each setting does, and use the table below for which way to set it.

Tune in the order below, and re-measure with the calibration tool after each change. Stock config is the
starting point, not the answer.

## Robot configuration during tuning

Tune with the **guard plates ON**, competition battery installed, on a surface matched to the NHRL arena
floor. The plates add significant ground friction, which sets how much Startup Power you need to break
static friction and how aggressively Stall Protection must push. Tuning plates-off under-sets the low end,
and the robot has a dead low end in competition. Bench static-floor friction is a lower bound: in-fight
downforce raises it and cannot be replicated on the bench.

## Must-set (drive correctness)

| Setting | Value | Why |
| --- | --- | --- |
| Bi-Directional (fwd/rev) | ON | A ground robot needs forward and reverse with neutral at center stick. |
| Reverse Rotation | per-ESC | Set so both wheels drive the chassis forward on a positive command, without swapping motor wires. Verify left/right after flashing. |
| Motor Poles / KV | actual motor specs | Wrong poles make the sine changeover stutter; wrong KV mis-sets the low-RPM restriction. Both masquerade as a large deadzone. |

## Fight survivability (the key deviations from crawler/stock advice)

| Setting | Value | Why |
| --- | --- | --- |
| Stuck Rotor Protection | OFF | The wiki warns it makes the ESC "give up" on obstacles. A bot that pins an opponent or hits a wall must keep pushing, not cut out. **The single most important change from defaults.** |
| Stall Protection | ON | Pushes harder as a loaded motor nears stall, which maintains a shove or pin. Watch motor heat. |

## Momentum vs control (measure both with the calibration tool)

These set how fast the robot stops when command returns to neutral, i.e. the `tau_linear_decel` / coast the
calibration tool fits. A rammer wants momentum into contact; an aim-orbit wants a quick settle. Decide
empirically.

| Setting | Starting value | Why |
| --- | --- | --- |
| Brake on Stop | OFF | Preserves coast/momentum. It only brakes at neutral throttle, so it never brakes mid-ram, but it shortens coast when aiming. Try ON if the controller overshoots while orbiting a target. |
| Running Brake Level | low / zero | High running brake kills momentum on throttle-down, which is bad for a rammer. |
| Stopped Brake Level | moderate | Enough to hold position between maneuvers. Not a performance lever. |
| Complementary PWM | ON | More linear throttle response and better low-speed control (active freewheeling). It adds braking on duty-down; if coast matters more than linearity, test OFF and compare the fitted tau. |

## Low-end responsiveness (shrinks the measured deadzone, improves fine aim)

| Setting | Value | Why |
| --- | --- | --- |
| Startup Power | raise from default | Raise until the wheels reliably break static friction at low command. This directly reduces the measured deadzone and lets `lifted_deadzone_percent` in `config/main.toml` come down. |
| Sinusoidal Startup | ON, small Sine Startup Range | Smooth low-RPM start avoids cogging/stutter at small commands. |
| Sine Mode Power | modest | The wiki warns high values generate a lot of heat. |

## Power / thermal (burst duty)

| Setting | Value | Why |
| --- | --- | --- |
| Timing Advance | moderate-to-high | More power and RPM at the cost of heat; acceptable for a short match. Back off if motors run hot. |
| Variable PWM | ON | Picks PWM frequency vs RPM automatically and avoids commutation interference. |

## Diagnostics

| Setting | Value | Why |
| --- | --- | --- |
| 30ms Telemetry | ON | KISS-protocol telemetry (RPM, voltage, current, temperature). Cheap insurance, and a future path to direct wheel-speed feedback even though ground truth here is the overhead AprilTag. |

## Not in this AM32 build's documented settings

Current limiting, temperature-limit cutoff, low-voltage cutoff, and bidirectional-DShot RPM are not on the
AM32 settings page. Do not rely on them. Thermal safety comes from sensible timing and sine-power choices,
not a hard limiter.

## Tuning loop (why this is paired with the calibration tool)

Each ESC setting change is followed by a re-measure so its effect is known, not guessed.

1. Dump the current AM32 config to `playground/calibration/am32_settings_<date>.txt` (versioned in git).
2. Run a calibration session (`apriltag_track.py --drive`) and fit the plant (`fit_plant_calib.py`).
3. Change **one** setting (e.g. Startup Power up, or Brake on Stop on), re-run, and compare deadzone,
   max speed, accel tau, and coast tau.
4. Keep the change only if the measured plant moved the way you wanted. Record the final config.

## What each tuned parameter feeds

- Measured physical deadzone -> `lifted_deadzone_percent` in `config/main.toml` (currently a guess of 10%).
- Coast tau (set by Brake on Stop / Running Brake) -> `tau_linear_decel` in `simulation/kinematic_sim.toml`.
- Max speed and accel tau -> the `[our_robot]` block in `simulation/kinematic_sim.toml`.
