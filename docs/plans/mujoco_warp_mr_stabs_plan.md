# MuJoCo Warp simulation of Mr Stabs Mk2, tuned from hand-driven recordings

Goal: a rigid-body MuJoCo model of Mr Stabs Mk2 whose unmeasured parameters (floor contact, motor
and ESC, delay) are fit by running thousands of parallel MuJoCo Warp rollouts against overhead
AprilTag recordings of the robot being driven by hand. The mass properties come from CAD and are
held fixed. The fit is judged on held-out sessions against the grey-box plant fit on the same data.

This sits under `learned_sim_environment_plan.md`. That plan fits a kinematic plant and a sensing
model; this one adds the 3D rigid-body layer the kinematic sim cannot represent: pitch (the
backflip), traction limits, the drivetrain's reflected inertia, and later contact with walls.

The input is manual driving only. No chirps, steps or scripted waveforms. The robot drives in
the 1.52 m test box under a tripod-mounted ZED X One S, the main application on the ZED Box Mini
records the session, and the driver follows a maneuver menu (below).

## What is already known

Mass properties, from the Onshape assembly (screenshot 2026-09-25 23:10) and the Onshape URDF
export, lumped into three bodies. Frame is FLU at the axle midpoint (x forward, y left, z up). The
mapping from the Onshape assembly frame is x = -Y_asm, y = +X_asm, z = +Z_asm, confirmed by the
longitudinal-vertical product of inertia agreeing in sign between the two independent sources.

| Body | Mass | COM (mm) | Inertia about COM (kg m^2) |
| --- | --- | --- | --- |
| Chassis | 459.0 g (CAD 457.3 g + 1.7 g to match the scale) | (31.94, 0.00, 0.86) | Ixx 7.769e-4, Iyy 5.461e-4, Izz 1.2521e-3, Ixz -2.31e-6 |
| Wheel, each | 18.445 g | (0, +/-65.26, 0) | axle 4.2528e-6, radial 2.33e-6 |
| Whole robot | 495.9 g on the scale, 494.22 g in CAD | (29.55, 0.00, 0.80) | roll 9.387e-4, pitch 5.898e-4, yaw 1.449e-3 |

The URDF export alone sums to 328.9 g: the motors, ESCs, flight controller, batteries and switch
come out at zero, probably because their masses are Onshape overrides the exporter does not carry. The chassis above is therefore the Onshape
whole-assembly total minus the URDF wheels, not the URDF chassis.

Drivetrain, from the Repeat Robotics Compact 1806 product page and CAD:

| Quantity | Value | Source |
| --- | --- | --- |
| Gear ratio N | 22.6:1 | repeat-robotics.com/products/repeat-compact-1806 |
| Motor | BL 1806, 2300 KV, so Kt = 60 / (2 pi KV) = 4.152e-3 N m/A | same |
| Gearmotor mass | 44.5 g (motor alone 22.4 g) | same |
| Free speed at the output | 1130 rpm at 3S, 1500 rpm at 4S (ideal KV x V) | same |
| Rotor inertia J_r | 9.3e-7 kg m^2 (9.3 g cm^2) | CAD integration of flux ring, 14 magnets, aluminum end cap, shaft |
| Reflected at the wheel | N^2 J_r = 4.77e-4 kg m^2 per wheel | derived |
| Wheel radius | 25 mm | CAD |

The rotor number is cross-checked by mass: CAD rotor 9.8 g, plus stator laminations, base and
bearings, plus an estimated 3 to 4 g of copper, gives about 22 g against the 22.4 g listed motor.
A steel end cap would push it to about 25 g, so the cap is aluminum. The gearbox in the CAD is a
hollow shell, so the sun, planets and carriers are missing; bounding them at 1 to 2 g inside a 6 mm
radius adds at most about 4% to the rotor term.

Reflected inertia dominates the drivetrain. Each gearmotor acts like J/r^2 = 0.76 kg of extra
translational mass, about 1.5 kg against a 0.5 kg robot, and adds 6.5e-3 kg m^2 to yaw against the
body's 1.45e-3. The backflip follows from it: the chassis sees the drive reaction
F r + (J_w + N^2 J_r) alpha, and with the COM 31.9 mm ahead of the axle the nose lifts at about
0.29 g of forward acceleration. Without the rotor term (the Unity model) the threshold was 1.4 g,
above any traction the wheels have, which is why that model never flipped and the real robot does.

Plant behavior, from the 2026-07-03 and 2026-07-05 AprilTag sessions
(`docs/experiments/control_improvement/stage2_mr_stabs_mk2_calibration.md`):

- Actuation latency about 60 ms (run `175805`, pooled onset stack). The best-measured number.
- Linear decel tau about 0.078 s, accel tau about 0.058 s, angular tau 0.05 to 0.07 s. Each rests
  on one or a few segments.
- Deadzone at or below 0.04 on both channels.
- Reverse spins uncontrollably and settles slower than forward (1.08 to 1.21 m/s against 1.31 to
  1.33 m/s at |command| 0.25). The plant is asymmetric.
- Linear motion loses the tag: the robot leaves the OAK-1 view within a second at anything above
  a quarter stick. Tag detection runs 50 to 68% at 1080p even with 2x upsampling.

One inconsistency this model should settle. The stage-2 fit reports max forward speed 5.6 m/s,
but the gearmotor's ideal free speed is 3.93 m/s at 4S (4.64 m/s on a full LiHV pack). The 5.6 was
a linear extrapolation from commands 0.2 to 0.5, so the command-to-voltage map is probably not
linear (AM32 throttle curve, radio scaling), or the measured speeds are off. The MuJoCo actuator
has a hard voltage ceiling, so the fit will expose which.

Firmware between the radio and the ESCs (`firmware/mr_stabs_mk2/src/main.cpp`):

- **Auto-steer** (flip switch DOWN, the power-on default) runs a heading-hold PID on BNO055 yaw:
  kp 0.08, ki 0.01, kd 0.01 (percent per degree), 2 degree deadband, output added to the left/right
  mix. It only acts while the turn stick is within 1% of center, and waits 0.25 s after a turn
  before engaging. Any commanded turn passes straight through. The same switch position also turns
  on upside-down detection.
- **Mixing**: left = -a + turn, right = -a - turn, both scaled down together if either exceeds
  100%. The ESC stop threshold is 1% per motor and is tunable over the diagnostics server.
- **Diagnostics stream**: an event stream on the robot's WiFi access point carrying `left_cmd`,
  `right_cmd`, `a_percent`, `b_percent`, `pid_setpoint`, `pid_output`, BNO055 orientation and
  acceleration, stamped in robot milliseconds. It sends at 10 Hz, or every control loop in its
  recording mode.
- A failsafe stops the robot after 5 s of identical radio frames. Hand driving never produces that.

Fitting runs with auto-steer on, the way the robot competes. The PID does not have to be fit or
even modeled for this: the diagnostics stream logs the per-motor commands after the PID and the
mixer, and those are the drivetrain's input. The drivetrain is fit open loop on them. The PID is
reproduced in the sim separately, from the gains above, for closed-loop use.

## Recording

The main application on the ZED Box Mini is the recorder. It already runs the ZED X One S, locks
the field from the floor marker array, records MCAP, and takes remote commands. Three additions
make it a data-collection tool for this fit: a transmitter subclass with an ESP32 WiFi
diagnostics component, an AprilTag keypoint model, and a profile that selects both.

### Session procedure

1. Put Mr Stabs Mk2 in the 1.52 m plywood drive-test box. Mount the ZED X One S on a tripod about
   0.75 m above the box floor, looking down into the box, and connect it to the ZED Box Mini
   over GMSL2.
2. Connect the ZED Box to ethernet for remote control. A second WiFi dongle on the box joins the
   robot's `MR-STABS` access point (see "Networking" below). The radio also plugs in over USB.
3. Start the application on the ZED Box with the `mr_stabs_mk2_sysid_zed_box` profile and start
   an MCAP recording (`/command/set_recording`).
4. Put the floor marker array (the 3x5 AprilTag GridBoard, ids 160 to 174) in the box and press
   "field init" (`/command/reinit_field`). `FiducialFieldFilter` accumulates 10 frames and locks
   the field frame.
5. Remove the floor marker array.
6. Drive Mr Stabs Mk2 around for a few minutes, following the maneuver menu below.
7. Stop and save the recording.

The robot's flip switch stays DOWN (auto-steer on), the competition setting. Heading hold only
acts while the turn stick is within 1% of center, and the ESP32 stream logs the per-motor
commands after it, so the drivetrain fit stays open loop (see "Firmware" above).

### Camera geometry at 0.75 m

With the SDK's rectified intrinsics (fx 665, fy 716 at 1920x1200, a 111 x 80 degree view), a
camera pointing straight down from 0.75 m sees about 2.0 x 1.2 m at the tag's height, 5 cm above
the floor. An 80 mm tag spans about 76 px, nearly twice the 40 px stage 2 worked with, and the
global shutter (the SDK opens the camera as a "ZED XOne GS") removes rolling-shutter skew on a
moving tag.

The box floor is 1.52 m, so pointing straight down covers the long axis but leaves about 0.35 m
of the short axis out of view. Tilt the tripod head until the whole floor is in frame. PnP solves
the tag in 3D, so a tilted view costs pixels on the far side, not accuracy. Check the far corner
still gives the tag at least about 40 px, and set a short manual exposure to limit motion blur.

### Networking

- Ethernet carries remote control, ssh, and copying recordings off the box.
- A second USB WiFi dongle joins `MR-STABS`, so the built-in AX210 stays with whatever it does
  now. Bind the connection to the dongle's interface (`nmcli connection modify MR-STABS
  connection.interface-name <dongle>`) so NetworkManager never brings it up on the AX210.
- The ESP32's access point offers itself as a gateway, so set the connection to never take the
  default route (`nmcli connection modify MR-STABS ipv4.never-default yes`). Otherwise the box
  can route remote-control traffic into the robot.
- The box has one USB3 Type-A port and the radio needs it too, so the dongle and the radio share
  it through a hub. Pick a dongle with a Linux driver in the box's 5.15 kernel, or one Stereolabs'
  L4T build already carries; an out-of-tree driver would have to survive the held kernel.
- `HostServices` changes the box's WiFi through `/command/set_wifi_access`. Check it only touches
  the AX210 and leaves the dongle alone while recording.

### Addition 1: ESP32 WiFi diagnostics in an OpenTxTransmitter subclass

`Esp32DiagnosticsOpenTxTransmitter` extends `OpenTxTransmitter`
(`include/transmitter/opentx_transmitter.hpp`) and owns a new component, `Esp32WifiDiagnostics`.
The base class keeps doing everything it does now: stick channels, CRSF telemetry, trainer
output, and its `/diagnostics/opentx_transmitter` logging. The subclass overrides `initialize()`
and `update()`, calls the base versions, and adds the diagnostics stream around them. It needs
none of the base class's private members.

`Esp32WifiDiagnostics`:

- **Connects to a configured IP**, the ESP32 on Mr Stabs Mk2 (`192.168.4.1`, the ESP32 access
  point default), on its own worker thread. The main loop never waits on the network, per the
  no-blocking rule for the perception loop.
- **Starts recording mode** with `GET /record/start`, so the firmware sends every control loop
  instead of at 10 Hz, then holds the server-sent-events stream at `/events` open. It sends
  `GET /record/stop` on shutdown and reconnects with backoff if the robot reboots or drops off.
- **Parses each event** (a CSV line from `DiagnosticsServer::update` in
  `firmware/mr_stabs_mk2/src/diagnostics_server.cpp`) into:
  - IMU: BNO055 orientation (x, y, z) and acceleration (x, y, z)
  - received commands: `a_percent`, `b_percent`, `flip_switch`, `armed`, `radio_connected`
  - output and PID values: `left_cmd`, `right_cmd`, `pid_setpoint`, `pid_output`
  - `timestamp_ms` (robot clock), `loop_us`, `is_upside_down`
- **Queues parsed events** for the subclass. In `update()` the subclass drains the queue and
  writes each event as its own message on a new `/robot/esp32_diagnostics` JSON topic, with the
  host receive time as `log_time` and the robot's `timestamp_ms` inside. One message per event,
  not a per-cycle `/diagnostics` summary, because the fit needs every firmware loop.
- **Reports health** through `DiagnosticsLogger` as `esp32_diagnostics`: connected, events per
  second, parse errors, reconnects, and the gap between robot and host clocks.

The app links no HTTP library (Crow and asio are only in `viz_relay`). The HTTP GET and the SSE
line reader are small enough for raw POSIX sockets, following `src/simulation/sim_connection.cpp`.

Config: `Esp32DiagnosticsOpenTxTransmitterConfiguration` extends
`OpenTxTransmitterConfiguration`, calling its hand-written `parse_fields` and adding
`esp32_host` (a free-form IP string, like other ids), `esp32_port` (80), `record_mode` (true),
and `reconnect_period_s`. Register it in `src/transmitter/config.cpp` beside `OpenTxTransmitter`,
with the factory branch next to the existing one. Add the topic's schema to
`include/foxglove_adapters/json_schemas.hpp` and the topic to `docs/foxglove_recording_format.md`.

Tests: an event-parsing test on recorded CSV lines, and a socket test against a local fake server
that serves `/events`, drops the connection mid-stream, and checks the reconnect.

### Addition 2: AprilTag robot keypoints

`AprilTagKeypointModel` implements `KeypointModelInterface`
(`include/keypoint_model/keypoint_model_interface.hpp`) and registers in
`src/keypoint_model/config.cpp` beside `YoloKeypointModel`. It does two things with each frame:

1. **Records the raw tag pose.** It detects the robot tags with OpenCV's aruco module and
   `DICT_APRILTAG_36h11`, the same detector setup `FiducialFieldFilter` uses for the floor array,
   with AprilTag corner refinement. For each detection it writes to a new `/apriltag/robot_tags`
   topic:
   - tag id, the four subpixel corners, the decision margin
   - both `SOLVEPNP_IPPE_SQUARE` solutions (rvec, tvec in the camera frame) and their reprojection
     errors
   - the frame's image timestamp

   The corners are the real measurement. The box records the video only as lossy H.264, so the
   corners are what lets PnP be re-solved offline with a revised tag size or intrinsics.
2. **Emits keypoints** so the rest of the pipeline runs unchanged. It projects the robot's front
   and back keypoint positions, placed from the tag pose and the tag-to-body transform, into the
   image and returns them labeled `MR_STABS_MK2` with `height_above_plane` set. The robot filter
   and `/robot_markers` then work as they do with the YOLO model, which gives a live view of the
   tracked pose and a comparison against the offline smoother.

Mr Stabs Mk2 carries two tags: 41 on top and 76 underneath, both tilted about 9.6 degrees. Take
their exact positions and orientations relative to the axle center from the Onshape URDF export
(the `apriltag_36h11_41` and `apriltag_36h11_76` links), not from the older Unity prefab. Seeing
tag 76 means the robot is upside down, which the firmware also handles.

Intrinsics: `update(RgbImage image)` receives no `CameraInfo`. Add it as a second parameter,
`update(RgbImage image, const CameraInfo &camera_info)`, on `KeypointModelInterface` and every
implementation, rather than a separate setter. The change reaches:

- `NoopKeypointModel` and `YoloKeypointModel`, which ignore the new parameter
- the runner's call (`keypoint_model_->update(camera_data.rgb)` becomes
  `update(camera_data.rgb, camera_data.camera_info)`)
- `ParallelModelBatch`, whose `update(const RgbImage &)` and `shared_image_` also carry the
  `CameraInfo` for the keypoint worker. The robot blob worker shares `worker_loop` but keeps its
  image-only call, so the keypoint lambda reads the stored camera info itself.
- the two fake keypoint models in `tests/perception_batch/test_parallel_model_batch.cpp`

Passing it per frame also keeps the model right if the camera ever changes resolution.

Cost: aruco detection over a full 1920x1200 frame could take tens of milliseconds on the Orin NX,
which would drop frames at 60 fps. Search a region around the last detection and fall back to the
full frame when the tag is lost. The spike step measures it.

Config: `tag_size_m`, `robot_tag_ids` (top and bottom), `roi_margin_px`,
`max_reprojection_error_px`, and the tag-to-body transforms. Tests: detection and PnP on a
rendered frame with a known tag pose, and IPPE solution selection on a tilted tag.

### Addition 3: the sysid profile

`config/mr_stabs_mk2_sysid_zed_box.toml`. The name has to match `[^_].*_(jetson|zed_box)$` in
`config/profiles.toml` to be selectable. It extends `mr_stabs_mk2_zed_box`, which keeps the ZED X
One S at 1920x1200 and 60 fps, the label mapping, and `[mcap] enable = true`, and overrides:

```toml
extends = "mr_stabs_mk2_zed_box"

[transmitter]
type = "Esp32DiagnosticsOpenTxTransmitter"
esp32_host = "192.168.4.1"
esp32_port = 80
record_mode = true

[keypoint_model]
type = "AprilTagKeypointModel"
tag_size_m = 0.08            # measure the printed tag
robot_tag_ids = [41, 76]

[field_filter]
# The test box, not the 2.35 m cage. Board offsets are where the array sits for field init.
field_size_x = 1.52
field_size_y = 1.52

[navigation]
type = "NoopNavigation"      # the driver drives; autonomy has nothing to do
```

### Fix: video on a mid-session recording start

`ZedOneRgbCamera::set_recording_enabled(true)` after initialization returns `encoder_.running()`
without starting the encoder (`src/rgbd_camera/zed_one_rgb_camera.cpp`). Starting a recording from
the remote UI after launch, which is step 3 of the procedure, therefore records no `/camera/video`.
Start the encoder there. The video is lossy and not used for the fit, but it is the only way to
review a session afterwards.

### What to drive (maneuver menu)

Each session is a few minutes. Steady holds are worth the most, so hold a stick position for at
least a second whenever the box allows.

| Maneuver | Identifies |
| --- | --- |
| Straight holds at a few throttle levels, forward and reverse, released to zero | gain, command-to-voltage map, decel (coast or brake) |
| Very slow creep, stick barely off center | deadzone, static friction |
| Spins in place at 3 to 4 stick levels, both directions | angular gain, skid and torsional friction |
| Arcs of several radii, figure eights | steer-brake coupling, per-side asymmetry |
| Direction reversals (forward to reverse, left spin to right) | delay versus lag, which smooth driving confounds |
| Smooth ramps up and down | the command-to-voltage curve between the hold levels |
| Throttle punches, getting sharper across the session, stopping at the first nose lift | reflected inertia, COM, traction |

Avoid:

- Wall contact. The 1.52 m box makes this the main constraint; keep runs short rather than
  touching the rails.
- Stick jitter near center. It makes the deadzone ambiguous.
- Full throttle plus full turn early in a session, before the punches have shown where the nose
  lifts.

The punches replace `find_flip_accel.py`'s ramp: the driver works up to the nose lift instead of
a script. The predicted threshold is 0.29 g. Stop at the first wheelie rather than the first
flip; a lifted nose already carries the information and a flip ends the session.

Volume: aim for 10 to 15 sessions over at least two battery charge levels. Record the pack voltage
before and after each session in a notes file next to the MCAP, with the floor condition and
anything unusual. Hold out whole sessions for validation, not windows from sessions used in
training.

## Robot pose from the recording

Offline, in Python, per session. The goal is a smooth but accurate 2D pose (x, y, yaw) of the
axle center in the field frame, plus pitch and roll for the nose-lift windows.

1. **Field frame.** Read the camera-to-field transform that field init produced from `/tf`
   (`diag_io.load_camera_in_field`). The floor marker array defines it, and it is fixed for the
   session because the tripod does not move. Check that it is constant across the recording.
2. **Tag pose in the field frame.** For each `/apriltag/robot_tags` message, map the IPPE
   solutions through the field transform.
3. **Resolve the IPPE ambiguity.** A small planar tag has two PnP solutions that can both fit the
   corners. Keep the one whose tag normal matches the expected tilt: about 9.6 degrees from field
   up for tag 41 on an upright robot. Break close calls by consistency with the neighboring frames.
   Log how often the choice was close.
4. **Body pose.** Apply the inverse tag-to-body transform from the URDF export to get the axle
   center's full 3D pose. The 2D pose is its (x, y, yaw); pitch and roll are kept alongside.
5. **Measurement noise.** Estimate the per-detection noise from segments where the robot is still.
   Scale it per detection by the reprojection error and the tag's size in pixels, so far-side and
   blurred detections count for less.
6. **Outlier gating.** Reject detections whose innovation against the forward filter exceeds a
   chi-square gate, and log each rejection.
7. **Smoothing.** A fixed-interval Rauch-Tung-Striebel smoother: a forward Kalman filter on
   (x, y, yaw, their rates and accelerations), then the backward pass. It is non-causal, so it
   smooths without the lag a causal filter adds, which matters because the fit reads the delay out
   of these poses. Yaw is unwrapped before filtering. Pitch and roll get their own smoother.
8. **Tuning the process noise.** Choose it by cross-validation: hold out every fifth detection,
   smooth the rest, and minimize the error on the held-out ones. Too little process noise rounds
   off the reversals and punches; too much passes the noise through. This picks the balance from
   the data instead of by eye.
9. **Optional IMU fusion.** After clock alignment (below), add the BNO055 yaw rate from
   `/robot/esp32_diagnostics` as a measurement of the yaw rate state. It sharpens yaw during fast
   spins and bridges tag dropouts. Keep it switchable, so its effect on the fit can be checked.
10. **Output.** A truth CSV per session: time, the smoothed state and its covariance, pitch and
    roll, and the raw measurement it came from, if any.

Checks on the result:

- On still segments, the scatter of raw poses around the smoothed pose sets the noise floor. The
  fit reports every error as a multiple of it.
- Residuals between raw and smoothed poses should look like white noise. Structure in them means
  the process noise is too low or the IPPE choice flipped.
- The smoothed yaw rate should match the BNO055 yaw rate on spins, which is an independent check
  when the IMU is not fused.
- Compare with the live `/robot_markers` track. Large differences point at the keypoint path or
  the robot filter, not the smoother.

The fit's loss is computed against the raw measurements, not the smoothed poses, so smoothing
cannot bias the fitted parameters. The smoothed state supplies each window's initial conditions.

## Data pipeline

1. Clock alignment: the app stamps everything on its own clock, including the host receive time of
   each ESP32 event. Map the robot's `timestamp_ms` onto it with a linear fit (offset and drift),
   then check it by cross-correlating the ESP32 `a_percent`/`b_percent` against the stick channels
   the transmitter logs, and the BNO055 yaw rate against the smoothed tag yaw rate. The
   stick-to-ESP32 offset is the radio link delay; report it, since the closed-loop sim needs it.
2. Command tape: `left_cmd` and `right_cmd` from `/robot/esp32_diagnostics`, divided by 100.
   These are the per-motor commands after the PID and the mixer, so the drivetrain fit is open
   loop even with auto-steer on. The remaining delay (ESC and mechanics) is not applied here; the
   fit shifts the tape per candidate.
3. Windows: reuse `plant.make_windows` with window lengths of 1 to 2 s. Gate out:
   - windows where tag detections cover less than a threshold of frames (start at 70%, tune on the
     noise floor)
   - windows within a robot length of the box rails
   - windows where pitch exceeds a few degrees, except in the separate nose-lift set
   - windows with gaps in the ESP32 stream
4. Initial state per window: the smoothed pose and velocity at the window start, wheel speeds from
   the no-slip relation (v +/- omega * 0.06526) / 0.025. The rotor state follows the wheels
   through the armature.

## MuJoCo model

`simulation/assets/robots/mr_stabs_mk2/` holds `robot.urdf` with older values (422.8 g base, 15 g
wheels, no rotor inertia). Replace it with an MJCF built from the table above.

- **Chassis**: free joint, `inertial` with the full tensor. Collision from a convex decomposition
  (CoACD) of the chassis collision mesh, plus a small explicit geom at the nose where the robot
  rests. The robot rests on the front skid, since the COM is ahead of the axle.
- **Wheels**: hinge joints on y, `armature = 4.77e-4` (the reflected rotor inertia), collision as
  a 25 mm cylinder the width of the tread.
- **Actuator**: a DC motor with back-EMF is affine in command and joint speed, so it fits MuJoCo's
  `general` actuator directly:

  ```
  torque = (eta N Kt V_batt / R) * u  -  (eta N^2 Kt Ke / R) * qvel
  ```

  with `gainprm[0]` the first coefficient and `biasprm[2]` the second. `forcerange` carries the
  current limit. `u` is the per-motor command (`left_cmd` or `right_cmd` / 100). Deadzone and the
  throttle curve are applied to the command tape before it becomes `ctrl`. Joint `frictionloss`
  and `damping` carry gearbox friction.
- **Firmware layer**, for closed-loop use only: a per-step function that takes the stick commands
  and the sim's yaw and reproduces `mix_motor_outputs` (the 1% turn threshold, 0.25 s cooldown,
  PID with its 2 degree deadband, and the 100% normalization) to produce `ctrl`. The fit does not
  use it, since the fit replays logged per-motor commands.
- **Zero-command behavior**: with AM32 complementary PWM and brake-on-stop settings, a zero
  command either shorts the motor (the back-EMF term stays active, which is braking) or lets it
  freewheel (the term drops out). Model both, pick by the recorded AM32 settings, and let the coast
  data confirm.
- **Floor**: a plane with fitted friction. Wheels use `condim` 4 or 6 so torsional and rolling
  friction exist; spins scrub through the skid, so its friction matters as much as the wheels'.
- **Timestep**: start at 1 ms and halve it until the flip threshold and coast tau stop changing.

Sanity checks before any fitting:

- The simulated nose lift under a torque ramp should match the analytic 0.29 g.
- Free speed at a set voltage should match KV x V / N.
- The coast tau should land near the measured 0.078 s with plausible R and braking settings. If no
  plausible R reaches it, the zero-command model is wrong.

## Parameters

Fixed from CAD, perturbed only in the sensitivity check: masses, inertias, COM, geometry.

Fitted, with priors:

| Parameter | Prior or bounds | Notes |
| --- | --- | --- |
| Drivetrain delay (firmware to motion) | 0 to 60 ms | profiled on a grid, as `jig_fit` does; the radio part of the 60 ms comes from clock alignment |
| Deadzone per motor | 0 to 0.06 | ESC stop threshold is set to 0.01; stage 2 says at or below 0.04 end to end |
| Command-to-voltage curve | linear plus one curvature term | settles the 5.6 m/s question |
| Pack voltage | measured per session, with a linear sag term | not fitted blind |
| Motor resistance R | 0.05 to 1 ohm | wide; no published value |
| Gearbox efficiency | 0.6 to 0.95 | |
| Wheel joint friction and damping | wide | |
| Left/right gain ratio | 0.8 to 1.2 | reverse spin shows asymmetry |
| Wheel-floor friction (sliding, torsional, rolling) | sled test when available, wide until then | |
| Skid-floor friction | wide | |
| Contact `solref` / `solimp` | MuJoCo defaults, then a narrow search | |
| Armature scale | 1.0, sigma 20% | tests the CAD rotor number |
| Zero-command mode | brake or coast, discrete | from AM32 settings |

About 15 continuous parameters plus two discrete choices.

## Fitting on MuJoCo Warp

- **Batch layout**: one world per (candidate, window) pair. A generation of 128 candidates over 64
  windows is 8,192 worlds. The spike in step 2 measures steps per second on one A6000 and sets the
  real sizes.
- **Per-world parameters**: MuJoCo Warp can vary some model fields per world for domain
  randomization. The spike confirms which of the fields above are supported (friction,
  `gainprm`, `biasprm`, armature, damping). Unsupported fields move to an outer loop over
  candidates.
- **Optimizer**: CMA-ES, gradient-free, so the delay grid and the discrete zero-command choice
  don't need special handling. Several restarts, to see whether they land in the same region.
- **Loss**: open-loop multi-step pose error as in `plant.predict_windows`: position and heading at
  0.2, 0.4 and 1.0 s horizons, Huber-weighted, normalized by the session noise floor. A pitch term
  applies only on the nose-lift windows.
- **Staged release**, so each group is fit on the data that identifies it:
  1. Straight holds and coasts: delay, deadzone, voltage curve, R, efficiency, braking mode.
  2. Spins and arcs: skid and torsional friction, left/right ratio.
  3. Nose-lift windows: armature scale and wheel traction, with everything else frozen.
  4. A final joint pass over all windows, starting from the staged result.
- **GPU queue**: fitting jobs go through `training/gpu_queue.py` like training. First submission
  uses `--profile mujoco-fit --eta 2h`, since the queue has no history for it; later ones pass
  `--work` as rollout-steps so the queue learns a rate.
- **Dependencies**: `mujoco` and `mujoco-warp` (with `warp-lang`) as x86_64-only entries in the
  platform-conditional deps in `pyproject.toml`. The Jetson never runs the fit. Pin versions; the
  MuJoCo Warp API is still moving.

## Validation

The model is accepted when all of these hold on held-out sessions:

1. **It beats or matches the grey-box plant.** Fit `plant.py` (M5) on the same training windows
   and compare position and heading error at 0.4 and 1.0 s. If MuJoCo loses on flat driving, the
   actuator model is wrong, since the rigid body adds physics and should not lose accuracy.
2. **Errors sit near the noise floor.** Report every error as a multiple of it.
3. **The physical numbers are plausible.** R, efficiency and friction inside physical ranges;
   armature scale near 1; radio plus drivetrain delay near 60 ms; coast tau near 0.078 s.
4. **Nose lift matches.** The fitted model lifts the nose on the same held-out punches that lifted
   the real robot, and not on the ones that didn't.
5. **The firmware layer matches.** Driven by the logged sticks and the fitted drivetrain in closed
   loop, the sim's `pid_output` and per-motor commands track the logged ones on straight holds.
   This is the check that the sim will behave like the robot once a controller drives it.
6. **The parameters are identified.** CMA-ES restarts agree. Report the parameter pairs that trade
   off (friction against armature against gain is the likely one) and fix one of each pair from an
   outside measurement if they do.

The spread of parameter sets that pass becomes the randomization range for the sim in
`learned_sim_environment_plan.md`, not a single best fit.

## Code layout

- C++, in the app:
  - `include/transmitter/esp32_diagnostics_opentx_transmitter.hpp` and `src/transmitter/`: the
    subclass and its config.
  - `include/esp32_diagnostics/` and `src/esp32_diagnostics/`: `Esp32WifiDiagnostics`, the socket
    client and the event parser.
  - `include/keypoint_model/apriltag_keypoint_model.hpp` and `src/keypoint_model/`: the AprilTag
    model and its config.
  - `config/mr_stabs_mk2_sysid_zed_box.toml`: the profile.
  - Tests beside the existing transmitter and keypoint model tests.
- Python:
  - `auto_battlebot/recording/`: readers for `/apriltag/robot_tags` and
    `/robot/esp32_diagnostics`, next to `mcap_io.py` and `diag_io.py`.
  - `auto_battlebot/perception/tag_pose_smoother.py`: IPPE selection, gating and the RTS
    smoother. Library code, type-checked.
  - `auto_battlebot/mujoco_sim/`: MJCF builder from the mass-property table, actuator and
    command-tape mapping, batched rollout, loss. Covered by the `auto_battlebot*` include in
    `pyproject.toml`.
  - `playground/calibration/smooth_tag_poses.py` and `playground/calibration/fit_mujoco_plant.py`:
    the CLIs.
- `simulation/assets/robots/mr_stabs_mk2/`: the MJCF, the convex collision pieces, and a
  `mass_properties.toml` recording the mass table with its provenance (Onshape screenshot date,
  URDF export, the rotor integration and its densities), plus the tag-to-body transforms.
- Recordings stay where the app writes them on the box and are copied to the dev machine. Truth
  CSVs and fit output go under `playground/calibration/out/`, like the existing plant fits. Not
  `runs/` (training output only) and not `data/`.
- The write-up in `docs/experiments/control_improvement/`.

## Steps

1. **Freeze the mass properties and tag transforms.** Commit `mass_properties.toml` with the
   tag-to-body transforms for tags 41 and 76, and the lumping script that produced it. Half a day.
2. **Model and MuJoCo Warp spike.** Build the MJCF, pass the three sanity checks, measure batched
   throughput, confirm which fields vary per world. One to two days. This decides the batch layout.
3. **App additions**, each with its tests: the ESP32 diagnostics component and transmitter
   subclass, the AprilTag keypoint model with the `CameraInfo` parameter on `update()`, the sysid
   profile, and the video
   encoder fix. Three to five days.
4. **Dry run on the box.** One short session through the full procedure, then check:
   - the ESP32 stream keeps up with the firmware loop over WiFi, with no gaps and no slowdown of
     the robot's own loop (`loop_us`)
   - AprilTag detection keeps the app at 60 fps with the robot at full speed, and covers the whole
     box floor from the tripod
   - field init locks with the array in the tilted view
   - remote control over ethernet still works with the box joined to `MR-STABS`
5. **Record.** Two or three evenings of driving to get 10 to 15 sessions.
6. **Pose smoothing, windows and noise floor.** The smoother, its cross-validated process noise,
   the checks above, and the gated windows. Two days.
7. **Grey-box baseline** on the same windows. Half a day; the code exists.
8. **MuJoCo fit**, staged. Two to three days including iteration.
9. **Validation and report.** One day.
10. **Hand-off**: the passing parameter spread goes into the learned-sim randomization ranges.

## Risks and open questions

- **Box size.** At 1.52 m, a robot that reaches 3 m/s crosses the box in half a second. Straight
  holds will be short, so the top of the command-to-voltage curve may still rest on few samples.
  The staged fit uses what the box allows, and a later session in the full cage can extend it.
- **Detection cost.** If aruco detection can't hold 60 fps even with a region of interest, the
  model can run on a worker thread and drop frames only for keypoint output, while the raw tag
  topic still gets every frame it processes. Frame drops show up as gaps, not bias.
- **IPPE flips.** A wrong solution choice on a small tilted tag shows up as a yaw or pitch jump.
  The expected-tilt test plus temporal consistency should catch it; the smoother residual check
  is the backstop.
- **Clock alignment.** ESP32 events are stamped on arrival, which includes WiFi jitter. The
  linear fit to `timestamp_ms` removes the jitter; the cross-correlation checks confirm the
  offset.
- **Closed-loop data.** A human driver reacts to the robot, so commands correlate with past
  disturbances. Open-loop multi-step scoring on the recorded tape limits the bias, but it is why the
  punches and reversals matter: they are the least reactive inputs in the menu.
- **ESP32 stream reliability.** The stream runs over the robot's own WiFi access point from the
  same ESP32 that runs the control loop. If recording mode drops events or slows the loop, fall
  back to the stick channels the transmitter logs with the radio in pass-through (100% rates, no
  expo, no trims, no slow-up) and reproduce the firmware layer in the fit, which makes the PID
  part of what has to be right.
- **Friction identifiability.** Without the sled test, wheel friction and armature can trade off on
  the punch data. The sled test fixes friction from outside.
- **BNO055 yaw.** The PID acts on the BNO055's fused heading, which has its own lag and drift. The
  firmware layer in the sim uses the sim's true yaw until the logged orientation shows how much
  those matter.
- **MuJoCo Warp maturity.** If per-world model fields don't cover what's needed, the fallback is an
  outer loop over candidates, which costs throughput but not correctness.
