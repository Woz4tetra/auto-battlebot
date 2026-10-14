# Running a sysid session

How to collect drivetrain sysid data for Mr Stabs Mk2 and watch coverage live. I drive by hand
the whole time. The tools only listen and never send drive commands.

Background: `docs/plans/sysid_coverage_monitor_plan.md` (the monitor) and
`docs/plans/mujoco_warp_mr_stabs_plan.md` (session procedure, maneuver menu). As of 2026-10-13 the
monitor has only run on simulated sessions, not against the robot.

## Hardware setup

1. Put Mr Stabs Mk2 in the 1.52 m drive-test box.
2. Mount the ZED X One S on the tripod about 0.75 m above the box floor, looking straight down.
   Connect it to the ZED Box Mini over GMSL2.
3. Plug the radio into the ZED Box over USB.
4. Connect the ZED Box to the laptop over ethernet (`ssh -i ~/.ssh/auto-battlebot-compute-2 user@192.168.50.183`).
5. The ZED Box's second WiFi dongle joins the robot's `MR-STABS` access point. The app polls the
   robot's telemetry at `192.168.4.1:80` (`esp32_host` in the profile).
6. Leave the robot's flip switch DOWN (auto-steer on), the competition setting.

## Start the app on the ZED Box

```bash
./scripts/build_and_run.sh -c config/mr_stabs_mk2_sysid_zed_box.toml
```

This profile runs the AprilTag keypoint model (robot tags 41 and 76), records the ESP32 telemetry
ring, and uses `NoopNavigation` with autonomy off, so the trainer switch cannot put zeros on the
sticks.

Then, from the dashboard:

1. Start recording (`/command/set_recording`).
2. Put the floor marker array (3x5 GridBoard, ids 160-174) in the box and press field init
   (`/command/reinit_field`). It locks after 10 frames.
3. Take the marker array out.

## Start the coverage monitor on the laptop

```bash
source scripts/activate_python.sh
python playground/calibration/sysid_coverage.py --live ws://192.168.50.183:8765
```

Open `http://localhost:8790`. The page shows:

- a confidence ring per plant parameter (full ring = pinned to its target sigma)
- suggested stick motions, ranked by expected ring gain per second
- coverage map, transition table, correlation matrix
- a floor view with 500 ms ghost predictions
- holdout error and the estimate trace over time
- a link panel: telemetry rate, tag rate, clock-fit residual, gate counts

Check the link panel first. If telemetry or tag rate is zero, fix the WiFi link or the camera
before driving.

## Drive

Drive for a few minutes. There is no start button: the monitor picks up motions whenever they
happen. Use the suggestions to fill the rings that lag. Suggestions that need more floor than the
box has are marked floor-limited; skip those.

When done:

1. Stop the recording from the dashboard.
2. Ctrl-C the monitor. It writes the session to `~/sysid_sessions/<stamp>/`:
   - `plant.toml`: the live plant estimate
   - `coverage.json`: the last snapshot
   - `events.json`: the event log

The live estimate is a progress gauge. The calibration that ships is the offline fit on the MCAP.

## Check the recordings

Copy the MCAP off the ZED Box once the app has closed it, then:

```bash
python playground/calibration/check_sysid_recordings.py data/recordings/*2026-10-13*.mcap
```

Each recording gets PASS / WARN / FAIL for file integrity, camera rate, field stability, tag
coverage, the ESP32 stream, and the maneuver menu. Exit status is 1 on any FAIL. Drive again if a
recording fails.

## Replay without the robot

```bash
# a recorded session at 8x real time
python playground/calibration/sysid_coverage.py --replay rec.mcap --speed 8

# as fast as possible, no page
python playground/calibration/sysid_coverage.py --replay rec.mcap --speed 0 --no-serve

# write a simulated session, then replay it
python playground/calibration/sysid_coverage.py --synthetic /tmp/sysid_sim.mcap --seconds 150
```

The replay page stays up after the recording ends; Ctrl-C writes the session and exits.

Drives recorded on a match profile have no tags. Replay their video through
`config/playback/mr_stabs_mk2_sysid_playback.toml` first, one recording at a time.

## Troubleshooting

- Tag size: `tag_size_m = 0.064` is the CAD value. Measure the printed tag and correct it in both
  `config/mr_stabs_mk2_sysid_zed_box.toml` and the playback profile before the first session.
- Field shifted mid-session: the checker flags more than 5 mm or 0.5 deg of field wander. That
  means a bumped tripod or a re-init. Re-init the field and drive again.
- Truncated MCAP: a missing summary means it was copied while still recording. Copy it again.
