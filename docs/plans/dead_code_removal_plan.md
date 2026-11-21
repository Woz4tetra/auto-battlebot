# Dead code removal

Delete the components that no robot runs and finish retiring the `_jetson` base. When this plan is done, no C++ implementation is reached only by tests or by retired profiles.

The stereo camera path stays for the handheld ZED 2i in case it comes back: `ZedRgbdCamera`, `SvoRecorder`, `ZedSvoPlaybackCamera`, `PointCloudFieldFilter`, `HeightGate`, and the depth fields in `data_structures/`. `mr_stabs_mk2_desktop`, `playback/_playback.toml` and `experiments/mrs_buff_mk3_label_playback` keep reaching them.

## Profiles after the reorg

Commit `238a7b0a` cut the robot profiles down to five, counting `mr_stabs_mk2_sysid_zed_box`, which was restored after it. I resolved each with `./build/auto_battlebot -c <file> --print-config` on 2026-11-20.

| Profile | Field filter | Nav | Transmitter / drive | Plant |
| --- | --- | --- | --- | --- |
| `mrs_buff_mk3_zed_box` | Homography 2.35 m | RlPolicy `rl_nav_mk3_c_near` | OpenTx / Tank | `robots/mrs_buff_mk3` |
| `mrs_buff_mk3_aer_zed_box` | Homography 1.5 m | RlPolicy `rl_nav_mk3_c_near` | OpenTx / Tank | `robots/mrs_buff_mk3` |
| `mr_stabs_mk2_aer_zed_box` | Homography 1.5 m | RlPolicy `rl_nav_p4` | EspNow / Differential | `robots/mr_stabs_mk2_geometry` |
| `mr_stabs_mk2_fid_zed_box` | Fiducial 1.5 m | RlPolicy `rl_nav_p4` | EspNow / Differential | `robots/mr_stabs_mk2_geometry` |
| `mr_stabs_mk2_sysid_zed_box` | Fiducial 1.52 m, AprilTag keypoints | Noop, autonomy off | EspNow / Differential | `robots/mr_stabs_mk2_geometry` |

All five use `ZedOneRgbCamera`, DeepLab, YOLO keypoints (AprilTag on sysid), `RobotFrontBackFilter` with `KalmanMotionEstimator`, the static gate, and no battery. `mr_stabs_mk2_desktop` now inherits `MotionProfileNavigation` from `_common` at `max_linear_command = 0.8`.

The reorg left two things broken at `238a7b0a`. Step 1 fixes both:

- `config/default_profile` still names `mrs_buff_mk3_jetson`, which is gone. A box with no saved selection, or a saved selection naming a deleted profile, fails at startup with `Default profile 'mrs_buff_mk3_jetson' not found`.
- `tests/test_profile_pattern.cpp` `OffersExactlyTheDeployableProfiles` still expects the eight old profiles, so it fails.

## What goes

| Component | Lines (src + header) | Reached today by | Step |
| --- | --- | --- | --- |
| `_jetson.toml`, `jetson` in the profile pattern | 1 config | Nothing extends it | 1 |
| `CalibratedFieldFilter`, C++ `cage_calibration` | 254 | Nothing, not even a test of the filter | 2 |
| `FixedVelocityNavigation` | 52 | Nothing | 2 |
| `Esp32DiagnosticsOpenTxTransmitter` | 142 | `tests/test_config.cpp` only | 2 |
| `CompositeTransmitter` | 139 | `tests/test_espnow_transmitter.cpp` only | 2 |
| ESP32 WiFi telemetry (`src/esp32_diagnostics/`, `include/esp32_diagnostics/`, `tests/esp32_diagnostics/`) | 1,548 with tests | `Esp32DiagnosticsOpenTxTransmitter` only | 2 |
| `YoloBboxRobotBlobModel`, the blob merge in `RobotFrontBackFilter` | ~580 | Nothing: every config sets `NoopRobotBlobModel` | `measurement_chain_simplification_plan.md` step 1 |
| `DeadReckoningMotionEstimator`, `RobotTemporalMotionFilter` | 183 | `config/playback/mr_stabs_mk2_playback.toml` | `measurement_chain_simplification_plan.md` step 2 |
| `PursuitNavigation` | 388 | `playback/mr_stabs_mk2_playback`, `simulation/kinematic_sim`, `simulation/mr_stabs_mk2_kinematic_sim`, `simulation/mr_stabs_mk2_kinematic_sim_sensing` | 3 |
| Battery backend (`src/battery/`, `include/battery/`, `BatterySource`) | 802 | `_jetson.toml` only, which no profile extends | 4 |

The blob path and the dead-reckoning estimator are already steps 1 and 2 of `docs/plans/measurement_chain_simplification_plan.md`, so this plan only orders around them.

What stays:
- `MotionProfileNavigation`. No robot profile runs it after the reorg, but `mr_stabs_mk2_desktop`, every playback profile but one, and the MotionProfile sims do.
- ESP-NOW telemetry. `EspNowTransmitter` decodes `espnow_protocol::TelemetryPayload` itself and writes `/robot/espnow_telemetry`, without touching `esp32_diagnostics/`.
- The Python readers of `/robot/esp32_diagnostics` (`auto_battlebot/calibration/sysid/telemetry.py`, `auto_battlebot/recording/sysid_io.py`, `mcap_write.py`). They read recordings made before this plan, so old sysid sessions stay usable.
- The firmware's `GET /telemetry` endpoint. This plan touches no firmware.
- The robot's own battery voltage from CRSF telemetry (`CrsfBattery` in `OpenTxTransmitter`, `vbat` on the sysid page). That is the robot pack, not the host UPS.
- `auto_battlebot/perception/cage_calibration.py` and `config/cages/`. `playground/cage_scene/` and `tests/python/test_cage_calibration.py` use the Python loader.
- `PursuitPolicy` in `auto_battlebot/rl/policies/scripted.py` and the `pursuit` opponent behaviour in `auto_battlebot/rl/envs/`. Both are training-side opponents, not the C++ navigator.

## Order

```
1. profile cleanup (fixes HEAD)    measurement chain step 1 (blob path)
2. already-dead types              measurement chain step 2 (DR estimator)
         │                                   │
         └───────────────┬───────────────────┘
                         ▼
            3. PursuitNavigation   (needs MC step 2)
            4. Battery             (needs step 1)
```

Step 1 goes first because HEAD has a failing test and a missing default profile. Step 3 waits for measurement chain step 2 because both change `mr_stabs_mk2_playback.toml`. Switching the estimator first and the navigator second keeps each replay diff attributable to one change.

## Steps

Each step is one commit. Before each one is called done, run:

```bash
./scripts/build.sh                 # release build, also catches NDEBUG-only breaks
./scripts/build_and_test.sh        # debug + GoogleTest
./scripts/lint
```

### 1. Finish the profile reorg

- `config/default_profile`: `mrs_buff_mk3_zed_box`, the renamed `mrs_buff_mk3_jetson`
- Delete `config/_jetson.toml`
- `config/profiles.toml`: pattern becomes `[^_].*_zed_box$`
- `tests/test_profile_pattern.cpp`: the expected list becomes the five profiles above. Drop `_jetson` from the excluded list
- `tests/test_config.cpp` and `tests/python/test_deploy_profile_list_matches_app.py` write their own synthetic `*_jetson` and `*_desktop` files and need no change
- `auto_battlebot/recording/video_import.py`: `active_profile` default `"_jetson"` becomes `"_zed_box"`
- Stale names of deleted profiles:
  - `mrs_buff_mk3_fid_rl_zed_box`, `mr_stabs_mk2_fid_rl_zed_box`: `docs/plans/nav_scoring_and_rl_v2_plan.md`, `docs/reference/config/profiles.md`
  - `mr_stabs_mk2_jetson`: `docs/reference/config/config_extends.md` (example chain), `profiles.md`, `robot_plant_files.md`
- Other docs: the `_jetson` base row in `profiles.md`, `training/deeplab/README.md`, the config tree in `.claude/skills/replay/SKILL.md`

The handheld (`handheld-autonomy`, ZED 2i) has no profile after the reorg. A deploy there selects a ZED Box profile, and `ZedOneRgbCamera` cannot open a ZED 2i. Don't deploy to the handheld. If it comes back, it needs a `_handheld` base built from what `_jetson.toml` had, minus the battery.

### 2. Delete the already-dead types

- `CalibratedFieldFilter`: `src/field_filter/calibrated_field_filter.cpp`, its header, its `REGISTER_CONFIG` row and configuration struct in `include/field_filter/config.hpp`, `tests/field_filter/test_calibrated_field_filter.cpp`
- C++ `cage_calibration`: `src/field_filter/cage_calibration.cpp`, its header, `tests/field_filter/test_cage_calibration.cpp`. Its only consumer is `CalibratedFieldFilter`
- `FixedVelocityNavigation`: `include/navigation/fixed_velocity_navigation.hpp`, its registration and configuration struct
- `Esp32DiagnosticsOpenTxTransmitter`: source, header, registration, configuration struct, its cases in `tests/test_config.cpp`
- `CompositeTransmitter`: source, header, registration, `kCompositeToml` and `CompositeTransmitterTest` in `tests/test_espnow_transmitter.cpp`
- ESP32 WiFi telemetry, in the same commit as the transmitter that is its only caller:
  - `src/esp32_diagnostics/` and `include/esp32_diagnostics/`: `Esp32WifiTelemetry`, `Esp32TelemetryBatch`, the event decoder and its CSV and JSON writers
  - `tests/esp32_diagnostics/`, including `FakeRingServer`
  - `tests/python/test_mr_stabs_web_mirrors.py`: it reads `esp32_diagnostics_event.hpp` at import. Delete `test_events_csv_header_ends_with_seq_and_matches_jetson_parser_width`, which checks the firmware page against the C++ parser. Drop the module-level `DIAG_EVENT_HPP` read, and keep the tests that only compare the firmware page with firmware sources
- Docs:
  - their rows in `docs/reference/config/fields.md`
  - `docs/reference/robot_link/esp32_wifi_telemetry.md`: delete the C++ sections (event decoding, batch header, workarounds, test fake server). Move the firmware ring and USB bridge facts to the firmware's robot-link doc if they still describe the firmware, then delete the file and its entry in `docs/README.md` and `docs/plans/workaround_ledger.md`
  - `docs/reference/recording/foxglove_recording_format.md`: mark `/robot/esp32_diagnostics` as written only by recordings before this change, and drop the `Esp32WifiTelemetry` producer description
  - `docs/reference/calibration/sysid_modules.md`: the live `Esp32WifiTelemetry` source, if it names one
  - `docs/plans/protobuf_messaging_plan.md`: `/robot/esp32_diagnostics` loses both of its producers (`esp32_wifi_telemetry.cpp` and the transmitter), so drop it from the message inventory and the `TELEMETRY_TRANSPORT_WIFI` enum value
  - `docs/plans/motor_stall_protection_plan.md`: the bullet about updating `src/esp32_diagnostics/esp32_telemetry_batch.cpp` and its test fixture when `TelemetryPayload` grows. Only the `tests/test_espnow_transmitter.cpp` fixture is left
  - the `CalibratedFieldFilter` sections of `docs/reference/perception/cage_calibration.md` and `camera_calibrations.md`. The Python format description stays
  - the `Esp32DiagnosticsOpenTxTransmitter` row in `docs/plans/hardware_checks.md`, since nothing is left to check
  - the `FixedVelocityNavigation` row and step mention in `docs/plans/command_units_plan.md`

### 3. Delete `PursuitNavigation`

Switch the remaining pursuit profiles to `MotionProfileNavigation` (`from = "navigation/motion_profile"`), then delete the class.

| Profile | Change |
| --- | --- |
| `config/playback/mr_stabs_mk2_playback.toml` | Replace the `[navigation]` table. Replay before and after |
| `config/simulation/kinematic_sim.toml` | Replace the `[navigation]` table |
| `config/simulation/mr_stabs_mk2_kinematic_sim.toml` | Replace the `[navigation]` table. It is then the same as `mr_stabs_mk2_motion_profile_sim` |
| `config/simulation/mr_stabs_mk2_motion_profile_sim.toml` | Delete. Point `playground/calibration/sim_stop_test.py:28` at `simulation/mr_stabs_mk2_kinematic_sim` |
| `config/simulation/mr_stabs_mk2_kinematic_sim_sensing.toml` | Replace the `[navigation]` table |

Then:
- Delete `src/navigation/pursuit_navigation.cpp`, its header, its registration and configuration struct, `tests/navigation/test_pursuit_navigation.cpp`
- `playground/rl/lockstep_eval.py`: remove `pursuit` from `--nav`. Its overlay relies on the base profile being pursuit, and none is left. The `--pursuit-grid` scenario export is an opponent behaviour and stays
- `auto_battlebot/recording/diag_io.py`: keep `PURSUIT_NAV_HW_ID` in `NAV_HW_IDS`, so older recordings still score
- `.claude/skills/eval/SKILL.md`: the `NAV_HW_IDS` gotcha names PursuitNavigation as a live implementation
- Docs:
  - `docs/reference/config/fields.md`, `profiles.md`
  - `docs/reference/control/hazards.md`, `plant_backed_control.md`, `rl_opponents.md`, `rl_toy_configs.md`: drop the C++ pursuit navigator. Keep the pursuit opponent
  - `docs/plans/command_units_plan.md`: the pursuit row, the pursuit retune decision, and pursuit in step 3 and step 5
  - `docs/plans/nav_scoring_and_rl_v2_plan.md:233`: "MotionProfile, or pursuit" becomes MotionProfile

Checks:
- Replay one Mr Stabs SVO on `playback/mr_stabs_mk2_playback` before and after, and compare the transmitted sticks and the hazard incursions.
- Run `./scripts/run_simulation.sh simulation/sim_mr_stabs_mk2.toml` and `sim_stop_test.py` on the new `mr_stabs_mk2_kinematic_sim`.

### 4. Delete the host battery monitor backend

The battery comes back through an OpenUPS reader, so the topic and the front end stay: `BatteryMessage`, the `/status/battery` row in `include/remote/protocol.hpp`, the web battery display, and the `Battery` message docs. Only the Waveshare backend goes.

- `src/battery/`, `include/battery/`, `include/enums/battery_source.hpp` and its line in `include/enums.hpp`
- `BatteryConfiguration battery` in `include/config/config.hpp`, `load_battery_from_toml` in `src/config/config.cpp`, `make_battery_monitor` in `src/main.cpp`
- `tests/battery/`
- Docs:
  - the `[battery]` section in `docs/reference/config/fields.md`
  - the `BatteryMonitor` pacing note in `docs/reference/recording/foxglove_recording_format.md`
  - the battery clause of the `ScopedSignalQuit` check in `docs/plans/hardware_checks.md`

Nothing publishes `/status/battery` until the OpenUPS backend lands, so the web page hides the battery cell. The robot-pack voltage from CRSF telemetry stays.

## Next steps

1. Land step 1 now. HEAD fails `ProfilePattern.OffersExactlyTheDeployableProfiles` and has no valid default profile until it does.
2. Land step 2.
3. Land measurement chain steps 1 and 2, then steps 3 and 4 here.
