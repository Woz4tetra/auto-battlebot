# Workaround ledger

Every workaround the phase 1 comment audit found, with its root-cause fix where one is known. Fix
rows one at a time: the root cause goes, the workaround goes with it, and the row is deleted.

As of 2026-11-20 no comment defends any of these in code. Each reason now sits in a "Workarounds"
section of the subsystem's reference or setup doc. Where a batch extracted the workaround, the
function or constant name says what it guards. File line numbers below are from the 2026-11-15
audit and have moved since; search for the symbol. Built from the 322 `HACK` rows of the audit CSV,
deleted 2026-11-20 and still in git history at `docs/plans/comment_purge_audit.csv`.

## Status 2026-11-20

Batches retired every row that could be settled on the dev box: deleted, fixed, or pinned by a test that fails with the workaround removed. Checks that need a device moved to `docs/plans/hardware_checks.md` (section "Ledger rows that need a device"). What is left:

- "Blocked on a decision or hardware": rows waiting on a decision, a missing asset, or a tool
- `espressif32@6.13.0` platform pins: deferred to Mk4

## How to read it

- **Verified?** starts with `yes:`, `no:`, or `unverified:`. Auditors read code paths and installed
  library sources. They did not build C++, run the app, flash firmware, or touch hardware, so most
  `unverified` rows need one of those to settle.
- 124 rows are verified, 10 failed verification, 186 are unverified, and 2 carry no tag. 83 rows
  have no known root-cause fix.
- By area: `training/` 76, `firmware/` 39, `src/` 31, `tests/` 30, `hardware/` 27, `scripts/` +
  `install/` + `service/` + `web/` + `CMakeLists.txt` + `mypy.ini` + `docker/` 59, `playground/` +
  `simulation/` + `logo/` 31, `auto_battlebot/` 16, `include/` 13.

## Claims that failed verification

Phase 5 pilot, 2026-11-19. Each row got one confirming run.

| Location | Outcome |
| --- | --- |
| `training/deeplab/semantic_train.py:51` | Fixed: `seed_worker` deleted. torch 2.10 seeds Python `random` per worker; a 2-worker DataLoader drew 8 distinct values |
| `web/src/lib/image.ts:65` | Fixed: `CompressedMessage.data` is typed `Uint8Array<ArrayBuffer>`, so the Blob takes it without the `slice()` copy; `npm run check` passes |
| `mypy.ini:63` | Fixed: `[mypy-nhrl_common]` and `[mypy-nhrl_audit]` deleted; mypy passes (`mypy_path` itself was removed later) |
| `install/install_python_environment.sh:229`, `mypy.ini:17`, `mypy.ini:108`, `mypy.ini:112`, `tests/python/test_gpu_queue.py:18` | Fixed: the shared modules moved into `auto_battlebot/`; the `.pth` writer, `mypy_path` and both `ignore_missing_imports` sections are gone, and the test imports `auto_battlebot.gpu_queue` normally |
| `firmware/espnow_dongle/scripts/upload:2` | Fixed: defaults to `/dev/espnow_dongle`. Before, with both plugged in, it flashed the robot |
| `firmware/espnow_dongle/platformio.ini:25`, `firmware/mr_stabs_mk2/platformio.ini:30` | Claim wrong, overrides kept: the board file sets `use_1200bps_touch` and `wait_for_upload_port` true and `build_flags` never reach upload settings. Reasons in `docs/reference/robot_link/firmware_build.md` |

## Repeated workarounds

One root-cause fix retires several rows each.

- **ESP32 platform pin.** `espressif32@6.13.0` in `firmware/mr_stabs_mk2/platformio.ini:15`,
  `firmware/espnow_dongle/platformio.ini:8`, and the 250-byte frame limit in
  `espnow_protocol.h:22`, plus `DShot.cpp:8` writing into NeoPixel's private
  `rmt_reserved_channels[]`. Fix: port DShot to the IDF 5 `rmt_tx` driver.
- **libnvjpeg symbol clash.** Fixed: the dlopen plugin is the fix, and the comments went once a
  test guarded it. CTest `NoNvjpegLink` runs `scripts/check_no_nvjpeg_link.sh` on the direct
  `NEEDED` entries of `auto_battlebot_lib` and `auto_battlebot_test`, and a POST_BUILD step runs it
  on `auto_battlebot` and `viz_relay`. Any `libnvjpeg` or `libauto_battlebot_nvjpg` entry fails.
  Reasons are in `docs/reference/recording/foxglove_recording_format.md` ("NVJPG plugin").
- **Eigen `FromTwoVectors` `-Wmaybe-uninitialized` on GCC 13.** Fixed: the four sites call
  `rotation_between()` in `transform_utils`. It builds the quaternion directly with a
  fixed antiparallel axis, so no SVD runs and no `#pragma` is needed. Unit tests cover it.
- **ZED SDK 5.2 quirks.** `src/rgbd_camera/zed_one_rgb_camera.cpp` has five, including the
  setting nudge at `:394` and `_Exit` on a wedged `grab()` at `:51`. Most may go with an SDK upgrade.
- **BlenderProc segmentation pass indices.** Fixed: `synthgen/pass_index.py` `assign_pass_index()`
  runs as each mesh is created, and the distractor re-arm is gone. The 9 "must be first import"
  comments are gone too: BlenderProc enforces it (`SetupUtility.py:389`) and `pyproject.toml`
  keeps the order. Reasons in `docs/reference/tools/synthgen_render.md`.
- **ESP DTR/RTS reset.** Fixed: `auto_battlebot/robot_link/esp32_serial.py` `open_esp32_serial()`
  serves all three scripts (DTR and RTS high before `open()`); the reason is in
  `docs/reference/robot_link/espnow_link.md`. The `usb_dashboard.py` variant was not deliberate.
- **Camera retry and cancel.** Fixed: no camera's `initialize()` clears `cancel_open_` any more
  (it is set at construction only, and `cancel_initialize()` is terminal), and `Runner` no longer
  drains commands before the retry open.
- **`sys.path` routes around the package rule.** Fixed 2026-11-19. `gpu_queue` moved to `auto_battlebot/gpu_queue.py`, `seg_to_bbox` and `clear_image_cache` to `auto_battlebot/datasets/`, `gen_splash_animation` to `auto_battlebot/branding/splash_animation.py`, and the kinematic sim (`camera_utils`, `hazards`, `kinematic_sim_server`, `mujoco_window`, `policy_opponent`, `protocol`, `rl_eval_server`, `viewer`, `config/`, `plants/`) to `auto_battlebot/kinematic_sim/`. The old paths under `training/`, `logo/` and `simulation/` are thin entry points. The `auto_battlebot.pth` writer, the `mypy_path` line, the `seg_to_bbox` and `clear_image_cache` mypy sections, the test `sys.path` inserts and the path loaders are gone. Left alone: Blender-run scripts under `training/synthetic/` still import each other by top-level name, and `tests/python/conftest.py` `load_training` still loads `training/` CLI scripts by path.

## Bugs found during the audit

Defects in code, not workarounds.

Fixed 2026-11-19:

- `scripts/run_simulation.sh`: the fallback assigned the same missing per-robot config; it now
  falls back to `config/simulation/kinematic_sim.toml` (`86c44dab`).
- `firmware/velocity_jig/scripts/console.py` and `firmware/velocity_jig/README.md`: IMU scale now
  0.140 dps and 0.000488 g per LSB, matching `include/config.h`; a test pins the console to the
  header (`86c44dab`).
- `src/field_filter/point_cloud_field_filter.cpp`: the degenerate-plane fallback now returns
  (0, 0, 1), the same z >= 0 convention as the flip and `field_pose.cpp`, with tests (`75c48602`).
- `include/tensorrt_inference/trt_engine.hpp`: `ScopedQuietLogging` nesting is `thread_local`, so
  probing on one thread no longer demotes errors on others (`86c44dab`).
- `auto_battlebot/perception/trt_yolo.py`: `preprocess_frame` requires `letterbox_padding`
  (`86c44dab`).
- `hardware/cage_pack/camera_boot.py`: deleted (`86c44dab`).
- `CMakeLists.txt`: the ZED debug override passes `-U_DEBUG` instead of `-Wno-extra -Wno-error`;
  those sources build under `-Werror` (`86c44dab`).
- Missing docs: the test citation of `docs/robot_filter_decay_plan.md` was already gone; the stage0
  report now points at `our_robot_hold_window_s` in `docs/reference/config/profiles.md`. No
  remaining reference to `docs/velocity_jig_wiring.md` or `docs/hardware/README.md`.

Fixed 2026-11-19, found during phase 2:

- `src/transmitter/espnow_transmitter.cpp`: enabling autonomy while SA is already on now ramps
  from standstill; the first command after `enable()` used to skip the slew limit. Test
  `EspNowTransmitterPtyTest.ReEnablingWithSaOnRampsFromStandstill`.
- `include/robot_filter/ekf_math.hpp` `wrap_angle`: returns `(-pi, pi]` as documented; `-pi` maps
  to `pi`.
- `src/mask_model/deeplab_mask_model.cpp`: feeds RGB with the training corpus statistics. The
  audit found the ImageNet normalization; the eval found the bigger defect, 3-channel BGR frames
  passed to the network unswapped. Macro IoU 0.9219 to 0.9440
  (`docs/experiments/perception/2026-11-19_deeplab_preprocessing_fix.md`).
- `playground/opponent_prediction/run_replays.py`: start frames come from
  `config/playback/svo_start_frames.toml` through `auto_battlebot.recording.svo_start_frames`, so
  14-12-27 replays from 3000, not the script's own 1540. `make_eval_dataset.py` uses the same loader.
- `scripts/nhrl_to_recordings.py`: a clip skipped as already converted now advances the start
  stamp, so a re-run gives new clips the stamps a full run would. The one converted set on disk,
  `BrettZone_2026-05` (6 clips), had no overlapping stamps.
- `scripts/convert_ros1_mcap.py`: `--check` now flags a `/diagnostics` array whose log time
  reaches no per-module topic, so a dropped array fails the check; before, only a module with more
  messages than the source failed it.
- `src/mcap_recorder/mcap_recorder.cpp`: the recorder takes its output directory from
  `McapRecorderConfig::recordings_dir` (default `data/recordings`), and every recorder test writes
  to a per-test temp dir. Before, `HoldingDirectoryScanSurvivesAnUnreadableEntry` evicted real
  recordings (about 283 test MCAPs and 10 replay MCAPs on 2026-11-19) and the toggle tests left
  `toggle_*_test` files there. Test `McapRecorderToggleTest.EvictsOnlyFromItsOwnDirectoryAndNeverFromAnother`
  keeps a 1.1 GB sentinel outside the recorder's directory.
- `src/robot_filter/front_back_keypoint_converter.cpp` `get_pose_from_points`: the pose is a
  rotation about z by the front-back heading. It was the 3D shortest arc from +x, which for a
  robot facing near -x with the keypoints at different heights came out upside down (roll pi,
  yaw still right). Every consumer reads yaw only, so filtered poses do not change. Test
  `FrontBackKeypointConverterTest.PoseStaysUprightWhenTheKeypointsDifferInHeight`.

| Location | What the code does | What the comment claims would break | Verified? | Root-cause fix |
| --- | --- | --- | --- | --- |
| `firmware/espnow_dongle/platformio.ini:8` | Pins platform = espressif32@6.13.0 (Arduino core on IDF 4.4). | An unpinned espressif32 resolves to the globally installed pioarduino 55.x (IDF 5), which changes ESP-NOW frame limits the wire format assumes. | unverified: matches project memory note pioarduino-shadows-espressif32; did not build unpinned. | Port to IDF 5 (ESP-NOW v2 frames, rmt_tx DShot on mr_stabs_mk2) and pin to that platform explicitly. |
| `firmware/mr_stabs_mk2/platformio.ini:15` | Pins platform = espressif32@6.13.0 (Arduino core 2.x, IDF 4.4). | pioarduino 55.x (IDF 5) removes NeoPixel's rmt_reserved_channels and deprecates the legacy RMT driver DShot.cpp uses. | yes: DShot.cpp:10,62 use rmt_reserved_channels from .pio/libdeps/local/Adafruit NeoPixel/esp.c:154 and the legacy rmt_config/rmt_write_items API; did not build on IDF 5. | Port DShot to IDF 5 rmt_tx/rmt_rx with its channel allocator. |

## TEST rows deferred

Rows from the `TEST` verdict batches that could not get a test in a headless build. The comment is
gone and the claim is in a reference doc; the test is still owed.

| Location | What the code does | What the comment claims would break | Verified? | Root-cause fix |
| --- | --- | --- | --- | --- |

## Rows that could not be tested (phase 2, 2026-11-19)

The comment is gone and the claim is in a reference doc. The test is owed once the seam, hardware or
harness named in the last column exists.

| Location | Claim | What blocks a test |
| --- | --- | --- |

## Blocked on a decision or hardware

Rows a batch could not settle. Each needs the action in the last column first.

| Location | Needs |
| --- | --- |
| `auto_battlebot/mujoco_sim/onshape_export.py:1` | Decide whether purchased parts get real mass in the Onshape part studio, or export mass per part through the Onshape API; needs the Onshape exporter |
| `include/mask_model/fixed_mask_model.hpp:8` | Replace `FixedMaskModel` with `FieldFilterInterface::requires_mask()` (false for `FixedFieldFilter`), gate the Runner on it, switch the three sim TOMLs to `NoopMaskModel`. Needs `docs/reference/simulation/kinematic_sim_config.md`, which has uncommitted edits |
| `include/rl_policy/policy_interface.hpp:14` | Decide whether navigation outputs a `NormalizedCommand` type; it changes every navigation implementation |
| `include/robot_descriptions_cache.hpp:9` (`Resolved` lifetime) | Deleted with `RobotDescriptionsCache` in step 3 of `docs/plans/measurement_chain_simplification_plan.md` |
| `tests/python/test_tag_pose_smoother.py:578` | Kept: retune declined 2026-11-20 |
| `training/model_eval/score_size_ladder.sh:97-98` | Needs the `cage_high_x50_conf044/d40000` eval set (not in the main checkout), or confirmation that `nhrl_cage_high_eval` replaces it; that set has `reviewed: []` |
| `hardware/cage_pack/enclosure.py:221` | The failure is in Onshape, not OCC: with the constant at 0 the build123d enclosure is valid. Settle with one Onshape push of the enclosure at 0 (costs Onshape API calls from the yearly budget) |
| `hardware/cage_pack/enclosure.py:295` | The failure is in Onshape, not OCC: with the constant at 0 the build123d enclosure is valid. Settle with one Onshape push of the enclosure at 0 (costs Onshape API calls from the yearly budget) |
| `src/navigation/rl_policy_navigation.cpp:189` | Zero-command path is tested; the coasted spin and motion-blur claim needs a recording. Decide between decaying coasted yaw rate in the robot filter and training the policy on observation age |
| `hardware/zed_box_mini_mount/assembly_page.py:62` | Needs a test of GLB handling on the artifact host, which means an upload; decide whether to allow one |
| `tests/test_kalman_motion_estimator.cpp` (`make_our_pose_measurement`, `make_opponent_pose_measurement`) | Needs a `heading_measured` field on `RobotDescription` (today inferred from `keypoints.empty()` in `kalman_motion_estimator.cpp`), touching the tracker, estimator and scene adapter. A perception behaviour change: decide whether to make it |
| `include/mcap_recorder/mcap_recorder.hpp` (start_file_locked, mutex_) | Clang `-Wthread-safety` rejects `std::mutex` under libstdc++ as a non-capability type. Decide whether to add an annotated Mutex wrapper and lock guard to these classes |
| `include/rgbd_camera/video_encoder.hpp`, `zed_rgbd_camera.hpp` (lock discipline) | Clang `-Wthread-safety` rejects `std::mutex` under libstdc++ as a non-capability type. Decide whether to add an annotated Mutex wrapper and lock guard to these classes |

## Follow-up: purge the relocated comments from docs

Do this after the hacks above are fixed or pinned by tests. Until then the docs below are the only
record of why each workaround exists, and the ledger rows point at them.

The purge moved comments into docs instead of deleting them. A 2026-11-20 sweep classified the 145
docs the purge commits created or appended to:

| Kind | Docs | Action |
| --- | --- | --- |
| Measurement | 25 | Keep |
| Reference | 18 | Keep |
| Path or link edits only | 13 | Nothing |
| Relocated comments, whole doc | 14 | Delete the doc, after writing any test it names |
| Relocated comments in sections | 75 | Delete those sections, keep the rest |

Relocated comments go stale faster than the comments did, because nothing next to the code
reminds an editor they exist. Five days after the move:

- Every "Line N, above X" anchor in `docs/hardware/` is wrong (`CP10_PITCH` moved from 38 to 20)
- `kinematic_sim_design.md` and `shared_hazards_file.md` cite `simulation/*.py`, now
  `auto_battlebot/kinematic_sim/`
- `opentx_transmitter.md` cites `kChannelTimeout`, now `kChannelStreamStallTimeout`
- `2026-09-22_zed_frame_rotation.md` defends the "buffer moved" log in `ZedDevice::retrieve`
  after the root cause was fixed
- The basement scene doc warns against a seek in `export_svo_depth.py`, which has none
- `zed_box_mini_mount.md` lost its field names in the move and is unreadable

### What to remove

- `## Workarounds` and `## Workarounds in tests` sections: 34 docs, 37 with "Unverified", "Not
  verified" or "not reproduced" entries. Each entry goes once its ledger row is closed
- Sections keyed to a symbol, file, or line: `### <file>.py`, `<function>:`, "Line N, above X",
  "Beside `...`". The largest are `docs/reference/calibration/sysid_modules.md` (about 960 lines,
  130 headings) and `docs/reference/recording/foxglove_recording_format.md` (about 368 lines)
- Bullets that restate a test: `docs/reference/control/hazards.md` repeats about 14 invariants
  already pinned by `tests/hazards/*` and `HazardBarrierTest.*`
- History ("the old brake", "previously", "when the comment was written") and plan checkpoints
- Implementation notes appended to dated experiments: `opponent_filter_evaluation.md`,
  `mujoco_mr_stabs_model_spike.md`, `rl_navigation_report.md`. Keep their results

### What to keep or move

- Measurements stay in `docs/experiments/`. Rows missing a date, box, or recording get one or
  lose the number. Several carry the purge date (2026-11-15) instead of the measurement date
- How-it-works prose that sits in `docs/experiments/` moves to `docs/reference/` (16 docs, mostly
  the `*_scene_pipeline`, sysid smoother, and apriltag docs)
- Fitted-value provenance in reference docs moves to a dated experiment doc:
  `camera_calibrations.md`, `robot_plant_files.md`, the sweep numbers in `synthgen_cage_specs.md`

### Tunables defended in prose

These are justified in docs instead of living in a config struct. Move each to a defaulted config
field, then delete the prose:

- `kExcessGain = 1.5` in `src/navigation/hazard_avoidance.cpp`, duplicated in
  `tests/hazards/test_hazard_geometry.cpp`
- `kMaxPushOutPasses = 6` in `src/hazards/hazard_geometry.cpp`
- `kChannelStreamStallTimeout` in `src/transmitter/opentx_transmitter.cpp`
- `GATE_MARGIN` and `TAG_DROPOUT_MIN_GAP_NS` in the sysid tag smoother path

### Docs that are relocated comments start to finish

- `docs/experiments/control/2026-11-15_kinematic_sim_design.md`
- `docs/experiments/control/2026-11-15_rl_policy_bundle.md`
- `docs/experiments/control/hazard_avoidance/2026-11-15_shared_hazards_file.md`
- `docs/experiments/control/nav_eval/2026-10-25_reverse_freeze_repro.md`
- `docs/experiments/perception/2026-11-15_field_corner_naming.md`
- `docs/experiments/perception/2026-11-15_free_roam_field_mask.md`
- `docs/experiments/perception/2026-11-15_parallel_model_batch.md`
- `docs/experiments/perception/rgb_homography/2026-11-15_field_transform_comparison.md`
- `docs/experiments/platform/2026-09-22_zed_frame_rotation.md`
- `docs/experiments/platform/2026-11-15_battery_monitor_threading.md`
- `docs/experiments/platform/2026-11-15_jpeg_encoder_selection.md`
- `docs/hardware/README.md`
- `docs/reference/perception/gpu_letterbox_memory.md`
- `docs/reference/robot_link/opentx_transmitter.md`

### Docs with relocated-comment sections

- `docs/experiments/control/2026-11-15_control_loop_threading.md`
- `docs/experiments/control/2026-11-15_rl_policy_hazard_barrier.md`
- `docs/experiments/control/kalman_filter/jig_calibration_provenance_2026-08-20.md`
- `docs/experiments/control/kalman_filter/opponent_filter_evaluation.md`
- `docs/experiments/control/mujoco_mr_stabs_model_spike.md`
- `docs/experiments/control/nav_eval/2026-11-15_forward_step_test.md`
- `docs/experiments/control/nav_eval/2026-11-15_rl_nav_metrics.md`
- `docs/experiments/control/rl_navigation_report.md`
- `docs/experiments/hardware/2026-09-25_mr_stabs_mk2_mass_properties.md`
- `docs/experiments/hardware/2026-11-08_cage_pack_bom_prices.md`
- `docs/experiments/hardware/2026-11-12_cage_pack_shock_model.md`
- `docs/experiments/hardware/2026-11-15_drivetrain_bldc_conversions.md`
- `docs/experiments/perception/2026-09-04_depth_mode_coverage.md`
- `docs/experiments/perception/2026-11-15_basement_scene_pipeline.md`
- `docs/experiments/perception/2026-11-15_cage_high_annotation.md`
- `docs/experiments/perception/2026-11-15_cage_high_scene_pipeline.md`
- `docs/experiments/perception/2026-11-15_homography_field_filter.md`
- `docs/experiments/perception/2026-11-15_massd_scene_pipeline.md`
- `docs/experiments/perception/brettzone_cage_high_recordings.md`
- `docs/experiments/platform/2026-09_camera_following_clock.md`
- `docs/experiments/sysid/2026-08-23_process_noise_fit.md`
- `docs/experiments/sysid/2026-10-15_mujoco_open_loop_heading_hold.md`
- `docs/experiments/sysid/2026-10-15_mujoco_reverse_window_stratification.md`
- `docs/experiments/sysid/2026-10-15_sysid_yolo_dropout_fill.md`
- `docs/experiments/sysid/2026-10-25_mrs_buff_mk3_model.md`
- `docs/experiments/sysid/2026-10-25_sysid_tag_smoother.md`
- `docs/experiments/sysid/2026-11-15_apriltag_ground_truth.md`
- `docs/hardware/cage_pack.md`
- `docs/hardware/mr_stabs_mk2.md`
- `docs/hardware/mr_stabs_mk2_pcb.md`
- `docs/hardware/tx16s_hr10.md`
- `docs/hardware/zed_box_mini_mount.md`
- `docs/reference/calibration/plant_fit_modules.md`
- `docs/reference/calibration/sysid_modules.md`
- `docs/reference/calibration/velocity_jig_config.md`
- `docs/reference/config/profiles.md`
- `docs/reference/config/robot_plant_files.md`
- `docs/reference/control/control_loop.md`
- `docs/reference/control/hazards.md`
- `docs/reference/control/plant_backed_control.md`
- `docs/reference/control/rl_opponents.md`
- `docs/reference/control/rl_policy_bundle.md`
- `docs/reference/control/rl_training.md`
- `docs/reference/eval/eval_code.md`
- `docs/reference/perception/camera_calibrations.md`
- `docs/reference/perception/field_pose.md`
- `docs/reference/perception/robot_filter_pipeline.md`
- `docs/reference/perception/trt_engine.md`
- `docs/reference/perception/trt_yolo.md`
- `docs/reference/recording/foxglove_recording_format.md`
- `docs/reference/recording/svo_recording.md`
- `docs/reference/recording/video_recording.md`
- `docs/reference/remote/protocol.md`
- `docs/reference/robot_link/esc_drive.md`
- `docs/reference/robot_link/esp32_wifi_telemetry.md`
- `docs/reference/robot_link/espnow_link.md`
- `docs/reference/robot_link/firmware_build.md`
- `docs/reference/robot_link/motor_guard.md`
- `docs/reference/robot_link/tuning.md`
- `docs/reference/simulation/kinematic_sim_config.md`
- `docs/reference/tools/gpu_queue.md`
- `docs/reference/tools/kinematic_sim_viewer.md`
- `docs/reference/tools/logo_splash_animation.md`
- `docs/reference/tools/python_packaging.md`
- `docs/reference/tools/synthgen_cage_specs.md`
- `docs/reference/tools/synthgen_render.md`
- `docs/reference/tools/synthgen_scene_contracts.md`
- `docs/reference/tools/yolo_training.md`
- `docs/setup/build_notes.md`
- `docs/setup/claude_code_setup.md`
- `docs/setup/dashboard_kiosk.md`
- `docs/setup/docker_playback.md`
- `docs/setup/install_notes.md`
- `docs/setup/jetson_setup.md`
- `docs/setup/zed_box_mini.md`

### Rule for next time

Add to the Comments section of `CLAUDE.md`: a comment moved into a doc is still a comment. Docs
hold measurements and how the code works now, never a defense of a specific line. No
`Workarounds` sections.
