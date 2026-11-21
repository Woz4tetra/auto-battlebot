# Workaround ledger

Every workaround the phase 1 comment audit found (2026-11-15, 322 rows), with its root-cause fix. The audit CSV was deleted on 2026-11-20 and is still in git history at `docs/plans/comment_purge_audit.csv`. Fixed rows were deleted as they landed; their commits carry the details.

## Status 2026-11-20

Every row that could be settled on the dev box is gone. It was either deleted, fixed, or pinned by a test that fails with the workaround removed. Checks that need a device are in `docs/plans/hardware_checks.md` ("Ledger rows that need a device"). Three accepted rules stay as comments, listed in `docs/reference/tools/comment_budget.md`.

What is left:

- Two rows deferred to Mk4
- Four rows owned by active plans
- Seven rows kept by decision
- The follow-up doc purge below

## Deferred to Mk4

One root-cause fix retires both: port DShot to the IDF 5 `rmt_tx` driver and pin the IDF 5 platform explicitly.

| Location | What the code does | What breaks without it |
| --- | --- | --- |
| `firmware/espnow_dongle/platformio.ini:8` | Pins `platform = espressif32@6.13.0` (Arduino core on IDF 4.4) | An unpinned `espressif32` resolves to the globally installed pioarduino 55.x (IDF 5), which changes the ESP-NOW frame limits the wire format assumes (250 bytes, `espnow_protocol.h`) |
| `firmware/mr_stabs_mk2/platformio.ini:15` | Pins `platform = espressif32@6.13.0` (Arduino core 2.x, IDF 4.4) | IDF 5 removes NeoPixel's `rmt_reserved_channels` and deprecates the legacy RMT driver `DShot.cpp` uses |

## Owned by a plan

The row closes when the named step lands.

| Location | Plan and step |
| --- | --- |
| `tests/test_kalman_motion_estimator.cpp` (`make_our_pose_measurement`, `make_opponent_pose_measurement`) | `docs/plans/measurement_chain_simplification_plan.md` step 1: delete the blob path, so every measurement carries a yaw and the dummy keypoint goes |
| `include/robot_descriptions_cache.hpp` (`Resolved` lifetime) | `docs/plans/measurement_chain_simplification_plan.md` step 3: `RobotDescriptionsCache` is deleted |
| `src/navigation/rl_policy_navigation.cpp` (coasted spin after the robot is lost) | `docs/plans/measurement_chain_simplification_plan.md` step 3: zero velocity once the mean stops at `max_coast_s`, and tracks expire |
| `include/rl_policy/policy_interface.hpp` (navigation outputs stick values in `VelocityCommand`) | `docs/plans/command_units_plan.md`: navigation outputs real units and `CommandShaper` converts them to sticks |

## Kept by decision

Each workaround stays. Its reason is in the subsystem doc until the follow-up purge removes it.

| Location | Decision |
| --- | --- |
| `include/mask_model/fixed_mask_model.hpp` | Kept 2026-11-20. The kinematic sim's fake 2x2 mask gets the runner past its empty-mask check |
| `hardware/cage_pack/enclosure.py:221` | Kept 2026-11-20. Settling it needs an Onshape push, which costs API calls from the yearly budget |
| `hardware/cage_pack/enclosure.py:295` | Kept 2026-11-20, as above |
| `hardware/zed_box_mini_mount/assembly_page.py:62` | Kept 2026-11-20. Testing GLB handling on the artifact host needs an upload |
| `tests/python/test_tag_pose_smoother.py:578` | Kept 2026-11-20. Retune declined |
| `auto_battlebot/mujoco_sim/onshape_export.py` (wheels-only mass lumping) | Skipped 2026-11-20. Purchased parts have no real mass in the Onshape part studio |
| `training/model_eval/score_size_ladder.sh:97-98` | Skipped 2026-11-20. Needs the `cage_high_x50_conf044/d40000` eval set, which is only on megamind |

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
