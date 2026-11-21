# docs

| Folder | Holds |
| --- | --- |
| `setup/` | How a machine or service is set up: Jetson, ZED Box Mini, megamind, Docker playback, install and build facts, the dashboard kiosk and network |
| `guides/` | Procedures you follow start to finish: adding an eval recording, running a sysid session |
| `reference/` | How the code works, one folder per subsystem |
| `hardware/` | Robots, PCBs, mounts, radios |
| `plans/` | Active plans only. A finished plan is deleted once its lasting facts are in `reference/` |
| `experiments/` | Dated measurements and reports, by topic |
| `diagrams/` | Graphviz sources and rendered SVGs (`./docs/generate_diagrams.sh`) |
| `media/` | Images and gifs for the READMEs |

`pseudo_code.md` is user-authored. Do not edit it.

## reference/

| Folder | Docs |
| --- | --- |
| `config/` | `config_extends.md` (extends chains, `from` section files), `robot_plant_files.md` |
| `perception/` | `deeplab_floor_mask_data.md`, `robot_filter_pipeline.md`, `field_pose.md`, `cage_calibration.md`, `camera_profiles.md`, `trt_engine.md`, `trt_yolo.md` |
| `simulation/` | `kinematic_sim_config.md`, `sim_profiles.md` (why each sim profile sets its values) |
| `control/` | `plant_backed_control.md`, `hazards.md`, `rl_policy_bundle.md`, `control_loop.md`, `empty_circle_solver.md`, `rl_opponents.md`, `rl_training.md`, `rl_training_config.md`, `rl_toy_configs.md` |
| `calibration/` | `sysid_modules.md`, `plant_fit_modules.md`, `velocity_jig_config.md` |
| `eval/` | `eval_code.md`, `taxonomies.md` |
| `recording/` | `foxglove_recording_format.md` (the recording contract), `svo_recording.md`, `video_recording.md` |
| `remote/` | `protocol.md` (topics, commands, field projection) |
| `robot_link/` | `esc_drive.md`, `espnow_link.md`, `firmware_build.md`, `firmware_telemetry.md`, `motor_guard.md`, `pid.md`, `tuning.md`, `yaw_control.md` |
| `tools/` | `gpu_queue.md`, `logo_splash_animation.md`, `perception_reliability.md`, `synthgen_cage_specs.md`, `synthgen_config.md`, `synthgen_render.md`, `synthgen_scene_contracts.md`, `domain_mix_arms.md`, `yolo_training.md`, `python_packaging.md`, `comment_budget.md`, `sanitizer_builds.md`, `thread_safety.md` |

## experiments/

| Folder | Topic |
| --- | --- |
| `perception/` | Detector, keypoint, synthetic data, field filter and homography results (`rgb_homography/`, `parallel_yolo_batch/`) |
| `control/` | Navigation, control law, RL and sim reports (`kalman_filter/`, `nav_eval/`, `hazard_avoidance/`) |
| `sysid/` | Jig, MuJoCo, sensing and process noise fits |
| `firmware/` | Mk2 gyro, ESC and motor guard bench results (`espnow_link/`) |
| `hardware/` | Mass properties, cage pack, drivetrain conversions, UPS |
| `platform/` | Camera clocks and stamps, encoders, the GPU queue, status pacing |

## Where a new doc goes

- Steps you follow: `guides/`
- How code works: `reference/<subsystem>/`. Current behavior only
- How a box is configured: `setup/`
- A measurement or result: `experiments/<topic>/YYYY-MM-DD_<name>.md`
- Nothing new at the top level
