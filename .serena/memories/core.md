# Project source map

auto-battlebot: autonomous aim-assist + control for NHRL combat robots. Runs on Jetson
Orin Nano with a ZED 2i stereo camera. End-to-end latency target: under 60ms.

## Pipeline (main loop in `Runner`)
camera -> perception (field mask + robot blobs + keypoints) -> filter -> target
selection -> navigation -> transmit

## Architecture: config-driven factory pattern
The active TOML config selects which concrete impl of each interface is instantiated at
startup. All code in namespace `auto_battlebot`.
- Interfaces: `include/<module>/`
- Implementations: `src/<module>/`
- Factories wire interface->impl based on config.
- New component: add interface header + impl + register in the module's factory.

## Modules (`include/*`)
channels, config, crsf, data_structures, diagnostics_logger, enums, field_filter, health,
keypoint_model, logging, mask_model, mcap_recorder, navigation, publisher, rgbd_camera,
robot_blob_model, robot_filter, serial, simulation, target_selector,
tensorrt_inference, time, transmitter, ui, viz, foxglove_adapters

## Other top-level dirs
- `tests/` — GoogleTest (build with build-test/)
- `training/` — Python analysis + model training tooling (YOLO format, in-repo editors)
- `simulation/` — Python sim + sweep/scoring harness
- `scripts/` — build/run/deploy/format entrypoints
- `config/` — TOML configs (see `mem:suggested_commands` for the real file layout)
- `playground/` — Python experiments
- `firmware/`, `docker/`, `service/`, `secrets/` — deploy/runtime support
- `data/` — MCAP recordings, SVO files, TensorRT engines. DO NOT MODIFY.

## Invariants
- No ROS at all (removed 2026-09-07). Viz/recording go through the Foxglove SDK; see
  `docs/foxglove_recording_format.md` and `viz_relay`.
- No blocking calls in the perception loop (tight latency budget).
- Prefer TOML config over compile-time switches.
- Compiler flags: -Wall -Wextra -Werror. Fix warnings, never suppress.
- Playback mode with SVO recordings is the primary hardware-free regression path.