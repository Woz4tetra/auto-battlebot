# auto-battlebot

Autonomous aim-assist and control system for NHRL combat robot competitions. Runs on a Jetson Orin Nano with a ZED 2i stereo camera. End-to-end latency target: under 60ms.

## Build

```bash
./scripts/build.sh                                         # release build, output: build/
./scripts/build_and_test.sh                                # debug + GoogleTest, output: build-test/
./scripts/build_and_test.sh --gtest_filter=KeypointTest.*  # run specific tests
./scripts/clean_build.sh                                   # remove build artifacts
```

## Run

```bash
./scripts/build_and_run.sh -c config/playback.toml  # replay SVO recording, no hardware needed
./scripts/run_simulation.sh                          # simulation mode
./scripts/build_and_run.sh -c config/main.toml       # full hardware mode
```

Use playback mode for development and regression testing.

## Formatting

Run before committing:

```bash
./scripts/apply_formatting
```

- C++: Google style, 4-space indent, 100-char line limit (`.clang-format`)
- Python: `ruff`
- TOML: `taplo`

## Architecture

Config-driven factory pattern. The active TOML config selects which implementation of each interface gets instantiated at startup. Main loop in `Runner`:

```
camera -> perception (field mask + robot blobs + keypoints) -> filter -> target selection -> navigation -> transmit
```

All code is in namespace `auto_battlebot`. Interfaces live in `include/<module>/`, implementations in `src/<module>/`, factories wire them together based on config.

## Conventions

- New component: add interface to `include/<module>/`, implementation to `src/<module>/`, register in the factory
- Prefer TOML config over compile-time switches for behavior changes
- No full ROS. The project uses `miniroscpp` intentionally. Do not add `package.xml` or full ROS dependencies
- Compiler flags are `-Wall -Wextra -Werror`. Fix warnings, do not suppress them

## Testing

Tests use GoogleTest in `tests/`. Build with `build-test/` (debug + `BUILD_TESTING=ON`). Playback mode with SVO recordings is the primary way to regression-test without hardware.

## Platforms

- Deployment: Jetson Orin Nano (aarch64, TensorRT 10, CUDA)
- Dev: Ubuntu 22/24 x86_64 with NVIDIA GPU
- `pyproject.toml` has platform-conditional deps. Do not flatten them.

## What to avoid

- No blocking calls in the main perception loop. The latency budget is tight.
- Do not modify files under `data/` (MCAP recordings, SVO files, TensorRT engines).
