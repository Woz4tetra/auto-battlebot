# scripts

Build, run, deploy, and inspect. Run everything from the repo root.

Several of these are manual entry points that nothing else calls; they are listed
here so they read as tools rather than as dead files.

## Build

| Script | When to run it |
| --- | --- |
| `build.sh` | Release build into `build/`. The everyday one. |
| `build_debug.sh` | Debug build. |
| `build_and_test.sh` | Debug build plus GoogleTest into `build-test/`. Takes `--gtest_filter=...`. |
| `clean_build.sh` | Remove build artifacts. Also clears stale object files after a large refactor. |
| `build_and_install.sh` | Build and install to the system prefix. Used by the systemd unit in `service/`. |

## Run

| Script | When to run it |
| --- | --- |
| `run.sh` | Run the built binary. |
| `build_and_run.sh` | Build, then run. `-c <config>` selects the profile. |
| `run_simulation.sh` | Simulation mode against the Genesis server. |
| `run_viz_relay.sh` | Start `viz_relay`, which owns the Foxglove websocket on 8765. The app streams to it over a unix socket. |

## Setup and deploy

| Script | When to run it |
| --- | --- |
| `install_ubuntu_22.sh`, `install_ubuntu_24.sh` | Dev machine setup, per Ubuntu release. |
| `install_jetson.sh` | Jetson Orin Nano setup. |
| `setup_python.sh` | Create `venv/`. |
| `activate_python.sh` | `source` it to activate `venv/`. |
| `setup_simulation.sh` | Install the simulation dependencies. |
| `setup_claude_code.sh` | Only for headless or CI setups. Interactive use needs nothing: `.claude/settings.json` already declares the plugin and the wording hook. |
| `deploy_to_jetson.sh` | Push a build to the robot. Honors `.deployignore`. |
| `sync_models.py` | Sync model and engine files. |
| `docker/` | Container entrypoints for the playback image. See `docs/docker_playback.md`. |

## Recordings and diagnostics

| Script | When to run it |
| --- | --- |
| `download_recordings.sh` | Pull SVO and MCAP recordings off the Jetson, skipping any basename already present under `data/`. Set `JETSON_HOST` to target a different machine. |
| `convert_ros1_mcap.py` | One-way migration for recordings made before ROS was removed. `auto_battlebot/recording/mcap_io.py` raises an error naming this script when it meets a legacy file. |
| `nhrl_to_recordings.py` | NHRL fixed-cage fight video to a replayable recording set: fetch, convert, land in `data/saved_recordings/<set>/`, write the playback config. Re-runs skip what is already converted. |
| `video_to_mcap.py` | The primitive under it: one or more videos to one MCAP each, shaped like the RGB camera's output. Use `nhrl_to_recordings.py` for NHRL footage. |
| `mcap_latency_report.py` | Per-tick and per-stage latency report from a recording. The source of the latency numbers in `docs/experiments/`. |
| `mcap_out.py` | Print `/log` messages in journalctl style. Reach for it when a run misbehaved and you want the log without opening Foxglove. |
| `mcap_topic_sizes.py` | Total size, average message size, and KB/s per topic. Use it when a recording is unexpectedly large. |

## Checks

| Script | When to run it |
| --- | --- |
| `lint` | Apply formatters, then run clang-tidy, ruff, and mypy. `--quick` skips the slow two. Run before committing. |
| `no_ai_slop_check.sh` | Wording check. Wired to PostToolUse in `.claude/settings.json`, so it runs on write. |
