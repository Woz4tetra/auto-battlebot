# Commands

## Build / test
- `./scripts/build.sh` — release build -> build/
- `./scripts/build_and_test.sh` — debug + GoogleTest -> build-test/
- `./scripts/build_and_test.sh --gtest_filter=KeypointTest.*` — run specific tests
- `./scripts/clean_build.sh` — remove build artifacts

## Run (config selects the whole component graph)
- `./scripts/build_and_run.sh -c config/<profile>.toml` — build + run a profile
- `./scripts/run.sh -c config/<profile>.toml --print-config` — dump merged config and exit
- `./scripts/run_simulation.sh` — kinematic/genesis simulation mode
- Binary flags: only `-c/--config <path-or-profile>` and `--print-config`. No per-key CLI
  override (SVO path, model paths, etc. must be set in a config).

## Config file layout (CLAUDE.md examples are STALE)
There is NO `main.toml` or `playback.toml` (old flat names, removed). Configs compose via an
`extends` chain resolved against the repo `config/` root, so an `extends` value like
`playback/_playback` works from any subdir:
```
_common.toml                       # shared base, never run directly
  _desktop.toml / _jetson.toml     # platform bases (x86 dev vs Jetson aarch64)
    mr_stabs_mk2_{desktop,jetson}.toml, mrs_buff_mk3_{desktop,jetson}.toml   # live hardware
    playback/_playback.toml -> playback/<robot>_playback.toml   # SVO replay (hardware-free)
    simulation/{_simulation,headless_sim,genesis}.toml
    experiments/{label_playback,eval_candidate,experiment_playback}.toml
profiles.toml                      # standalone; UI profile switcher (excludes _-prefixed)
```
- Hardware-free dev/regression: `config/playback/<robot>_playback.toml` (set
  `[rgbd_camera] svo_file_path`). Deploy: `*_jetson.toml`.
- Detector eval configs: `config/experiments/`. See project skills `/replay` and `/eval`.
- Note: `main.toml` still appears as a stale fallback in `src/config/config.cpp` and
  `src/main.cpp` help text — treat as a latent bug, not current behavior.

## Deploy
- `./scripts/deploy_to_jetson.sh`, `./scripts/build_and_install.sh`

## Formatting
- `./scripts/apply_formatting` (C++ clang-format Google style; Python ruff; TOML taplo)
- WARNING: shared working tree. Only format files you intentionally changed. Never run
  repo-wide apply_formatting unprompted.