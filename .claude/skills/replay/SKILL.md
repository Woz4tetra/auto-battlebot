---
name: replay
description: Replay an SVO recording through the perception -> filter -> nav stack in playback mode (hardware-free). Use when asked to "replay", "play back", "run an SVO", "regression test a recording", or reproduce field behavior from a recording on the dev machine.
---

# replay

Runs an SVO recording through the full C++ stack in playback mode. No camera or radio
needed. This is the primary hardware-free regression path.

## Config structure (read first)

Configs live in `config/` and compose via an `extends` chain resolved against the repo
`config/` root (so `extends = "playback/_playback"` works from any subdir):

```
_common.toml                      # shared base, not run directly
  _desktop.toml / _jetson.toml    # platform bases (x86 dev vs Jetson)
    <robot>_desktop.toml          # live hardware configs
    playback/_playback.toml       # extends _desktop; SVO source + PlaybackTransmitter
      playback/mr_stabs_mk2_playback.toml   # per-robot label mappings
      playback/mrs_buff_mk3_playback.toml
```

There is **no `main.toml` or `playback.toml`** (old flat names, removed). The SVO path is
set via `[rgbd_camera] svo_file_path`; there is **no CLI override** for it (the binary only
takes `-c/--config` and `--print-config`).

## Steps

1. **Resolve the SVO.** Recordings live in `data/svo/` and `data/svo/tests/`. Accept an
   absolute path or a bare name; if bare, check both dirs. Never modify anything under
   `data/`.

2. **Pick the robot base.** Default `playback/mr_stabs_mk2_playback`; use
   `playback/mrs_buff_mk3_playback` if the user names mrs_buff. This sets which robot is
   OUR_ROBOT vs THEIR_ROBOT.

3. **Write a throwaway overlay** so shared configs stay untouched (the working tree is
   co-edited). Underscore prefix keeps it out of the UI profile switcher; it is gitignored.
   Write `config/_replay_scratch.toml`:

   ```toml
   extends = "playback/mr_stabs_mk2_playback"

   [rgbd_camera]
   svo_file_path = "data/svo/tests/<recording>.svo2"
   svo_real_time_mode = false   # true = wall-clock speed; false = as fast as possible
   ```

4. **(Optional) verify resolution** before a long run:
   `./scripts/run.sh -c config/_replay_scratch.toml --print-config` (dumps the merged
   config and exits — confirm `svo_file_path` and the model engine paths are what you
   expect).

5. **Run:** `./scripts/build_and_run.sh -c config/_replay_scratch.toml`
   (builds release into `build/`, then runs). Add `svo_start_frame = N` under
   `[rgbd_camera]` to skip ahead.

## What to observe

- With `[ui] enable = true` (default via `_desktop`), the overlay window shows field mask,
  robot blobs, keypoints, and target selection.
- With `[mcap] enable = true`, a recording is written for offline analysis (see `/eval`).
- Watch for perception dropout and coast behavior on the opponent track (known baseline:
  live <50% of frames, p90 gap ~340ms).

## Notes

- If the run needs a specific SVO permanently (not a one-off), the user edits the committed
  playback config themselves; do not commit `_replay_scratch.toml`.
- `svo_real_time_mode = false` gives deterministic frame-exact replay (stamps come from the
  SVO frame stamp), which is what regression comparisons rely on.
