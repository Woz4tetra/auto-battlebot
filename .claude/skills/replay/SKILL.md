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
  _desktop.toml / _zed_box.toml   # platform bases (x86 dev vs ZED Box Mini)
    <robot>_desktop.toml          # live hardware configs
    playback/_playback.toml       # extends _desktop; SVO source + PlaybackTransmitter
      playback/mr_stabs_mk2_playback.toml   # per-robot label mappings
      playback/mrs_buff_mk3_playback.toml
```

There is **no `main.toml` or `playback.toml`** (old flat names, removed). Pick the recording
with `--recording`; known recordings, their start frames, and a note on each live in
`config/playback/recordings.toml`. Do not write a scratch overlay to swap the SVO.

## Steps

1. **Resolve the recording.** `./build/auto_battlebot --list-recordings` prints every catalog
   entry, its start frame, and whether the file is on this machine. Use the catalog name (the
   file stem). A path also works: if its stem is in the catalog it takes that start frame,
   otherwise it starts at 0. Never modify anything under `data/`.

2. **Pick the robot base.** Default `playback/mr_stabs_mk2_playback`; use
   `playback/mrs_buff_mk3_playback` if the user names mrs_buff. This sets which robot is
   OUR_ROBOT vs THEIR_ROBOT. BrettZone `.mcap` video recordings need a `VideoPlaybackCamera`
   profile: `playback/brettzone_cage_high`.

3. **(Optional) verify resolution** before a long run:

   ```bash
   ./scripts/run.sh -c config/playback/mr_stabs_mk2_playback.toml --recording <name> --print-config
   ```

   It dumps the merged config and exits. `svo_file_path` and `svo_start_frame` show source
   `cli --recording`; confirm them and the model engine paths.

4. **Run:**

   ```bash
   ./scripts/build_and_run.sh -c config/playback/mr_stabs_mk2_playback.toml --recording <name>
   ```

   It builds release into `build/`, then runs. Add `--start-frame N` to start somewhere other
   than the catalog's frame.

   Leave `svo_real_time_mode` and `rebase_stamps` alone. Both default to true, and both have to
   stay true for the replay to reproduce field timing. `rebase_stamps = false` leaves frame
   stamps on the recording's clock while the control loop reads the wall clock, and
   `svo_real_time_mode = false` decodes as fast as the machine allows; either one breaks the
   filter's latency compensation, so the emitted estimate sits at the shutter position instead of
   leading it. Both log a warning at startup. Set `svo_real_time_mode = false` only for a
   throughput run where timing does not matter, and say so in the write-up.

## What to observe

- With `[ui] enable = true` (default via `_desktop`), the overlay window shows field mask,
  robot blobs, keypoints, and target selection.
- With `[mcap] enable = true`, a recording is written for offline analysis (see `/eval`).
- Watch for perception dropout and coast behavior on the opponent track (known baseline:
  live <50% of frames, p90 gap ~340ms).

## Notes

- A run that needs another setting changed (not just the recording) still takes a throwaway
  overlay that `extends` the robot base; underscore-prefix it (`config/_replay_scratch.toml`).
  It is not gitignored, so delete it when done and do not commit it. Pass `--recording` to it as well, so it never inherits the
  base's `svo_start_frame`.
- To add a recording, add an entry to `config/playback/recordings.toml`. Changing the committed
  default in `playback/_playback.toml` is the user's call.
- `svo_real_time_mode = false` gives deterministic frame-exact replay (stamps come from the
  SVO frame stamp), which is what regression comparisons rely on.
