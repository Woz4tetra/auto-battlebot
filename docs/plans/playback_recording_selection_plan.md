# Playback recording selection

Pick a playback recording with a CLI flag instead of editing a TOML file or writing a scratch
overlay. Known recordings, their start frames, and a note about each live in one catalog file.

## Problem

- `_playback.toml` used to carry ~40 commented-out `svo_file_path` lines. Swapping a recording meant
  moving a `#`. The comment purge removed them, and the notes on them (`# field finding`, `# auto
  driving with eric`) went with them.
- The binary takes only `-c` and `--print-config` (`src/main.cpp:190`), so every one-off replay
  writes a throwaway overlay: `config/_replay_scratch.toml` in the replay skill,
  `playground/opponent_prediction/run_replays.py` line 60. A scratch overlay that forgets
  `svo_start_frame = 0` inherits the base's 2000 and EOFs at once.
- `config/playback/svo_start_frames.toml` holds start frames keyed by SVO stem, but no path or
  note, and the C++ side never reads it. Start frames get copied by hand into overlays.

## Design

### Catalog: `config/playback/recordings.toml`

`git mv config/playback/svo_start_frames.toml config/playback/recordings.toml`, then reshape it:

```toml
[recordings."2026-05-02T10-06-06"]
path = "data/svo/NHRL_2026-05-02/2026-05-02T10-06-06.svo2"
start_frame = 8000
note = "NHRL May, vs Clyde"

[recordings."2026-05-01T11-22-34"]
path = "data/svo/tests/2026-05-01T11-22-34.svo2"
note = "field finding"

[recordings."BZ-nhrl_may26_3lb-mrsbuff-ironwarrior-W-17-Cage-2-Overhead-High"]
path = "data/saved_recordings/BrettZone_2026-05/BZ-nhrl_may26_3lb-mrsbuff-ironwarrior-W-17-Cage-2-Overhead-High.mcap"
```

- Key: the file stem, so existing `start_frame_for(path)` lookups keep working.
- `path`: required, repo-relative.
- `start_frame`: optional, default 0.
- `note`: optional free-form string. It is data, so the comment budget does not count it.
- Restore the entries and notes from the pre-purge comment block (`git show 61a04cb1^:config/playback/_playback.toml`).
  Drop entries whose file exists on no machine.

### CLI flags on `auto_battlebot`

| Flag | Effect |
| --- | --- |
| `--recording <name\|path>` | Override the playback source after the merged config loads |
| `--start-frame N` | Override the start frame. Needs `--recording` |
| `--list-recordings` | Print the catalog (name, start frame, present on this machine, note) and exit |

Resolution of `--recording`:

1. If the argument is a catalog key, use its `path` and `start_frame`.
2. Otherwise treat it as a path. If its stem is a catalog key, take that entry's `start_frame`;
   if not, start at 0.
3. A missing file fails at startup with the path it tried.

Which keys get overridden depends on the merged `[rgbd_camera] type`:

| Camera type | Keys |
| --- | --- |
| `ZedSvoPlaybackCamera` | `svo_file_path`, `svo_start_frame` |
| `VideoPlaybackCamera` | `video_file_path`, `start_frame` |
| anything else | startup error naming the profile. The flag never touches a live profile |

A `.svo2` path on a `VideoPlaybackCamera` profile, or a non-SVO path on a `ZedSvoPlaybackCamera`
profile, is a startup error that names the profile to use instead.

### Where it hooks into config loading

- Add an overrides argument to `load_classes_from_config` (`src/config/config.cpp:245`), applied to
  `toml_data` after `load_merged_config` and before any section parses.
- Build the override as TOML text and parse it with `toml::parse(text, "cli --recording")`, then
  merge it into `[rgbd_camera]`. The nodes then carry that source name, so `--print-config` shows
  `cli --recording` as the provenance.
- `ConfigProvenance::source_of` (`src/config/config_provenance.cpp:130`) calls `weakly_canonical` on
  the source path. Teach it to pass non-file sources through unchanged.
- `--print-config` accepts `--recording` too, so a swap can be checked before a long run.

### Catalog loader in C++

New `include/config/recording_catalog.hpp` and `src/config/recording_catalog.cpp`: parse
`recordings.toml`, look up by key or stem, list entries. The catalog path comes from
`get_config_dir()`, so deploys carry it with the rest of `config/`.

## Steps

1. `git mv` the table to `recordings.toml` and reshape it. Restore entries and notes from
   `61a04cb1^`.
2. Python: rename `auto_battlebot/recording/svo_start_frames.py` to `recording_catalog.py` with
   `git mv`. Add `load_catalog()` returning `name -> (path, start_frame, note)`. Keep
   `start_frame_for` with the same signature. Update callers:
   - `training/model_eval/make_eval_dataset.py` (`--start-frames` becomes `--catalog`)
   - `playground/opponent_prediction/run_replays.py`
   - `tests/python/test_svo_start_frames.py`, renamed with `git mv`
3. C++ catalog loader and its GoogleTest: key lookup, stem lookup, missing `start_frame`, bad
   `path` type.
4. Override hook in `load_classes_from_config` and the provenance fix. Tests in
   `tests/test_config.cpp`:
   - SVO profile with `--recording <key>` sets both fields
   - video profile sets `video_file_path` and `start_frame`
   - live camera profile throws
   - extension and camera type mismatch throws
   - `--start-frame` beats the catalog value
   - provenance reads `cli --recording`
5. `main.cpp`: add the three flags and pass the overrides through both the `--print-config` and run
   paths.
6. `run_replays.py`: pass `--recording` instead of writing `svo_file_path` and `svo_start_frame`
   into the overlay. It still needs an overlay for `opponent_mode`, so keep the overlay for that
   key only.
7. Docs:
   - `.claude/skills/replay/SKILL.md`: replace the scratch overlay step with `--recording`, remove
     the "no CLI override" line
   - `docs/reference/config/profiles.md` line 62: describe `recordings.toml` and the flags
   - `docs/guides/adding_eval_recordings.md` lines 34 and 64: new file name and format
   - `training/model_eval/README.md` line 321
   - leave dated `docs/experiments/` files alone, they record what was true then
8. Check that `list_available_profiles` still ignores `recordings.toml`, the same way it ignores
   `svo_start_frames.toml` now.
9. `./scripts/build.sh`, `./scripts/build_and_test.sh`, `./scripts/lint`, and the Python tests.
   Build the release `build/` too, since `build-test/` is debug only.

## Done when

```bash
./build/auto_battlebot --list-recordings
./build/auto_battlebot -c config/playback/mr_stabs_mk2_playback.toml --recording 2026-05-02T11-45-08 --print-config
./scripts/build_and_run.sh -c config/playback/mr_stabs_mk2_playback.toml --recording 2026-05-02T11-45-08
./scripts/build_and_run.sh -c config/playback/brettzone_cage_high.toml --recording BZ-nhrl_may26_3lb-mrsbuff-ironwarrior-W-17-Cage-2-Overhead-High
```

- `--print-config` shows the catalog path and `start_frame = 13000` with source `cli --recording`.
- The SVO replay reaches match state at field init, same as the committed `_playback.toml` default.
- `--recording` on `config/mr_stabs_mk2_desktop.toml` exits with the camera type error.
- No file under `config/` or `data/` changes during any of the runs.

## Out of scope

- Selecting a recording from the remote UI. The catalog makes it possible later; nothing here needs it.
- Changing the committed default in `_playback.toml`. It stays as the no-flag default.
