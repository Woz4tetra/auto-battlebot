# Config extends and loading

## Effective config dump

`load_merged_config(path)` (`include/config/config.hpp`) loads `path` and resolves its `extends` chain into one merged TOML table, which is exactly the data fed to every section parser. `--print-config` parses it through every section parser and prints each field the parsers read, defaults included, tagged with the `file.toml:line` that set it or `default` (`include/config/config_provenance.hpp`). The Python mirror is `load_profile_toml(name)` in `auto_battlebot/config_profile.py`.

## Section files (`from`)

A table that holds only `from` loads its contents from another file. The file holds the table at
the same path, so one file can serve several sections:

```toml
# config/mr_stabs_mk2_jetson.toml
[plant]
from = "robots/mr_stabs_mk2"

# config/robots/mr_stabs_mk2.toml
[plant]
k_fwd = 5.7302
...

[plant.process_noise]
q_along = 10.0
...
```

Rules, enforced by `src/config/config.cpp` and mirrored in `auto_battlebot/config_profile.py`:

- The path resolves against `config/` like `extends`; `.toml` is appended if missing.
- Works at any depth: `[robot_filter.motion_estimator] from = "robots/x"` reads
  `[robot_filter.motion_estimator]` from `config/robots/x.toml`. Nothing else in the file is read.
- Expansion runs per file, before the `extends` merge, so the result is the same as writing the
  table inline in that file.
- The loaded table replaces the inherited one outright, subtables included. A file that omits a key
  never inherits it from a base.
- A later file in the chain that writes the section inline merges key by key on top, as usual.
  The overlay generators (`lockstep_eval.py`, `sim_stop_test.py`, `run_replays.py`) rely on this.
- `from` must be the only key in its table, a subtable included (`[plant.process_noise]` beside
  `[plant] from = ...` is an error). Put it in the section file.
- A section file may not use `extends` or `from`.
- `from` at the top level of a file is an error.

Each error names the profile, the section, and the file. `--print-config` shows the expanded
values.

`PlantParams.from_toml` (`auto_battlebot/control/plant.py`) expands `from` too, so a profile path
passed to it still yields the robot's plant instead of all-default parameters.

Section files live in `config/robots/`, `config/transmitters/`, `config/navigation/`,
`config/field_filters/` and `config/keypoint_models/`. `docs/reference/config/profiles.md` lists
which profile loads which, and `docs/reference/config/robot_plant_files.md` covers the plant files.

## Type swaps

A variant that changes a section's `type` replaces the whole section instead of merging key by key.
Base-only fields (for example `camera_fps` under `ZedRgbdCamera`) would otherwise reach the new
implementation and trip its unknown-fields check. `ConfigTest.ExtendsTypeChangeReplacesSection`
pins this.

## Label to keypoint mapping format

The label-to-keypoints config is an array of tables, which guarantees order:

```toml
[[section]]
label = "LABEL_NAME"
keypoints = ["KEYPOINT_LABEL_1", "KEYPOINT_LABEL_2"]
```

Storage is ordered so indices match the model's class order.

## Config casts

Factory functions receive a `const BaseConfig&`. Component constructors take `const DerivedConfig&`. `config_cast<DerivedConfig>` in `include/config/config_cast.hpp` is a `static_cast` to the derived type.
