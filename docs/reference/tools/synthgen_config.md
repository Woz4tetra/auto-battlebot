# synthgen config

`training/synthetic/synthgen/configuration.py` parses the synthgen TOML specs into these frozen dataclasses. Only fields whose name does not say everything are listed.

## `DamagePartConfig`

One `[[robots.damage_parts]]` entry: a named assembly that battle damage can remove. Faces whose centres fall in the part's boxes are split off into their own objects at load, so a draw can hide the assembly whole. Entries are matched in config order and the first one to claim a face keeps it.

| Field | Meaning |
| --- | --- |
| `pieces` | Each piece is a tuple of boxes, `[x_min, y_min, z_min, x_max, y_max, z_max]` in the model's native frame, like keypoints. A plain part is one piece. A subset part loses 1..n of them. |
| `boxes` (TOML only) | Shorthand for `pieces`. On a plain part the boxes form one piece. On a subset part each box becomes its own piece, which suits a wheel. A piece that needs several boxes, such as a guard with a side wall and a front arm, uses `pieces`, which only a subset part may. Give exactly one of `boxes` or `pieces`. |
| `objects` | Only faces of these Blender objects are claimed (`.001` suffix ignored). Empty means any object. |
| `subset` | A draw removes 1..len(`pieces`) of the pieces rather than all of them (the wheels). |
| `includes` | Other parts of the same robot that go whenever this one does. |
| `count_weights` | Subset parts: relative odds of removing 1, 2, ... pieces. Empty means uniform. |
| `selectable` | False for a part that only goes through another part's `includes`, such as a decal. |

## `RobotConfig`

| Field | Meaning |
| --- | --- |
| `flat_ground` | True for Z-up GLBs (for example Meshy models): the robot sits flat at identity pitch, like distractor CAD robots, instead of the +/-90 deg pitch the Onshape `.gltf` (Y-up) robots need. |

## `DistractorsConfig`

| Field | Meaning |
| --- | --- |
| `robot_air_probability` | None falls back to the `[randomization]` value at the use site. |

## `CageConfig`

| Field | Meaning |
| --- | --- |
| `render_samples` | None keeps the run's `--render-samples`. A cage is darker and has glass, so it usually wants more. |
| `view` | The frame this cage writes: `pinhole` at the rectified matrix, `distorted` as the sensor sees it, or `rectified` through the C++ Rectifier's maps. See `synthgen.lens`. |
| `rectify_alpha` | The `getOptimalNewCameraMatrix` alpha behind the rectified matrix. 1.0 keeps every sensor pixel. |
| `randomize_house_bot` | Draw a new house bot position and heading every scene (`synthgen.house_bot_pose`). False leaves it where the spec's `[house_bot_box]` put it. No effect on a cage without one. |
| `house_bot_clearance_m` | Gap kept between the house bot's footprint circle and each robot's. |
| `house_bot_robot_radius_m` | Footprint radius assumed for a robot when keeping the house bot clear of it. |

## `DamageConfig`

The `[damage]` section: battle damage drawn per robot instance per scene. `synthgen.damage_scene` applies it and reverts it when the scene's frames are written. Each instance's damage is recorded in `manifest.jsonl`, so damage-on and damage-off arms are two filters over one render rather than two renders.

| Field | Meaning |
| --- | --- |
| `scene_probability` | Chance a scene is a damaged scene at all. Damage is drawn per scene because one render call covers all of a scene's camera poses. The fully clean pool is therefore exactly 1 - `scene_probability`, whatever a scene holds. |
| `probability` | Chance each robot instance inside a damaged scene is damaged. |
| `part_severity` | Fraction of a part robot's removable parts to hide, drawn from this range. |
| `chunk_volume_fraction` | Fraction of a fused mesh's bounding volume the cutter is sized to take, drawn from this range. |
| `max_part_volume_fraction` | Parts holding more than this share of the summed part bounding volumes never go. |
| `keypoint_clearance_m` | Clearance protecting keypoint anchors from removal. |
| `protected_name_patterns` | Lowercase substrings of Blender object names that must survive. |
| `removable_parts` | Named parts (`[[robots.damage_parts]]`) this batch may remove. Empty allows every part. |
| `part_count` | How many distinct named parts one damaged instance loses, clamped to what is allowed. |
| `cutter_pool_size` | Cutters pre-built at startup. One scene can need one per robot-like instance. |
