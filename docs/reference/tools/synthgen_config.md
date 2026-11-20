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

## `training/synthetic/config.toml`

The default spec for the Blender synthetic pipeline. Provenance for its fitted values is in `docs/experiments/perception/2026-11-19_synthgen_cage_mount_ranges.md`. Keys the dataclass tables above already explain are not repeated.

### `[[cages]]`

Real arenas the run renders some of its images in, alongside the HDRI arena scenes. Each entry is an independent scene with its own geometry spec, camera, lighting and mount ranges. There are three: `nhrl_cage` (the NHRL cage), `massd_arena` (the MassDestruction arena at the Charles River Museum: a 2.26 m plywood floor with a 0.43 m pit, wooden kick rails, a much brighter room than NHRL's cage, fitted from one broadcast clip, see `docs/experiments/perception/massd_arena_scene_2026-09-11.md`) and `meatball_basement` (the basement drive-test box: a 1.52 m plywood floor in 2x4 rails, in a corner of stone walls, fitted from our own ZED).

| Field | Meaning |
| --- | --- |
| `probability` | Fraction of the whole run. The cage probabilities share the run with the HDRI arena, so they must sum to at most 1.0. What is left over is arena scenes. The split is tracked, not coin-flipped, so a 100-image run lands on these ratios. |
| `enabled` | `meatball_basement` is off so existing runs keep their ratios. `config_cage_meatball.toml` pins a run to it with `only_cage`. |
| `spec` | Cage geometry, materials, lights and the exposure solved against footage. |
| `camera_calibration` | Our own e-CAM25, rectified at `rectify_alpha` the way the C++ `Rectifier` does it and scaled to `[output].image_width`/`image_height`. All three cages use the same lens so they stand in for one camera. |
| `tube_jitter` | Per-scene jitter on the LED tube emission strength, plus or minus this fraction, so the set is not a single lighting state. |
| `mat_margin_m` | Metres of mat kept clear of its edge when placing robots. 0.15 on the meatball box leaves a 1.22 m square of the 1.52 m floor. |
| `house_bot_clearance_m` | Measured between footprint circles, with each robot's radius taken as 0.20 m. |

### `[cages.mount]`

Where a camera can be clamped on the cage and where it then looks. Each draw picks a wall from `walls`, then samples the rest. `aim = "centre"` derives tilt from where the mount landed and jitters it by `tilt_offset_deg`, instead of drawing tilt independently.

| Field | Meaning |
| --- | --- |
| `along_m` | Metres along the wall from its centre. |
| `height_m` | Metres above the mat surface. Not capped by the polycarbonate top, since the mount is outside it. |
| `aim` | `"fixed"` samples `tilt_deg` directly. `"centre"` derives tilt so the optical axis meets the field centre and samples `tilt_offset_deg` on top, which ties tilt to how far out and how high the mount landed instead of drawing the three independently. |
| `inset_m` | Metres inside the wall plane. Negative is outside the cage, which is allowed: a pane the camera looks through from outside is hidden for that frame (`panels_outside_camera`), so it costs no labels. Keep it strictly negative so the hiding of the pane the camera looks through always applies. The near end is a render-cost limit, not a framing choice. |
| `tilt_offset_deg` | Degrees added to the aim-at-centre tilt. Tilt is measured off straight down, so negative is steeper, aimed nearer the camera and short of the centre. |
| `yaw_deg` | Heading off square-to-the-wall. |
| `roll_deg` | Camera roll about the optical axis. |

The NHRL ranges cover the eight near-glass poses of 12 flown by hand through the e-CAM25 lens. The MassD ranges come from the 2026-09-12 coverage sweeps, pulled in for the wider e-CAM25 lens. The meatball ranges bracket the stand the ZED stood on, and only the two open sides (`near`, `left`) are used. Numbers are in the experiments doc above.

### `[output]`

| Field | Meaning |
| --- | --- |
| `annotation_mode` | `keypoints_bbox` (default) writes YOLO pose labels with bbox and keypoints. `segmentation_bbox` writes YOLO segmentation polygons, with the bbox implied by the polygons. |
| `images_per_scene` | Camera viewpoints rendered per scene arrangement. Higher is faster generation but less object placement diversity. |
| `min_robot_visibility` | Minimum visible fraction of robot pixels after occlusion: visible instance pixels (with distractors) divided by unobstructed instance pixels from a distractor-free segmentation pass. Ignored when `ignore_obstructions` is true. |
| `ignore_obstructions` | Annotate robots and keypoints straight through occluding objects. Skips the distractor-clearing pass, the clean visibility-fraction render and the per-keypoint depth occlusion test. Every in-frame keypoint is marked visible (flag 2) and no frame is dropped for being occluded. Use it when the obstruction checks discard too many otherwise-usable frames. |
| `memory_cleanup_interval` | Forces periodic Blender data cleanup during long renders to avoid memory growth. 0 disables it. |

### `[[robots]]`

Each entry defines a robot model that can appear in scenes. Per scene, 1..`[scene].max_robots_per_scene` robots are selected by weighted sampling without replacement.

| Field | Meaning |
| --- | --- |
| `scale` | Applied to the imported model, 1.0 for none. Adjust if the Onshape export units do not match Blender's metres. |
| `weight` | Relative probability of the robot being chosen for a scene. |
| `ground_roll_upright`, `ground_roll_inverted` | Roll (Y-rotation, degrees) used when grounded. Pitch is fixed at +/-90 deg and yaw is always randomized. |
| `[robots.keypoints]` | 3D keypoint positions in model-local space, metres, measured from the CAD model origin in Onshape. |
| `[[robots.color_mapping]]` | Maps Onshape part colours (RGB 0-255) to material types. `blenderproc run prepare_robot_model.py config.toml --inspect` lists every colour present in the GLTF. |

To add a robot, duplicate a `[[robots]]` block with its `[robots.keypoints]` and `[[robots.color_mapping]]` entries.

`[[robots.damage_parts]]` entries for Mrs Buff remove whole assemblies instead of the random-fraction part hiding, because the GLB groups faces by colour, not by part, so that path only ever hid a scatter of bolts. Box coordinates are in the model frame the keypoints use: x from the back (-0.177) to the weapon (+0.045), y across the wheels, z up when the robot sits upright. They were read off face-centre plots of the Blender export on 2026-09-13. Order matters because the first entry to claim a face keeps it, so the stickers and the disk go before the parts around them. `objects` names the GLB's colour objects, and the export's `_N` index suffixes match. `mat_41_123_203` holds both the blue guard and the blue weapon pod, so boxes separate those.

| Part | Meaning |
| --- | --- |
| `top_sticker`, `bottom_sticker` | Only ever removed with their plate (`selectable = false`). |
| `wheels` | Each box is one wheel. A wheel usually comes off alone, so one is the likely draw. |
| `wheel_side_guards` | The red (-y) and blue (+y) TPU guards around the wheels, each one printed piece: a side wall and a front arm with the forks moulded in. One side usually goes, sometimes both. |
| `weapon_module` | The pod above the top plate: weapon motor, belt and bearing blocks. The disk hangs on it. |
| top plate | Three boxes leave out the weapon pulley, which shares the plate colour. |

### `[materials.*]`

PBR properties per material type, shared across all robots.

| Field | Meaning |
| --- | --- |
| `cc_texture` | An ambientCG (CC0 Textures) asset ID, applied tiling. `tin_foil` uses `Foil002` (gold foil); `Foil003` is the silver foil. |
| `texture_dir` | A directory of PBR images (Color, Roughness, Metalness, NormalGL, Displacement) applied through the model's UV mapping, which preserves its UV layout instead of tiling with generated coordinates. |

`tin_foil` can also be assigned to a guard through `[[robots.color_mapping]]` to give the front or weapon guard a foil finish.

### `[distractors]`

Each `[[distractors.sources]]` entry has a `path` and a `weight`. Weights are relative and need not sum to 1: a source with weight 3.0 is three times as likely to contribute a model as one with weight 1.0. `kind = "objaverse"` is an alternative source kind.

| Field | Meaning |
| --- | --- |
| `max_per_scene` | Also controls how many distractor models are loaded into memory at once. |
| `vram_budget_mb` | Approximate VRAM budget across the whole loaded distractor pool. 0 or absent disables enforcement. |
| `vram_audit_csv` | Precomputed per-model VRAM estimates used for pre-import budget checks. |
| `base_dimension_m` | Reference largest dimension distractor robots are scaled to. Every distractor robot is treated as 3 lb (beetleweight) regardless of its real weight class, so this is the 3 lb base size. It does not depend on the robots in the scene. |
| `scale_range` | Multiples of `base_dimension_m`. [0.5, 2.0] gives 0.5x to 2x the 3 lb base size. |
| `shuffle_interval` | Re-roll the distractor pool every N images. 0 keeps the same pool all run. |
| `robot_air_probability`, `robot_air_height_range` | Pose randomization for CAD (`kind = "cad"`) robot distractors. Grounded robots sit flat on the field with a random yaw and a 50 percent chance of being inverted. Airborne ones tumble with a fully random orientation. Objaverse distractors keep the global `[randomization]` air settings. |
| `motion_blur_probability` | Per-frame motion-blur probability for distractor categories. Defaults to `[randomization].motion_blur_probability` when omitted. |

### `[environment]`

| Field | Meaning |
| --- | --- |
| `ground_textures` | Extra ambientCG assets for ground plane randomization. `download_ambientcg.py --from-config` downloads them alongside the robot materials. |

### `[camera]`, `[scene]`, `[randomization]`

| Field | Meaning |
| --- | --- |
| `[camera].look_at_noise` | Gaussian noise added to the camera look-at target, metres. |
| `[camera].max_frame_fraction` | Largest fraction of the narrower image dimension the robots' bounding sphere may fill, 0-1. The camera is pushed back past `min_distance` whenever needed so a robot can never completely dominate the frame, which yields confusing near-full-frame annotations. Lower keeps robots smaller. |
| `[scene].ground_size_range` | Randomized range for the ground plane half-extent, metres. |
| `[scene].arena_radius_range` | Randomized range for the radius within which objects are placed, metres. |
| `[scene].ground_visibility` | Fraction of scenes that include a visible ground plane, 0.0 to 1.0. The remainder show only the HDRI background behind the robots. |
| `[scene].max_robots_per_scene` | Maximum robots rendered simultaneously per scene, 1 for always one. Each scene picks between 1 and this value, weighted by each robot's `weight`, clamped to the number of `[[robots]]` entries. |
| `[randomization].air_probability` | Probability the robot is airborne with a fully random tumbling orientation. The remainder of images place the robot on the ground in a `ground_pitch` pose. |
| `[randomization].air_height_range` | Height above ground when airborne, metres. |
| `[randomization].motion_blur_probability` | Directional motion blur applied to robot and distractor objects. Probability a frame receives it, 0.0 to 1.0. |
| `[randomization].motion_blur_strength_range` | Blur kernel size range in pixels, chosen randomly per frame. |
| ground orientation | Pitch (X-rotation) is fixed at +90 deg right-side-up and -90 deg upside-down. Roll (Y-rotation, degrees) is configured per orientation in each `[[robots]]` entry, to compensate for the model not sitting perfectly flat after the GLTF axis conversion. Yaw (Z-rotation) is always fully random. |

### `[damage]`

Battle damage, drawn per robot instance per scene, because a real robot two minutes into a match is missing parts. Drawn by `synthgen/damage.py`, applied and reverted by `synthgen/damage_scene.py`, and recorded per instance in `manifest.jsonl` beside `images/` and `labels/`.

The manifest makes damage-on and damage-off arms two filters over one render, not two renders. A damage-off arm keeps the frames whose every instance reports `damage == 0`, and `scene_probability` sizes that pool. Annotations need no special handling: damage is applied before both render passes, so bboxes and keypoint visibility are computed from the damaged silhouette.

| Field | Meaning |
| --- | --- |
| `scene_probability` | Damage is drawn per scene because one render call covers every camera pose in the scene. 0.5 splits the frames exactly into clean and damaged halves. |
| `probability` | Robot-like distractors (the Meshy opponent pool) are rolled too. Scenery distractors are not, since a cut-up traffic cone teaches the detector nothing about a battlebot. |
| `part_severity` | CAD robots arrive split into parts, so damage hides this fraction of the removable ones. |
| `removable_parts` | The batch's pre-selection of which `[[robots.damage_parts]]` may go. Robots with `damage_parts` (Mrs Buff) lose named assemblies instead. Empty allows every part a robot names. `--damage-parts` overrides it per run. |
| `part_count` | How many of the allowed parts one damaged instance loses. |
| `chunk_volume_fraction` | Fused Meshy meshes have no parts to hide, so a cutter primitive seeded on the surface takes this fraction of the bounding volume instead. |
| `max_part_volume_fraction` | A part holding more than this share of the summed part bounding volumes never goes, so one draw cannot remove most of the robot. |
| `keypoint_clearance_m` | Keypoint anchors are protected with this clearance. `[robots.keypoints]` are model-frame offsets, so removing the part a keypoint sits on would leave the label pointing at thin air. |
| `protected_name_patterns` | Blender object-name substrings that must survive, for anything the geometry test misses. Empty means the geometric protection is the only rule. |
| `cutter_pool_size` | Cutters pre-built at startup, before the segmentation pass is armed. One scene needs at most one per robot-like instance: `[scene].max_robots_per_scene` plus the distractor pool's robot-like share. |

## `training/synthetic/config_meshy_grade.toml`

The Exp 1 spec for `training/synthetic/synthgen`: four Meshy-generated opponents plus the mrs_buff robot, tuned to reproduce the old synthetic keypoint set (derivation in `docs/experiments/perception/meshy_grade_2026-07-16.md`). The `[[robots]]` blocks, materials, keypoint coordinates and `ground_textures` match `config.toml`. Fields the dataclass tables above or the `config.toml` section already explain are not repeated. `num_images` is 35000, the old synthetic count; `--num-images` overrides it.

| Key | Meaning |
| --- | --- |
| `[output] annotation_mode` | `keypoints_bbox` (default) writes YOLO pose labels with bbox and keypoints. `segmentation_bbox` writes YOLO segmentation polygons, with the bbox implied by the polygons. This spec uses `keypoints_bbox`. |
| `[output] images_per_scene` | Camera viewpoints rendered per scene arrangement. Higher is faster generation with less object placement diversity. |
| `[output] min_robot_visibility` | Minimum visible fraction of robot pixels after occlusion: visible instance pixels (with distractors) over unobstructed instance pixels from a distractor-free segmentation pass. Ignored when `ignore_obstructions` is true. |
| `[output] ignore_obstructions` | True annotates robots and keypoints straight through occluding objects. It skips the distractor-clearing pass, the clean visibility-fraction render and the per-keypoint depth occlusion test, marks every in-frame keypoint visible (flag 2) and drops no frame for occlusion. Use it when the obstruction checks discard too many usable frames. |
| `[output] memory_cleanup_interval` | Forces periodic Blender data cleanup during long renders to stop memory growth. 0 disables it. |
| `[[robots]]` | Each entry is a robot model that can appear in a scene. Per scene, 1..`max_robots_per_scene` robots are drawn by weighted sampling without replacement. |
| `robots.scale` | Multiplier on the imported model (1.0 is no scaling). Raise or lower it when the export units do not match Blender meters. The Meshy GLBs are about 1.9 m native, so 0.133 gives about 0.25 m, matching mrs_buff. clyde is 1.0 because its native size (0.287) is already right. |
| `robots.weight` | Relative probability the robot is chosen for a scene. |
| `robots.ground_roll_upright`, `ground_roll_inverted` | Roll (Y-rotation, degrees) used when grounded. Pitch is fixed at +90 deg upright and -90 deg inverted, and yaw is always random. The roll compensates for a model that does not sit flat after the GLTF axis conversion. |
| `robots.keypoints` | 3D keypoint positions in model-local space, meters, measured from the CAD model origin in Onshape. The four Meshy robots copy theirs verbatim, after review, from `nhrl_<name>.json`. |
| `robots.color_mapping` | Onshape part colors (RGB 0-255) mapped to material types. `blenderproc run prepare_robot_model.py config.toml --inspect` lists every color in the GLTF. An empty list renders the GLB's baked Meshy texture; the key is required. |
| `[materials.*] cc_texture` | ambientCG (CC0 Textures) asset ID, tiled. `Foil002` is gold foil and `Foil003` is silver foil. |
| `[materials.*] texture_dir` | Directory of PBR images (Color, Roughness, Metalness, NormalGL, Displacement) applied through the model's UV mapping instead of tiling, so the UV layout is kept. |
| `[[distractors.sources]]` `weight` | Each source has a `path` and a relative `weight` controlling how often its models fill the pool. Weights need not sum to 1: weight 3.0 is three times as likely to contribute a model as 1.0. |
| `[distractors] max_per_scene` | Distractors per scene. Also sets how many distractor models are loaded into memory at once. |
| `[distractors] vram_budget_mb` | Approximate VRAM budget in MB across the loaded distractor pool. 0 or absent disables enforcement. |
| `[distractors] vram_audit_csv` | CSV of precomputed per-model VRAM estimates for the pre-import budget check. |
| `[distractors] base_dimension_m` | Largest dimension that distractor robots are scaled to. Every distractor robot is treated as 3 lb (beetleweight) regardless of its real weight class, so this is the 3 lb base size. It does not depend on the robots in the scene. |
| `[distractors] scale_range` | Multiples of `base_dimension_m`. `[0.5, 2.0]` gives 0.5x to 2x the 3 lb base size. |
| `[distractors] shuffle_interval` | Re-rolls the distractor pool every N images. 0 keeps one pool for the run. |
| `[distractors] robot_air_probability`, `robot_air_height_range` | Pose randomization for `kind = "cad"` distractor robots. Grounded ones sit flat with random yaw and a 50% chance of being inverted. Airborne ones tumble with a fully random orientation. Objaverse distractors use the global `[randomization]` air settings. |
| `[distractors] motion_blur_probability` | Per-frame blur probability for distractor categories. Defaults to `[randomization] motion_blur_probability` when omitted. |
| `[[distractors.sources]]` with `kind = "cad"` | Opponent robots as distractors, which auto-creates the `nhrl_robot` (class 2) label. This spec leaves it out to reproduce the old 2-class (our-robots-only) recipe. A 3-class set with opponents as a minority (the all_robot_keypoints goal) adds `path = "../data/distractor_models/robots"`, `kind = "cad"`, and a low weight such as 0.5. |
| `ground_textures` | Extra ambientCG assets for ground plane randomization. `download_ambientcg.py --from-config` downloads them with the robot materials. |
| `[camera] look_at_noise` | Gaussian noise on the camera look-at target. |
| `[camera] max_frame_fraction` | Largest fraction of the narrower image dimension the robots' bounding sphere may fill, range 0-1. The camera is pushed back past `min_distance` when needed so a robot never dominates the frame, which gives confusing near-full-frame annotations. Lower keeps robots smaller. |
| `[scene] ground_size_range` | Randomized range for the ground plane half-extent. |
| `[scene] arena_radius_range` | Randomized range for the radius within which objects are placed. |
| `[scene] ground_visibility` | Fraction of scenes with a visible ground plane, 0.0 to 1.0. The rest show only the HDRI behind the robots. This spec uses 1.0. |
| `[scene] max_robots_per_scene` | Robots rendered at once. Each scene draws between 1 and this value, weighted by each robot's `weight`, clamped to the number of `[[robots]]` entries. 1 means always one. |
| `[randomization] air_probability` | Probability the robot is airborne with a fully random tumbling orientation. The rest sit on the ground in a ground pitch pose. |
| `[randomization] air_height_range` | Height range in meters above ground when airborne. |
| ground pose | Ground pitch is fixed at +90 deg right-side-up and -90 deg upside-down. Roll is set per `[[robots]]` entry. Yaw is fully random. |
| `[randomization] motion_blur_probability` | Probability a frame gets directional motion blur on robot and distractor objects, 0.0 to 1.0. |
| `[randomization] motion_blur_strength_range` | Blur kernel size range, drawn per frame. |

## `training/synthetic/config_cage_*.toml`

Per-venue specs that extend `training/synthetic/config.toml` and pin one venue with `only_cage`. The robots, distractor pool, materials, damage block and mount ranges read off the 12 hand-flown poses all come from `config.toml`; only the scene mix differs. It is pinned in a file rather than passed on the command line so the render is reproducible from the file.

| Key | Meaning |
| --- | --- |
| `only_cage` | Every scene renders in that venue: no HDRI arena scenes and no other venue's scenes. Venues render separately so an arm can draw from one or both. `config_cage_massd.toml` uses `massd_arena`, `config_cage_nhrl.toml` (the NHRL 3 lb cage) uses `nhrl_cage`, and `config_cage_meatball.toml` uses the basement drive-test box. The massd and nhrl specs serve the domain-mix experiment in `docs/experiments/perception/synthetic_domain_mix_plan_2026-09-12.md`. |
| `[output] num_images` | 20,000 per venue for massd and nhrl, as the domain-mix experiment asks. 5,000 basement frames for the 2026-09-19 domain mix in `config_cage_meatball.toml`. `--num-images` overrides it, which suits probes. |

The basement scene and how it was fitted are in `playground/basement_scene/README.md`; its mount block is in `config.toml`.
