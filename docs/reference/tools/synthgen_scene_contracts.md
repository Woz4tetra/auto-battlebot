# synthgen scene contracts

Companion to `synthgen_render.md`. How the Blender side of `training/synthetic/` poses cameras, damages robots, and annotates
distractors. Pure logic lives in `synthgen/` modules that import without Blender; the modules named
below need BlenderProc.

## Camera sampling (`synthgen/camera.py`)

`sample_camera_pose` never returns a pose where the look-at target is out of frame. When a
`robot_center` is given, it escalates in three steps:

1. Resample the look-at noise up to `CAMERA_TARGET_RETRIES` times until the center projects inside
   the written view.
2. Resample the camera geometry (up to `CAMERA_GEOMETRY_RESAMPLES` rounds), halving the noise each
   round.
3. Fall back to a deterministic zero-noise pose aimed straight at the center, which always frames
   it. `CameraSample.used_zero_noise_fallback` marks this case.

`setup_scene_cameras` returns `CameraPoses`, whose `zero_noise_fallback_count` is the number of
poses that took step 3.

## Cage mounts (`synthgen/cage_mount.py`)

`CageMount` fields and their conventions:

| Field | Meaning |
| --- | --- |
| `along_from_centre_m` | Runs along the wall from its centre, positive toward the wall's own +along axis (see `wall_axes`) |
| `inset_toward_centre_m` | How far inside the wall plane the lens sits, positive toward the mat centre |
| `tilt_from_down_deg` | Optical axis angle off straight down |
| `yaw_from_square_deg` | Heading away from square-to-the-wall |
| `roll_about_axis_deg` | Camera roll about the optical axis |

`CageMountRanges` keeps the range names (`along_m`, `inset_m`, `tilt_deg`, `yaw_deg`, `roll_deg`)
that the TOML `[cages.mount]` block, the pose page and the preview server JSON use.
`MOUNT_FIELD_BY_RANGE_NAME` maps each range name to its `CageMount` field.

`CageStage.sample_camera_poses` returns `MountPoses`. `unframed_fallback_count` counts poses that
ran out of `MOUNT_FRAMING_RETRIES` and kept the last sample regardless of framing.

## Damage (`synthgen/damage_scene.py`)

Damage never edits mesh data. `load_robots` and the distractor pool load every model once and reuse
it for every later scene, so a deleted part would stay deleted for the rest of the run, and a
boolean applied to mesh data would compound across scenes. Instead:

- Part removal hides the chosen parts from the render. The colour pass and the segmentation pass get
  the same silhouette deleting them would give. Named part removal hides the pieces
  `split_damage_parts` cut at load.
- Chunk removal borrows a cutter from a pool built once at startup and attaches a boolean
  difference modifier to the target mesh.

Both are recorded in a `DamageSession` and reverted after the scene's frames are written. The
cutter pool is pre-built because objects created mid-run would miss the segmentation arming that
`_enable_segmentation` does over the meshes that exist at call time. Pool cutters are created before
it, never render, and carry the background category id, so both passes ignore them.

`apply_scene_damage` returns `SceneDamage`. Only robot-like distractors are damaged, since a cut-up
traffic cone teaches the detector nothing about a battlebot. `per_instance` has one entry per
instance considered, damaged or not, robots first. Every entry goes in the manifest, so an arm can
filter on frames whose instances are all undamaged.

## Distractor keypoint annotations (`synthgen/keypoints.py`)

`build_distractor_keypoint_annotations` never discards a frame. The clean segmentation pass hides
distractors, so there is no unobstructed reference to gate visibility on. Only the (occluded)
instance bbox size and the per-keypoint depth checks apply, and a distractor that fails either is
counted in `RunStats.distractor_skip_counts`.

## Sprites (`render_sprites.py`)

`upright_matrix` copies the pose `pose_single_robot` gives a grounded robot: a -90 degree pitch for
upright (+90 for inverted) and the per-model `ground_roll_upright` / `ground_roll_inverted` roll
that takes out the tilt left by where the CAD origin sits. Matching both keeps the sprite footprint
equal to the robot's real footprint on the floor, so the sim viewer draws the same outline the
detector saw.
