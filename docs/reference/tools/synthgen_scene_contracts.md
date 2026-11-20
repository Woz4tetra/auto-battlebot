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

### Damage drawing (`synthgen/damage.py`)

`damage.py` decides what to remove without Blender. `damage_scene.py` applies it. Three mechanisms, chosen by what the robot's config and mesh allow:

- Named part removal, for a robot whose `[[robots.damage_parts]]` names its real assemblies (wheels, weapon disk, top plate). `split_damage_parts` cuts the model into those pieces at load and a draw removes whole assemblies, which looks like a robot that lost a fight where hiding a few bolts at a time did not. It skips keypoint protection, because the batch asked for those parts gone.
- Part removal, for CAD robots. `import_gltf_as_robot` returns the GLB's mesh objects, so those robots arrive already split.
- Chunk removal, for Meshy opponents. Their GLBs are usually one fused textured mesh, where part removal would either do nothing or delete the whole robot, so a cutter primitive seeded on the surface takes a bite out. `SURFACE_SAMPLE_LIMIT` (64) caps the surface vertices sampled as seeds, because a fused mesh can carry 100k vertices and the draw needs a handful.

Three things are protected, so damage never turns a label into a lie:

- A part whose bounding box contains a keypoint. `[robots.keypoints]` are model-frame offsets, and deleting what one sits on leaves it floating.
- A part big enough that removing it removes most of the robot.
- Anything matching a configured name pattern, for cases the geometry misses.

`damage` in the manifest is the fraction of mesh parts removed for the parts mechanisms and the fraction of bounding volume the cutter was sized to take for chunks. The scales differ, so `mechanism` is recorded beside the number. Draws use the module-level `random`, so `--seed` covers damage too.

`DamageBudget` tracks the damaged and clean scene split the way `choose_cage` tracks the scene mix: each scene goes to whichever side is behind its share. A run is only a handful of scenes, so a coin flip is useless for the clean pool.

`apply_scene_damage` decides per scene, not only per instance. A scene carries several robot-like instances, so rolling per instance alone left a frame with every instance clean at 15 percent of frames at probability 0.35, well short of the half a damage-off arm needs.

The two mechanisms work in different frames on purpose:

- Chunk removal works in world space, because the cutter is an unparented world object. A distractor's parent carries a per-scene scale (`place_scene_distractors` resizes each Meshy model to the 3 lb class), so a radius measured in the parent frame and applied in the world is off by that scale.
- Part removal works in the robot's parent frame, where the parts and keypoints were modelled. Bounding boxes there are axis-aligned to the robot, so the protected set does not depend on heading. The one length that does not survive the change is `keypoint_clearance_m`, which the caller divides by the parent's scale to keep it a clearance in metres.

### House bot pose (`synthgen/house_bot_pose.py`)

A pure module (no Blender) that draws where the house bot stands in one scene. The NHRL house bot drives around the cage during a fight, so a fixed pose would teach the detector it only sits in one corner facing one way. Each cage scene draws anywhere on the mat with the whole footprint on it, any heading, and clear of the robots already placed. `cage_scene.CageStage.place_house_bot` applies it.

`add_house_bot_model` adds a small area light on the camera side. The overhead tubes barely reach the box's vertical face, and the real venue lights it from the crowd side.

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

- The camera looks straight down, with +X right and +Y up in the image.
- `render_sprite` returns the rendered image's span in metres. That is the true scale of the file on disk, not the requested one, which pixel rounding perturbs.
- `add_sky_lighting` lights the scene from a sky-only outdoor HDRI. The sun gives the silhouette an edge and the sky fills the flat top surfaces, which is most of what a top-down sprite shows. Its rotation aims the sun, so it decides which side catches the highlight.
- `upright_yaw` returns the yaw that turns `front_local` onto +X once the model stands upright. The front and back keypoints are authored in the model's own frame. Standing the model up moves them, and the ground roll tilts them out of the horizontal plane, so the function takes the heading of the projection rather than the raw vector.

- The sprite manifest's `source` is repo-relative, so it names the same file whatever directory the render ran from.
