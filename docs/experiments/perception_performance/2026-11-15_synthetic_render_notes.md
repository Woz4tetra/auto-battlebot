# 2026-11-15 synthetic render notes

Notes moved out of code comments during the comment purge. Each entry names the file and symbol it came from.

## Notes moved from code comments (2026-11-15)

- `training/synthetic/render_shards.py` (`above DEFAULT_VIEW_ORDER = (VIEW_PINHOLE, VIEW_RECTIFIED, VIEW_DISTORTED)`):
  The view that runs one frame short comes last. NHRL keeps this order; MassD passes `--views rectified distorted pinhole` so the two venues are short on different views.
- `training/synthetic/render_sprites.py` (`above SPRITE_MARGIN = 1.08`):
  Sky-only outdoor HDRI. Sun plus blue-sky fill lights a top-down view evenly; the three-area-lamp rig this replaced left the flat upper surfaces that dominate a top-down sprite reading as near-black.
- `training/synthetic/synthgen/cage.py`, `add_house_bot_model` (`above light = bproc.types.Light()`):
  The overhead tubes barely reach the box's vertical face; the real venue lights it from the crowd side. A small area light on the camera side stands in for that.
- `training/synthetic/synthgen/cage.py`, `make_textured_material` (`above gate = mat.new_node("ShaderNodeMath")`):
  The body texel is a slightly bluish grey (saturation ~0.1), the LEDs are 0.6 and up, so gate on saturation above a threshold before scaling.
- `training/synthetic/synthgen/cage.py` (`above LED_SATURATION_THRESHOLD = 0.35`):
  HSV saturation above which a house bot texel counts as a lit LED rather than grey steel.
- `training/synthetic/synthgen/constants.py` (`above ALPHAS: tuple[float, ...] = (1.0, 0.0)`):
  The two rectification alphas the pose camera server toggles between. 1.0 keeps every source pixel, so the frame is wider than the lens and carries a black border; 0.0 crops to the largest all-valid rectangle, so there is no border and a narrower field. The C++ `Rectifier` ships 1.0.
- `training/synthetic/synthgen/constants.py` (`above ARENA_DENOISER = "OPTIX"`):
  Denoiser for the HDRI arena half; a cage scene uses its spec's ``render.denoiser``. OptiX runs on the render device, where BlenderProc's default Intel compositor node cost 6 s of CPU per ten-frame scene for the same output.
- `training/synthetic/synthgen/constants.py` (`above HOUSE_BOT_CATEGORY_ID = 3`):
  The NHRL house bot parked on a cage rail. It is scenery, not a competitor, but it is robot-shaped and sits in frame in real cage footage, so it gets its own category and its own YOLO class rather than being left unlabelled for the detector to guess at.
- `training/synthetic/synthgen/damage.py`, `DamageBudget`:

  ```text
  Tracks the damaged/clean scene split so a short run still lands on the ratio.

  Damage is drawn per scene, so a run is a handful of draws, not thousands: a 100-frame
  probe at ten frames per scene is ten of them. A coin flip at 0.5 put one scene in ten
  on the damaged side on 2026-09-12, which is inside its binomial spread and useless as
  a clean pool. This tracks the split the way ``choose_cage`` tracks the scene mix: each
  scene goes to whichever side is behind its share, so the ratio holds at any length.
  ```

- `training/synthetic/synthgen/damage.py` (`above SURFACE_SAMPLE_LIMIT = 64`):
  Surface vertices sampled as candidate cutter seeds. A fused Meshy mesh can carry 100k vertices and the draw only needs a handful of candidates.
- `training/synthetic/synthgen/damage.py`:

  ```text
  Battle damage as a per-instance random variable: what to remove, decided without Blender.

  A real robot two minutes into a match is missing parts. Every synthetic robot so far is
  factory fresh, so the detector never sees a chewed one until the match does. This module
  draws the damage; ``synthgen.damage_scene`` applies it to the Blender scene and undoes it.

  Three mechanisms, chosen by what the robot's config and mesh structure allow:

  * **Named part removal**, for a robot whose ``[[robots.damage_parts]]`` names its real
    assemblies (wheels, weapon disk, top plate). A CAD export grouped by material colour has
    no part structure to hide, so ``synthgen.robots.split_damage_parts`` cuts the model into
    those named pieces once at load, and a draw removes whole assemblies. The random-fraction
    mechanism below hid a few bolts at a time, which never looked like a robot that lost a
    fight. Named removal skips the keypoint protection: the batch asked for those parts gone.

  * **Part removal**, for the CAD robots. ``import_gltf_as_robot`` returns the GLB's mesh
    objects, so those robots arrive already split into parts and a subset can simply go.
  * **Chunk removal**, for the Meshy opponents. Those GLBs are usually one fused textured
    mesh, where part removal would either do nothing or delete the whole robot, so a cutter
    primitive seeded on the surface takes a bite out instead.

  Three things are protected, because damage must not turn a label into a lie:

  * a part whose bounding box contains a keypoint, since ``[robots.keypoints]`` are
    model-frame offsets and deleting what one sits on leaves it floating in mid-air;
  * a part big enough that removing it removes most of the robot;
  * anything matching a configured name pattern, for cases the geometry misses.

  ``damage`` in the manifest means the fraction of the robot's mesh parts removed for the
  parts mechanism, and the fraction of bounding volume the cutter was sized to take for the
  chunk mechanism. They are not the same scale and are not comparable across mechanisms;
  ``mechanism`` is recorded beside the number so an arm can separate them.

  Draws use the module-level ``random`` like the rest of the pipeline, so ``--seed`` covers
  damage too.
  ```

- `training/synthetic/synthgen/damage_scene.py`, `apply_scene_damage` (`above scene_damaged = budget.take(cfg.scene_probability)`):
  Whole scenes are clean or not, which is what keeps the clean pool the size the arms need. Rolling only per instance would not: a scene carries several robot-like instances, so a frame with every one of them clean measured 15 percent of frames at probability 0.35, not the half a damage-off arm has to draw from. The split is tracked rather than flipped, because a run is only a handful of scenes.
- `training/synthetic/synthgen/damage_scene.py` (`above def _world_corners(obj: bpy.types.Object) -> np.ndarray:`):

  ```text
  The two mechanisms work in different frames, on purpose.

  Chunk removal works in WORLD space, because the cutter is an unparented world object: a
  distractor's parent carries a per-scene scale (place_scene_distractors resizes each Meshy
  model to the 3 lb class), so a radius measured in the parent frame and applied in the
  world comes out wrong by that scale factor.

  Part removal works in the robot's PARENT frame, because that is the frame the parts and
  the keypoints were modelled in. Bounding boxes there are axis-aligned to the robot, so
  which parts are protected does not depend on which way the robot happens to be facing;
  measured over three poses on 2026-09-12, the same robot's protected set held at 9 and 36
  parts in its own frame while world-space boxes wobbled between 6 and 71 as it rotated.
  The one length that does not survive the frame change is keypoint_clearance_m, which the
  caller divides by the parent's scale to keep it a clearance in metres.
  ```

- `training/synthetic/synthgen/damage_scene.py` (`above CUT_INTERIOR_MATERIAL_NAME = "damage_cut_interior"`):
  The faces a boolean cut opens take the cutter's material (material_mode TRANSFER). With no material they took slot 0 of the target through the primitive's own UVs and rendered near white, which no real robot's torn-open interior looks like. Dark, rough and slightly metallic reads as exposed internals and frame at 34 to 70 px.
- `training/synthetic/synthgen/distractors.py`, `hide_distractor`:
  Take a distractor out of the render: hidden, and parked far off-screen via its parent.
  Hiding is what saves the time. A parked-but-visible pool still syncs into every pass (5.5M polygons and 850 MB of textures for a five-model pool), while a hidden one costs nothing. The park keeps it clear of the ray casts in ``clear_blocking_distractors``.
- `training/synthetic/synthgen/house_bot_pose.py`:
  Where the house bot stands in one scene (pure module, no Blender).
  The NHRL house bot drives around the cage during a fight, so a fixed pose teaches the detector that it only ever sits in one corner facing one way. Each cage scene draws a new one: anywhere on the mat with its whole footprint on it, any heading, and clear of the robots already placed so the two never interpenetrate. `cage_scene.CageStage.place_house_bot` applies it.
- `training/synthetic/synthgen/lens.py` (`above _UNDISTORT_CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-10)`):
  undistortPoints' default five iterations leave pixels of error at a wide lens's corners.
- `training/synthetic/synthgen/pipeline.py`, `render_scene` (`above data = _warp_render_data(None if cage is None else cage.lens, bproc.renderer.rend...`):
  Every map comes out of Blender at render resolution; everything below works in the written view, so the warp happens once, here, before anything reads a pixel.
- `training/synthetic/synthgen/pipeline.py`, `render_scene` (`above damage_session, instance_damage = apply_scene_damage(`):
 , Battle damage, before anything renders, Damage goes on after posing and material jitter and comes off in the finally below, so both render passes see the same chewed silhouette and the next scene gets the model back whole. The annotations need no special handling: bboxes and keypoint visibility are computed from the segmentation of what actually rendered.
- `training/synthetic/synthgen/pipeline.py`, `_render_clean_inst_seg_maps`:
  Render an occlusion-free instance segmentation pass with distractors hidden.
  Only the segmap is read from this pass, so it renders under ``occlusion_pass``: one sample and camera rays only. Distractors that ``clear_blocking_distractors`` already hid stay hidden afterwards, which is why visibility is saved alongside the transform.
- `training/synthetic/synthgen/preview_render.py`, `RobotField`:

  ```text
  One imported robot, standing at every mat sample point via linked duplicates.

  The robot is 511 mesh parts. Importing it nine times builds 4599 objects with their own mesh
  data, which is enough to take the process down; `obj.copy()` shares the mesh datablock, so
  eight clones cost eight transforms and nothing else. Materials come along for free because
  they hang off the shared mesh.

  All nine share a heading, so the projected widths in the readout compare like for like. They
  are placed once, at construction: the sample points are fixed in the world, and moving
  geometry per frame would throw away Cycles' persistent-data cache.
  ```

- `training/synthetic/synthgen/preview_render.py`, `_build_setup` (`above lens = build_lens_view(self._calibration, self._render_size, view, alpha)`):
  Scale with resolution_percentage, never by rescaling K: it is applied after projection, so the preview frames identically to the full render by construction.
- `training/synthetic/synthgen/preview_render.py`, `_write_mode_settings` (`above cycles.use_persistent_data = True`):
  The scene is static and only the camera moves, so the cache is pure win here. The batch pipeline turns this off for the opposite reason: it mutates geometry per frame.
- `training/synthetic/synthgen/preview_render.py`, `_write_mode_settings` (`cycles.use_adaptive_sampling = False`):
  overhead with no payoff at 8 spp Bounces stay at the spec's own values, not a cheaper preview set. The mat is lit through the polycarbonate panels, so truncating transmission bounces makes the preview systematically darker than the render it is supposed to be previewing. Samples are the lever here; the denoiser is what buys back the noise.
- `training/synthetic/synthgen/preview_render.py`, `warm_up`:
  Render a throwaway frame so the OptiX kernel compile lands before the port opens.
  The first Cycles frame costs about 1.3 s on an RTX 4080 Laptop; every later one is about 50 ms. Paying it here keeps it off the user's first keypress.
- `training/synthetic/synthgen/render_settings.py`, `occlusion_pass`:
  Render settings for a pass whose only output that gets read is the segmap.
  One sample, no adaptive sampling, no denoiser and camera rays only: the object-index pass the segmap comes from is decided by the first hit, so none of the colour work changes it. Measured 3.9 s instead of 28 s for ten frames, with byte-identical labels. Everything is put back on exit so the next colour pass renders as configured.
- `training/synthetic/synthgen/render_settings.py`, `use_gpu_only`:
  Take the CPU out of the Cycles device list whenever a GPU is rendering.
  BlenderProc's init enables every device of the chosen type, and Blender lists the CPU among the OptiX devices. A 1280x720 frame is small enough that the CPU's tiles finish after the GPU's, so it adds wall time (50 s versus 48 s per scene on three A6000s). A CPU-only run keeps the CPU, since it is the only device there is.
- `training/synthetic/synthgen/render_settings.py`:

  ```text
  Cycles device and denoiser choices, and the cheap settings for the occlusion-only pass.

  Measured on 2026-09-12 (one NHRL cage scene, 10 frames at 1280x720, one A6000): BlenderProc's
  default Intel denoiser runs as a compositor node on the CPU and cost 6 s per scene, and the
  occlusion-free re-render was a full-quality colour render that only ever had its instance
  segmap read, 28 s per scene. Together with the CPU sitting in the Cycles device list next
  to the GPUs, these were half of the scene time. Everything here requires Blender.
  ```

- `training/synthetic/synthgen/robots.py`, `split_damage_parts`:

  ```text
  Cut a robot's meshes into the named pieces its ``damage_parts`` describe.

  A CAD GLB grouped by material colour holds one object per colour, so the weapon disk
  shares objects with every other part painted the same. Faces are assigned to pieces by
  centre, and each piece's faces move into their own object, once, before materials and
  segmentation ids go on, so the pieces render and label like any other robot mesh.
  Battle damage then hides whole pieces and never edits mesh data mid-run.

  Returns:
      ``(meshes, pieces)``: the robot's mesh list with the pieces in it, and per part name
      the pieces that received faces, each a list of objects.
  ```

- `training/synthetic/synthgen/shards.py`:

  ```text
  How a sharded render splits its frames over GPUs and views, and how the runs merge back.

  Pure module (no Blender), used by ``render_shards.py`` for step 3 of
  docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md. Each GPU renders
  its shard as one ``render_scenes.py`` run per view. Every run writes its own directory, because
  ``data.yml`` and ``manifest.jsonl`` are written per run and would race in a shared one, and
  ``merge_parts`` hardlinks the finished runs into one flat dataset.

  **Allocation.** The total splits evenly over the views, remainder to the views in the order given,
  so the caller picks which view runs short by putting it last. Each view's count then splits over
  the shards with its remainder starting one shard further along per view, which keeps the shard
  totals within one frame of each other too. Every shard renders every view because a warped frame
  costs about twice a pinhole one: a pinhole-only GPU would sit idle for half the render.
  ```

- `training/synthetic/tests/test_configuration.py`, `test_parses_shipped_config` (`above for mount in (nhrl.mount, massd.mount):`):
  Both mounts stay clear of the glass. A camera within 0.055 m of the wall put an adjacent pane across a 118 degree frame edge-on, and the rough refraction cost 402 seconds per render pass at 32 samples where a healthy scene takes 4 to 23 at 128.
- `training/synthetic/tests/test_configuration.py`, `test_parses_shipped_config` (`above for mount in (nhrl.mount, massd.mount):`):
  Both mounts stand outside the wall, close to it, with tilt derived from where they land and aimed short of the field centre. NHRL's come from the 12 poses flown in pose_camera_server.py on 2026-09-12; MassD's come from the coverage sweeps, pulled in once the half moved to the wider e-CAM25 lens.
- `training/synthetic/tests/test_configuration.py`, `test_parses_shipped_config` (`above assert nhrl.render_samples == 64`):
  64 with the spec's OptiX denoiser matched 128 with the compositor denoiser on 2026-09-12 (37.6 vs 39.9 dB PSNR against a 512-sample reference, crops alike).
