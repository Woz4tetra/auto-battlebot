# synthgen render pipeline

Design facts for `training/synthetic/synthgen/` (`pipeline.py`, `robots.py`, `preview_render.py`). Blender is not required to test them: `blender_stubs` in `training/synthetic/tests/conftest.py` stubs the Blender modules.

## Camera poses and mounts

- `_scene_camera_poses` returns `ScenePoses(poses, cam_count, fallback_count, mounts)`.
- `mounts` holds one `CageMount` per pose, aligned by frame index. The manifest uses it to record which placement produced each frame.
- The HDRI half has no mounts: its cameras come off a sampled shell around the robots, not a wall. `SceneState` accepts zero mounts or exactly `cam_count` and raises `ValueError` otherwise.
- Clearing blocked keypoints moves distractors, never poses, so mounts stay aligned.

## Cages

- Every cage the scene is not using is deactivated first. Two cages share the world origin. A cage left showing puts its mat coplanar with the active one's and its light rig on top of the active one's, so the floor renders as both textures blended and washed out.

## Warped views

- Colour warps bilinear. Every other map at render resolution (category, instance, depth) warps nearest, so ids and depths are never blended. An unwarped view passes through untouched.
- `render_scenes.py --view` picks the frame every cage scene writes: `pinhole` at the rectified matrix, `distorted` as the sensor sees it, or `rectified` through the C++ Rectifier's maps. It overrides each `[[cages]]` view, and the HDRI arena half stays pinhole. When cages warp and the arena share is above zero, the pipeline logs a warning; pin the run to a cage (`--venue` or `only_cage`) to keep one view.

## Motion blur

- Robots blur at the `[randomization]` motion-blur probability. Distractors use the `[distractors]` one and fall back to the robot probability when it is unset.

## Manifest

- A fresh run (`start_index == 0`) replaces `manifest.jsonl`. A resume (`--start-index`, or a re-run into a directory that already holds images) appends to the rows already there.

## Segmentation pass indices

- `synthgen/pass_index.py` `assign_pass_index()` gives each new mesh object a pass index one above the highest index in `bpy.data.objects`. Every site that creates a mesh calls it right after `set_cp("category_id", ...)`: `environment.py`, `cage.py` (box, cylinder, quad, backdrop, house bot), `distractors.py`, `robots.py` (robot meshes and damage pieces) and `damage_scene.py` (cutters).
- Reason: BlenderProc 2.8.0 `RendererUtility.enable_segmentation_output` (line 469) numbers the existing meshes once with `obj.pass_index = index + 1`. `PostProcessingUtility` (line 340) maps the rendered index pass back to objects through `obj.pass_index`. A mesh created after that call has index 0, which is the background, so it drops out of the segmentation map.
- With the index set at creation, build order stops mattering and the distractor pool refresh no longer re-arms segmentation. A later `enable_segmentation_output` call renumbers every mesh and stays consistent.
- A damage piece is a Blender copy of its parent mesh and starts with the parent's index, so it needs its own.
- The tests fake `bpy` with `blender_stubs`, so `tests/test_pass_index.py` covers the numbering rule. Nothing here has rendered in Blender.

## First import in BlenderProc scripts

- `blenderproc run` calls `SetupUtility.check_if_setup_utilities_are_at_the_top` (`blenderproc/python/utility/SetupUtility.py`, line 389 in 2.8.0). It strips blank and `#` lines from the script and raises `RuntimeError` unless the first remaining line contains `import blenderproc` or `from blenderproc`.
- A docstring above the import fails the check. Shebangs and comments do not count.
- The reason is that BlenderProc's `__init__` installs the packages Blender's Python needs and removes the repo from `sys.path`, so third-party imports placed before it fail.
- Entry scripts keep `import blenderproc as bproc` on their first code line. `pyproject.toml` puts `blenderproc` in its own isort section ahead of the standard library, so `ruff` keeps that order without a skip comment.

## Robot import

- `ImportedRobot.bbox_corners` are the 8 corners of the robot's axis-aligned bounding box in parent-rest-pose world space. They give the ground clearance for arbitrary orientations.
- `RobotInstance.damage_pieces` maps a named damage part to its pieces. Each piece is the list of objects the load-time split cut for it.
- `RobotInstance` is compared by identity (`eq=False`): it holds numpy arrays and live Blender handles, and `robot in scene_robots` must mean the same robot object.

## Preview renderer

- Intrinsics are written on every mode transition, not only the first. `restore_render_state` puts the base camera back, so a cached setup would otherwise render at that base.
- Full renders set `use_persistent_data = False`. A stale Cycles cache must never reach a saved artifact. Preview renders keep it on.
- `warm_up` renders a throwaway frame so the OptiX kernel compile lands before the port opens, not on the first keypress.
- Preview renders scale with `resolution_percentage`, never by rescaling `K`. It is applied after projection, so the preview frames identically to the full render.
- Preview renders keep Cycles bounces at the spec's own values. The mat is lit through the polycarbonate panels, so truncating transmission bounces would make the preview systematically darker than the render it previews. Samples are the lever, and the denoiser buys back the noise.
- `RobotField` imports one robot and stands it at every mat sample point as linked duplicates. `obj.copy()` shares the mesh datablock, so the clones cost a transform each and materials come along for free. All clones share a heading, so projected widths in the readout compare like for like. They are placed once at construction, because moving geometry per frame would discard the Cycles persistent-data cache.

## Sharded renders

`synthgen/shards.py` is a pure module (no Blender) that `render_shards.py` uses to split a render over GPUs and views. Each GPU renders its shard as one `render_scenes.py` run per view. Every run writes its own directory, because `data.yml` and `manifest.jsonl` are per run and would race in a shared one. `merge_parts` hardlinks the finished runs into one flat dataset.

- The total splits evenly over the views, with the remainder going to the views in the order given. The caller picks which view runs short by putting it last.
- Each view's count splits over the shards, with the remainder starting one shard further along per view. Shard totals stay within one frame of each other.
- Every shard renders every view. A warped frame costs about twice a pinhole one, so a pinhole-only GPU would sit idle for half the render.
- `DEFAULT_VIEW_ORDER` is pinhole, rectified, distorted, so distorted runs short. NHRL keeps it. MassD passes `--views rectified distorted pinhole`, so the two venues run short on different views.

## Render settings

`synthgen/render_settings.py` holds the Cycles device and denoiser choices and the cheap settings for the occlusion-only pass. It requires Blender.

- `use_gpu_only` removes the CPU from the Cycles device list whenever a GPU renders. BlenderProc's init enables every device of the chosen type and Blender lists the CPU among the OptiX devices. A 1280x720 frame is small enough that the CPU's tiles finish after the GPU's and add wall time. A CPU-only run keeps the CPU.
- `occlusion_pass` is for a pass whose only read output is the segmap: one sample, no adaptive sampling, no denoiser, camera rays only. The object-index pass is decided by the first hit, so none of the colour work changes it. Everything is restored on exit so the next colour pass renders as configured.
- `_render_clean_inst_seg_maps` renders the occlusion-free instance segmentation with distractors hidden, under `occlusion_pass`. Distractors that `clear_blocking_distractors` already hid stay hidden afterwards, which is why visibility is saved alongside the transform.
- `ARENA_DENOISER` is OptiX for the HDRI arena half. A cage scene uses its spec's `render.denoiser`. OptiX runs on the render device, where BlenderProc's default Intel compositor node runs on the CPU.
- Preview mode sets `use_persistent_data = True` because the scene is static and only the camera moves. The batch pipeline turns it off because it mutates geometry per frame.

## Scene setup

- `render_scene` warps every map once, right after Blender renders, because Blender writes every map at render resolution and everything after works in the written view.
- `hide_distractor` hides a distractor and parks it far off-screen via its parent. Hiding is what saves the time, because a parked-but-visible pool still syncs into every pass. The park keeps it clear of the ray casts in `clear_blocking_distractors`.
- Damage goes on after posing and material jitter and comes off in the `finally` of `render_scene`. Both render passes see the same silhouette and the next scene gets the model back whole. Bboxes and keypoint visibility come from the segmentation of what rendered, so annotations need no special handling.
- `split_damage_parts` cuts a robot's meshes into the named pieces its `damage_parts` describe. A CAD GLB grouped by material colour holds one object per colour, so the weapon disk shares objects with every other part painted the same. Faces are assigned to pieces by centre. Each piece's faces move into their own object once, before materials and segmentation ids go on, so pieces render and label like any other mesh and damage never edits mesh data mid-run. It returns `(meshes, pieces)`: the mesh list including the pieces, and per part name the objects that received faces.
- `_UNDISTORT_CRITERIA` in `synthgen/lens.py` is 100 iterations at 1e-10. The `cv2.undistortPoints` default of five iterations leaves pixels of error at a wide lens's corners.
- `ALPHAS` in `synthgen/constants.py` are the two rectification alphas the pose camera server toggles. 1.0 keeps every source pixel, so the frame is wider than the lens and carries a black border. 0.0 crops to the largest all-valid rectangle, with no border and a narrower field. The C++ `Rectifier` ships 1.0.
- `HOUSE_BOT_CATEGORY_ID` (3) is the NHRL house bot parked on a cage rail. It is scenery, but it is robot-shaped and sits in frame in real cage footage, so it gets its own category and YOLO class rather than being left unlabelled for the detector to guess at.

## Materials and lighting

- `make_textured_material` gates the house bot texture on HSV saturation above `LED_SATURATION_THRESHOLD` (0.35) before scaling emission. The body texel is a slightly bluish grey (saturation about 0.1) and the LEDs are 0.6 and up.
- Faces a boolean cut opens take the cutter's material (`CUT_INTERIOR_MATERIAL_NAME`, material_mode TRANSFER). With no material they took slot 0 of the target through the primitive's own UVs and rendered near white. Dark, rough and slightly metallic reads as exposed internals at 34 to 70 px.
- `add_sky_lighting` and the sprite margin are covered in `synthgen_scene_contracts.md`.

## Cage scene construction (`synthgen/cage.py`)

- `solve_exposure()` compares a render to a target over the pixels both consider cage mat. A render without a mat category id (the sample renderer) falls back to the hull.
- `set_world()` sets a flat background colour, or the spec's panorama when `world.hdri` names one. Both go through the one `Background` node, and `set_world_background` unlinks any texture feeding it, so switching from a panorama venue back to a flat one needs nothing extra. A colour gain sits between the panorama and the `Background` node, so auto exposure tints the panorama the way it tints a flat background.
- `_add_backdrop()` drops the cap faces of the backdrop mesh so the tubes above still light the scene.
- `set_one_way_glass()` keyframes each pane out of the frames whose camera stands outside it.
- `add_house_bot_model()` places the textured house bot GLB on the mat, with its footprint scaled to the spec size. Every mesh of the model carries `category_id` so the segmentation pass boxes it as one object. The annotation itself is built in the pipeline.
- `make_textured_material()` pushes the LED colour further from grey with a hue/saturation node so it survives the exposure gain.

### House bot keypoints

`house_bot_keypoints()` returns the front and back keypoints of the placed house bot in its parent's local frame, in metres, or `None` when the model has no geometry. The convention matches every other robot in the set, the one `nhrl_common.keypoints_along_axis` applies to the CAD distractors: both points sit on the model's centerline at ground height, at the front and back extremes of its footprint. The axis is known, not fitted. The house bot's LED face is its own +x, so +x is the front, the way `HouseBotBoxSpec.roll_deg` and `yaw_deg` already treat it.

## Workarounds

Each entry names the function that carries the workaround. Entries marked unverified restate the claim the removed comment made; the source was not read or the failure not reproduced.

### Blender and BlenderProc

- Launch directory: BlenderProc changes the working directory to a temp directory and re-executes the script from there. `prepare_robot_model.py` and `render_sprites.py` read `BLENDERPROC_CWD` (falling back to `os.getcwd()`) at import and resolve relative CLI paths against it. Unverified: BlenderProc internals not read.
- `add_house_bot_model` (`synthgen/cage.py`): the GLB ships lying on its side. A roll about its x axis (the LED-face normal) stands it up, and the footprint is measured on the standing box. `size[0] <= 0` keeps the model's own dimensions. Unverified: asset not inspected.
- `_wire_maps_from_texture_directory` (`prepare_robot_model.py`): a loaded CC material can still be effectively flat in some Blender and asset combinations. The maps are then wired directly from the texture directory onto the slot material.

### Render state and denoising

- `set_denoiser` (`synthgen/render_settings.py`): `name` is a cage spec value, "OPTIX", "INTEL" or "none". OptiX needs an OptiX device, so a CUDA or CPU run gets OpenImageDenoise, which Blender 4.2 also runs on the GPU when one renders. "INTEL" maps to the same OpenImageDenoise, minus BlenderProc's compositor node and its CPU round trip. Going through BlenderProc first clears whatever denoiser was active, compositor node included.
- `OneWayGlass` (`synthgen/preview_render.py`): hides the pane the camera stands outside of, as the batch render does. Unlike `cage.set_one_way_glass` it writes `hide_render` directly instead of keyframing, because batch animates visibility across several poses per pass and preview shows one. The hidden set is rewritten only when it changes, because toggling visibility re-syncs geometry and discards Cycles' persistent-data cache. Unverified: cache behavior not measured.
- `panels_outside_camera` (`synthgen/cage_spec.py`): BlenderProc's segmentation stops at glass, so everything behind a pane the camera shoots through from outside the cage loses its label. Those panes are hidden. Panes seen from inside, across the arena, stay. The test is the wall plane, not the line of sight: a camera outside a wall but aimed away does not see that pane.

### Pipeline order (`synthgen/pipeline.py`)

1. `bproc.init()`, then devices and denoiser, resolution, samples, `enable_depth_output`, all before anything renders. Cycles keeps persistent data across the frames of one pass (BlenderProc's init turns it on), so the scene syncs once per pass rather than once per frame.
2. Per scene: reset keyframes, ground scale, environment, select and pose robots, pool refresh check, place distractors, lights, material jitter, cameras, render.
3. The clean (distractor-free) second render runs after the main render, only when obstructions are not ignored and distractors are active, at one sample. Only its segmap is read.
4. `frame_set(local_idx)` runs before per-frame keypoint projection, because `world_to_camera_view` reads the camera pose animated at that frame.

### Preview renderer (`synthgen/preview_render.py`)

- Views `pinhole`, `distorted` and `rectified` come from `synthgen.lens`, the module the batch pipeline writes its frames through, so a pose flown in one view renders in batch as it looked. `pinhole` scales `resolution_percentage`. Warped views build the lens at preview size and its maps index pixels.
- Mode switching writes a complete set of properties, never deltas. `restore_render_state` puts back the drift-prone half (color management, bounces, K, resolution) and the module writes the rest on top (engine, samples, denoising, persistent data, image format, resolution percentage). Preview and full mode go through the same transition, so neither inherits leftovers from the other.
- The camera is posed by assigning `matrix_world`, never `add_camera_pose`. Keyframes would accumulate on every WASD frame and a later render would walk the whole animation range. `animation_data` stays `None` for the life of the process, which `_assert_no_keyframes` checks on every frame. If it ever appears, direct pose assignment is silently overwritten and WASD stops responding.

### Docker image (`training/synthetic/docker/`)

- `entrypoint.sh` `link_baked_blender_into_home`: BlenderProc resolves Blender at `$HOME/blender`, and the image bakes it under `/root` (mode 0700). `/root` is made traversable and the install linked into the new home so the dropped user does not re-download Blender. Unverified: BlenderProc default path from its docs, not run.
- `entrypoint.sh` `hand_hf_cache_to_host_user`: `HF_HOME` is a baked, root-owned directory unless a host cache is mounted over it. The top level is chowned to the host user (a no-op for a host-owned mount) so the cache is writable after the privilege drop.
- `warmup.py`: `blenderproc pip install <pkg>` records only the named package (`install_default_packages=False`), so the installed-packages cache never lists BlenderProc's default runtime packages (pyyaml, imageio, gitpython, ...), and every runtime `blenderproc run` re-runs pip to check them ("Requirement already satisfied" spam). Running `blenderproc run` once at build goes through `setup_pip` with `install_default_packages=True` and records them in `<blender>/custom-python-packages/installed_packages_cache_v2.json`. The build step deletes any stale cache first. Unverified: `setup_pip` internals not read.

### Tools

- `discover_model_files` (`synthgen/asset_index.py`) skips any model whose JSON sidecar has `rejected: true`, the flag `review_nhrl_keypoints.py` sets.
