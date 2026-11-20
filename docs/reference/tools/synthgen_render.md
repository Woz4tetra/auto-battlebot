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

## Robot import

- `ImportedRobot.bbox_corners` are the 8 corners of the robot's axis-aligned bounding box in parent-rest-pose world space. They give the ground clearance for arbitrary orientations.
- `RobotInstance.damage_pieces` maps a named damage part to its pieces. Each piece is the list of objects the load-time split cut for it.
- `RobotInstance` is compared by identity (`eq=False`): it holds numpy arrays and live Blender handles, and `robot in scene_robots` must mean the same robot object.

## Preview renderer

- Intrinsics are written on every mode transition, not only the first. `restore_render_state` puts the base camera back, so a cached setup would otherwise render at that base.
- Full renders set `use_persistent_data = False`. A stale Cycles cache must never reach a saved artifact. Preview renders keep it on.
