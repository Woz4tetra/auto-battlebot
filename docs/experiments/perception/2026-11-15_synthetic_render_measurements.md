# Synthetic render measurements

Timings and checks behind the synthgen render settings. The design facts live in `docs/reference/tools/synthgen_render.md` and `synthgen_scene_contracts.md`.

## Scene time by render setting (2026-09-12)

One NHRL cage scene, 10 frames at 1280x720, one A6000:

- BlenderProc's default Intel denoiser runs as a compositor node on the CPU and cost 6 s per scene.
- The occlusion-free re-render was a full-quality colour render that only had its instance segmap read. It cost 28 s per scene.
- The CPU sat in the Cycles device list next to the GPUs. Together these were half of the scene time.
- `occlusion_pass` cut the occlusion-free render to 3.9 s for ten frames instead of 28 s, with byte-identical labels.
- Removing the CPU from the device list took a scene from 50 s to 48 s on three A6000s.
- OptiX on the render device replaces the 6 s of CPU denoising per ten-frame scene for the same output.

## Render samples

64 samples with the spec's OptiX denoiser matched 128 samples with the compositor denoiser on 2026-09-12: 37.6 vs 39.9 dB PSNR against a 512-sample reference, with crops alike.

## Preview renderer

- The first Cycles frame costs about 1.3 s on an RTX 4080 Laptop. Every later one costs about 50 ms. `warm_up` pays the first one before the port opens.
- Hiding a distractor saves the time. A parked-but-visible pool of five models still syncs 5.5M polygons and 850 MB of textures into every pass. A hidden one costs nothing.

## Camera clearance from the glass (2026-09-12)

A camera within 0.055 m of the wall put an adjacent pane across a 118 degree frame edge-on. The rough refraction cost 402 s per render pass at 32 samples, where a healthy scene takes 4 to 23 s at 128. Both shipped mounts stay clear of the glass.

Both mounts stand outside the wall, close to it, with tilt derived from where they land and aimed short of the field centre. NHRL's come from the 12 poses flown in `pose_camera_server.py` on 2026-09-12. MassD's come from the coverage sweeps, pulled in once the half moved to the wider e-CAM25 lens. See `2026-11-19_synthgen_cage_mount_ranges.md`.

## Damage

- Protected part sets (2026-09-12, one robot, three poses): the set held at 9 and 36 parts in the robot's own frame, while world-space boxes wobbled between 6 and 71 as it rotated.
- Scene damage split (2026-09-12): a coin flip at 0.5 put one scene in ten on the damaged side of a 100-frame probe at ten frames per scene. That is inside its binomial spread and useless as a clean pool.
- Per-instance rolling at probability 0.35 left 15 percent of frames with every instance clean.
- The cut interior reads as exposed internals at 34 to 70 px when dark, rough and slightly metallic. With no material it rendered near white.
