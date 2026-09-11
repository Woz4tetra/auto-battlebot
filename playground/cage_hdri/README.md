# Cage HDRI from the 2024 ZED walk-around

Builds an equirectangular HDRI of the NHRL venue from
`data/svo/tests/1729981242.4295187.svo2`, the one recording that orbits the cage
(146 s, 1080p30, ZED 2i, 2024-10-26), then renders example BlenderProc frames
lit only by it. Outputs land under `runs/cage_hdri/`, which is gitignored.

Every stage writes something you can look at before running the next one.

## Pipeline

```bash
source scripts/activate_python.sh

# 1. Posed frames. Replays the SVO through ZED tracking with IMU fusion. ~3 min.
python playground/cage_hdri/extract_svo_walk.py \
    data/svo/tests/1729981242.4295187.svo2 runs/cage_hdri/walk --stride 3
#    -> frames/*.png, poses.npz, poses.csv, trajectory.png (check the loop closes)

# 2. Stitch. Quick look first, then the full build (~30 min at 8192, every 2nd frame).
python playground/cage_hdri/stitch_hdri.py runs/cage_hdri/walk runs/cage_hdri/hdri_quick \
    --width 2048 --every 3 --center-frame 3600
python playground/cage_hdri/stitch_hdri.py runs/cage_hdri/walk runs/cage_hdri/hdri \
    --width 8192 --every 2 --center-frame 3600
#    -> cage_walk.exr, cage_walk_preview.png, coverage.png, gains.csv, stitch_report.md

# 3. Example renders through the synthetic Docker image, point lights off.
playground/cage_hdri/render_examples.sh runs/cage_hdri/hdri runs/cage_hdri/render 12
```

`--center-frame 3600` puts the viewpoint where the camera stood beside the cage
looking up at the LED rig. Any SVO frame index works; pick one from
`trajectory.png` and `poses.csv`.

## How the stitch works

Each frame is placed on the sphere by its IMU-fused orientation alone, and every
output pixel takes its single highest-priority observation. Priority is a hat over
the source image times `exp(-d / --center-sigma)`, where `d` is that frame's camera
distance from the viewpoint, so frames shot near the viewpoint define the geometry
and far frames only fill what those missed. Per-frame exposure is solved from
overlap with neighbouring frames because the SVO carries no exposure value.

## What did not work, and why the stitch looks the way it does

- **Depth reprojection.** Unprojecting every pixel with ZED depth and splatting from
  one centre ghosts into arcs: the camera walks a 6.9 x 7.5 m loop, VIO heading
  drifts over 146 s, and per-pixel depth is noisy, so frames disagree by degrees.
  Averaging them blurs everything; picking one winner per pixel keeps each region
  sharp and turns the disagreement into seams. That is the visible seam pattern.
- **ZED spatial mapping.** The SDK mesh fused all 4,390 frames, but from the walk
  centroid it covered 55% of the sphere with holes and phantom glass surfaces, and
  its 2048x2048 texture for the whole venue is soft. Rendered as a panorama it was
  worse than the rotation-only stitch.
- **Coverage.** The camera never pointed at the zenith or nadir, so about half the
  sphere is push-pull filled with a smooth average of what surrounds it. The
  overhead contribution in renders is that fill, not the tubes directly above.

## Limits

- Frames are 8-bit and the LED tubes clip in almost all of them. An unclipped
  observation outranks a clipped one, and what is still clipped gets
  `--clipped-boost` (default 10). That is a physically plausible prior, not a
  measurement.
- Every frame was shot from outside the polycarbonate, so the cage interior
  appears through glass with its reflections.
- `stitch_hdri.py` normalises the median covered luminance to middle grey so
  Blender strength 1.0 is a sensible starting exposure.
