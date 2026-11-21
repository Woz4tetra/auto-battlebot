# MassDestruction camera fit details

## Target extraction (`playground/massd_scene/extract_target.py`)

Robot-free target frame from a MassDestruction broadcast clip, banners cropped off.

The Omega broadcast lays a title bar over the top of the picture and a sponsor bar over the bottom. Both are cropped here (the crop used by `data/downloads/mass_destruction/MANIFEST.md`: rows 96 to 982 of a 1080p stream), so the target, the hull mask and every K written downstream describe the same 1920x886 picture.

Unlike the NHRL cage-high flow this takes one hand-checked frame rather than a per-pixel median: the fights fill the whole clip, so the median still carries robot ghosts, while a between-round frame is genuinely empty. `--scan` ranks candidate frames by how little of the floor differs from the clip median and writes a contact sheet to choose from; the chosen index is then passed with `--frame`.

Output layout (matching playground/cage_scene so the cage-high tools read it unchanged):

```
    <out>/targets/<clip>/target.png      1920x886, banners cropped
    <out>/targets/<clip>/hull_mask.png   255 inside the cached floor hull
    <out>/targets/<clip>/meta.json       clip, event, source frame, crop
```

```
Usage:
    venv/bin/python playground/massd_scene/extract_target.py \
        data/downloads/massd_resurgence6_mrsbuff/r1_beeroll_vs_mrsbuff.mp4 \
        --frame 8140 --out runs/massd_scene
    venv/bin/python playground/massd_scene/extract_target.py <video> --scan 12
```

`--scan` ranks sampled frames by the floor fraction that differs from the clip median. Robots, brooms and hands all read as floor that moved. Thin objects score low, so the ranking narrows the search rather than deciding it; look at the contact sheet.

## Camera fit (`playground/massd_scene/fit_massd_camera.py`)

Fit the MassDestruction broadcast camera: focal length and pose, from the floor alone.

Nothing about this camera is calibrated. The clip is a YouTube broadcast, so the focal length has to come out of the picture with the pose. The floor gives it: the plywood is a square seen corner-on, so its two edge families meet at two vanishing points, and a principal point at the image centre with square pixels turns their orthogonality into one equation for f:

```
    (u1 - cx)(u2 - cx) + (v1 - cy)(v2 - cy) + f^2 = 0
```

The vanishing points come from a RANSAC over LSD segments on the floor, which locks onto the rails, the painted squares, the pit rim and the plywood seams: every straight line in the arena runs along one of the two axes. With f in hand the two axis directions fix the plane's orientation, and three floor edges (the two far ones and the near one opposite the far-left) fix the rest: their intersections are the far corner and the right corner, and the distance between them is the mat size, which sets the range.

The fit is exact by construction, like the NHRL three-line fit: six unknowns, six equations, no residual to report. Judge it on `<clip>_overlay.png` and on the sanity checks in `poses/summary.csv`.

Scale in: `--mat-size`, the plywood floor edge to edge. 2.26 m is the MassD arena, from an 8 ft (2.4384 m) wall interior less a 2x4 rail on each side, cross-checked two ways against our own metric ZED recording of the same venue (see the report).

```
Usage:
    venv/bin/python playground/massd_scene/fit_massd_camera.py runs/massd_scene
```
