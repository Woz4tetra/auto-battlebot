# 2026-11-15 basement scene pipeline

## Notes moved from code comments (2026-11-15)

### `playground/basement_scene/build_wall_textures.py`

The box stands in a corner of the basement: a stone wall 2.5 cm behind the far rail and
another 7 cm off the right rail. A panorama at infinity cannot stand in for walls that
close, so each one becomes a vertical plane in the scene (`[[walls]]` in the spec) with this
texture on it.

For each wall:

1. Plane from depth. The ZED points in a region behind the box are fitted to a vertical
plane; its offset is the median of the inliers. The fit also reports how far its normal
is from the box axis, which is the check that the wall really is square to the box.
2. Orthographic warp. The wall plane is mapped to the image through the fitted pose and K,
and the target is resampled onto a grid at `--px-per-m`, columns from `start` to `end`
and rows from `z_top` down, the layout `synthgen.cage_spec.wall_quads` expects.
3. Seen mask from depth, not from the picture. A texel counts as seen only if the ZED put
that pixel within `--wall-tolerance` of the plane, so the hoist in front of the far wall,
the rails and the bracket at the right edge are left out rather than painted onto it.
Points at or below the rail tops are dropped too: the far rail stands inside the
tolerance of the far wall.
4. Fill. The largest wholly seen rectangle, its lighting gradient flattened, is tiled over
the whole wall in shifted, cross-faded copies (`texture_fill.offset_tile`). Every seen
texel is then put back on top with a feathered edge, so the fitted view sees its own
photo and every other view sees the same stone continued. The seen texels have the
photo's broad lighting divided out first (`SHADING_SIGMA_M`): the scene's tubes light
the wall, and a baked hot spot would show as a bright patch against the fill.

Writes `<out>/<name>_albedo.png`, `_seen.png` (the depth mask) and `.json`, and prints the
`[[walls]]` block to paste into the spec.

Usage:
venv/bin/python playground/basement_scene/build_wall_textures.py runs/basement_scene \
--out training/data/environments/meatball_basement/walls

- W-frame regions whose depth points belong to each wall: above the rail tops so the box itself stays out, and clear of the corner so neither wall votes for the other. (at `FAR_REGION = {"axis": 1, "min": 0.78, "max": 1.2, "along": (-0.7, 0.7)`)
- Scale of the photo's lighting divided out of the seen texels, metres. The scene's own tubes light the wall; left in, the real hot spot shows as a bright patch against the tiled fill. (at `SHADING_SIGMA_M = 0.25`)

A plain blur would drag the unseen texels' placeholder value into the estimate along
every edge of the seen region; normalised convolution uses the seen ones alone.

- Both run away from the shared corner, so the texture's left edge meets at the corner. (at `z_bottom, z_top = -args.floor_drop, args.ceiling`)

### `playground/basement_scene/fit_basement_camera.py`

The box is a 5 ft plywood floor inside 2x4 rails stood on edge, pushed into a corner of the
basement's stone walls. The camera is our own ZED on a stand beside it, so unlike the MassD
broadcast the intrinsics are known (`camera_rect.json`, read from the SVO) and the SVO has
metric depth. Two steps:

1. Floor plane from depth. RANSAC over the XYZ of the plywood-coloured pixels, refined by
SVD. This fixes camera height and tilt without assuming anything about the box.
2. Box centre and yaw in that plane from the image. The rails give sharp edges: the floor
meeting the left, right and far rails, and the inner and outer top edges of all four.
Starting from the rail-top points in the depth, (x, y, yaw) is moved until those edges,
projected through K, sit on the Canny edges of the target (mean clipped distance). Depth
alone puts the right rail 5 cm off: the camera looks straight at its inner face and the
stereo smears it.

Box size is an input (`--mat-size`, `--rail-height`, `--rail-thickness`), not a fit: the
depth measured 1.515 x 1.530 m inside the rails and 0.0875 m rail height, which is a 5 ft
square and a 3.5 in 2x4 to within the stereo's noise.

Reads `<scene>/targets/<clip>/{target.png, xyz.npy, meta.json}` and `<scene>/camera_rect.json`.
Writes `<scene>/poses/<clip>.toml` (CageCalibration, W frame: floor centre, z up, +y away
from the camera), `<scene>/poses/<clip>_overlay.png`, `<scene>/poses/summary.csv`, and
`<scene>/targets/<clip>/hull_mask.png`: the floor the camera sees, rails cut out.

Usage:
venv/bin/python playground/basement_scene/fit_basement_camera.py runs/basement_scene

Percentiles of the extent would not do: the stone wall stands at rail-top height just
outside two of the rails and drags them outward.

