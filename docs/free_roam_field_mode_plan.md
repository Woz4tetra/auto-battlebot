# Plan: "Free roam" field mode (fit any plane, no segmentation model)

## Goal

Let the system find and track a ground plane anywhere ("free roam") without running the
DeepLab segmentation model. The camera points at a flat surface; the system fits that
plane. The fitted rectangle's size does not matter in this mode, it only needs to cover
the flat plane in view.

## Key insight

`PointCloudFieldFilter` uses the field mask for exactly one thing: selecting which depth
pixels feed the point cloud (`src/field_filter/point_cloud_field_filter.cpp:25-28`).
RANSAC (`fit_plane_ransac`, :226) already finds the dominant plane among whatever points
it receives. So there is no need for a new field filter, a plane-helper refactor, or a
runner change. Swap the mask source instead: add a mask model that selects
plane-covering pixels with cheap classical heuristics, and keep `PointCloudFieldFilter`
byte-for-byte unchanged.

This supersedes the earlier `FreeRoamFieldFilter` approach, which duplicated the
RANSAC-to-transform pipeline and required extracting shared helpers out of
`PointCloudFieldFilter`.

## Why not the existing FixedMaskModel

`FixedMaskModel` returns a 2x2 ones mask (`include/mask_model/fixed_mask_model.hpp:23`).
That satisfies the runner's empty-mask guard for the headless sim (paired with
`FixedFieldFilter`), but `PointCloudFieldFilter::mask_depth_image` does
`masked_depth.setTo(NaN, mask == 0)` (:178), which requires the mask to match the depth
image size. The new model must emit a mask at full image resolution.

## Approach

### 1. New mask model: `FreeRoamMaskModel`

`include/mask_model/free_roam_mask_model.hpp` + `src/mask_model/free_roam_mask_model.cpp`
(needs a `.cpp` for the OpenCV work; run `cmake -S . -B build` after adding it, the
source glob is not CONFIGURE_DEPENDS).

Two composable pixel selectors, ANDed together to form the mask:

**Center ROI** (spatial gate)
- Fill a centered rectangle covering `roi_fraction` of width and height with 255.
- `roi_fraction = 1.0` disables the spatial gate (full frame).

**Seeded color match** (`color_filter = true`)
- Sample a small center patch (`seed_patch_fraction` of the frame, ~10%) and take its
  per-channel median as the reference color. The camera is pointed at the plane, so the
  center patch is field material by contract. Median, not mean, so a robot or debris
  overlapping the patch does not skew the seed.
- Convert the frame to CIE Lab and threshold per channel against the seed:
  `|L - seed_L| <= tolerance_l`, `|a - seed_a| <= tolerance_ab`,
  `|b - seed_b| <= tolerance_ab`. Lab separates lightness from chroma, which is what
  makes a loose-but-useful threshold possible: shadows and lighting gradients mostly
  move L, so `tolerance_l` is set very loose (default 60 of 255) while `tolerance_ab`
  stays moderately loose (default 25). This also behaves on gray/neutral field material
  where HSV hue is undefined.
- Seeding happens on every `update()` call, so each field-init attempt adapts to
  current lighting. No stored state.
- Morphological close (fixed 5x5 kernel) to fill speckle holes. No config knob;
  downstream `find_largest_contour_mask` already discards disconnected islands, and
  RANSAC rejects any outlier pixels that survive, so the mask only needs to be roughly
  right. That is the justification for loose thresholds throughout.

Recommended pairing: `color_filter = true` with `roi_fraction = 1.0`. The color match
then extends the mask across the entire visible plane surface, not just a center crop,
which is better RANSAC coverage. The ROI gate is the fallback for pathological color
conditions (heavy glare, field color matching background walls).

Everything downstream works untouched: `find_largest_contour_mask` picks the biggest
connected color-matched region, `mask_depth_image` sees a size-matched mask, the
runner's `field_mask.mask.mask.empty()` guard (`src/runner.cpp:252`) passes, and
`track_field` keeps composing off visual odometry as it does today.

### 2. Config registration

- Add `FreeRoamMaskModelConfiguration` to `include/mask_model/config.hpp`:
  - `roi_fraction` (double, default 1.0, validate 0 < x <= 1)
  - `color_filter` (bool, default true)
  - `seed_patch_fraction` (double, default 0.1)
  - `tolerance_l` (double, default 60.0)
  - `tolerance_ab` (double, default 25.0)
  - `debug_visualization` (bool, default false): show seed patch, color mask, final
    mask, mirroring `YoloSegMaskModel`'s debug option. Useful when tuning tolerances.
- Register in `src/mask_model/config.cpp`: `REGISTER_CONFIG(MaskModelConfiguration,
  FreeRoamMaskModelConfiguration, "FreeRoamMaskModel")` alongside :60-63, plus a
  `make_mask_model` branch (:93) and the header include.

### 3. Config wiring

Add a free-roam overlay in the existing extends chain that overrides only the field
model:

```toml
[field_model]
type = "FreeRoamMaskModel"
roi_fraction = 1.0
color_filter = true
tolerance_l = 60.0
tolerance_ab = 25.0

# [field_filter] stays PointCloudFieldFilter with its existing RANSAC params
```

DeepLab never loads. No other section changes.

## Accepted tradeoffs

- The min-area rectangle traces the color-matched footprint on the plane, so `size` and
  yaw are arbitrary, not a real field boundary. Accepted: size does not matter in this
  mode. Downstream consumers of field size (target selector, robot filter, nav, UI)
  will see that footprint; confirm in playback that none of them misbehave on it.
- If the center seed patch lands on a non-field surface, the color match follows the
  wrong material. That is the mode's contract: point the camera at the surface you
  want.
- Color match can bleed onto same-colored non-plane surfaces (a gray wall behind a gray
  floor). RANSAC handles this: those pixels become plane outliers and are dropped. Only
  a same-colored surface with more area than the true plane can steal the fit, and the
  ROI gate covers that case.
- `compute_field` runs once at field initialization, not per tick, so the color pass
  and larger point cloud have no steady-state latency cost.

## Testing

Unit tests for `FreeRoamMaskModel` on synthetic images:

- ROI only (`color_filter = false`): mask matches input size, 255 inside the centered
  ROI, 0 outside, label FIELD, `roi_fraction = 1.0` fills the frame.
- Color match: uniform-color frame with a distinctly colored blob off-center; blob is
  excluded, field color included.
- Lighting robustness: same frame with a horizontal brightness ramp (simulated lighting
  gradient); loose `tolerance_l` keeps the whole ramp included.
- Seed robustness: distinct-colored blob overlapping part of the seed patch; median
  seed still matches the field color.
- ROI AND color: pixel must pass both gates.

No `PointCloudFieldFilter` test changes; it is untouched.

Playback regression: run an SVO with the free-roam overlay, confirm a plane is found
and tracked with DeepLab disabled and downstream projection still works.

## Validation

```bash
./scripts/build_and_test.sh --gtest_filter=FreeRoamMaskModel*
./scripts/lint
```

## Open questions

- Tolerance defaults (60 L, 25 ab) are guesses. Tune with `debug_visualization` on a
  real free-roam recording under varied lighting.
- Whether specular glare on the field (bright saturated highlights) blows past even a
  loose `tolerance_l`. If it does, the highlight region drops out of the mask, RANSAC
  still fits the rest of the plane, so likely harmless. Verify in playback.
- Fit-once + track (current behavior) is kept. If free-roam drifts because visual
  odometry degrades off-arena, add periodic re-fit later.
