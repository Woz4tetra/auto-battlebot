# Plan: "Free roam" field mode (find any plane at the center, no segmentation model)

## Goal

Add a field-filter mode that finds a ground plane from the depth around the center of
the view, **without** running the semantic segmentation model. This lets the system
operate off-arena / anywhere ("free roam") where the DeepLab field mask would fail or
isn't wanted.

## Current behavior

The field is produced by two cooperating subsystems:

- **`field_model`** (`MaskModelInterface`, `include/mask_model/mask_model_interface.hpp`)
  — segmentation. Active impl `DeepLabMaskModel` (`config/_common.toml:20-21`) produces
  the field mask.
- **`field_filter`** (`FieldFilterInterface`,
  `include/field_filter/field_filter_interface.hpp`) — fits/tracks the plane.

Flow in `Runner::initialize_field` (`src/runner.cpp:246-262`):

```cpp
field_filter_->reset(camera_data.tf_visodom_from_camera);          // 248
MaskStamped field_mask = field_model_->update(camera_data.rgb);    // 249  segmentation runs
publisher_->publish_field_mask(field_mask, camera_data.rgb);       // 250
if (field_mask.mask.mask.empty()) { ... return; }                  // 252  early-out guard
initial_field_description_ = field_filter_->compute_field(camera_data, field_mask);  // 257
```

`PointCloudFieldFilter::compute_field` (`src/field_filter/point_cloud_field_filter.cpp:20-115`)
uses the mask only to choose which depth pixels feed the point cloud:

1. `find_largest_contour_mask(field_mask...)` (:25)
2. `mask_depth_image(depth, largest_contour_mask)` — NaN out non-field pixels (:26)
3. `create_point_cloud_from_depth(masked_depth, intrinsics)` (:27)

Then RANSAC (`fit_plane_ransac`, :226-266) → normal/center → `transform_from_plane`
(:310-336) → min-area rectangle for size/yaw → emits
`tf_camera_from_fieldcenter` (:72), the key downstream output.

The mask's **only** role is pixel selection. Everything after step 3 is reusable.

## Approach

Add a new `FreeRoamFieldFilter` implementation that replaces mask-based pixel selection
(steps 1-2) with a **center ROI selector**, and reuses the existing RANSAC → transform
→ rectangle pipeline. Pair it with a non-segmentation `field_model` so DeepLab never
runs.

### 1. New field filter implementation

Create `include/field_filter/free_roam_field_filter.hpp` +
`src/field_filter/free_roam_field_filter.cpp` (sources are auto-globbed —
`CMakeLists.txt:156-158` — so no CMake edit needed).

- Implement `FieldFilterInterface`. `compute_field(camera_data, field_mask)` ignores
  `field_mask` (`[[maybe_unused]]`, mirroring `FixedFieldFilter`/`NoopFieldFilter`).
- Select depth pixels in a window/radius around the principal point `(cx, cy)` (from
  `CameraInfo`, `include/data_structures/camera.hpp:12-18`) or `(width/2, height/2)`.
  Implement `select_center_roi_depth(depth, intrinsics, roi_params)` as the replacement
  for `mask_depth_image` — NaN out everything outside the ROI. Reuse
  `create_point_cloud_from_depth` unchanged.
- Reuse `fit_plane_ransac`, `plane_normal_from_coefficients`, `plane_center_from_inliers`,
  `transform_from_plane`, and the min-area-rectangle helpers. To avoid duplication,
  factor those out of `PointCloudFieldFilter` into a shared helper header
  (e.g. `plane_fit_utils.hpp`) or a shared base class, and have both filters call them.
- Emit the same outputs: `header.frame_id = FrameId::CAMERA_WORLD`,
  `child_frame_id = FrameId::FIELD`, `tf_camera_from_fieldcenter`, `size`, inliers.
  The non-EMPTY `frame_id` is the "plane found" signal the runner checks
  (`runner.cpp:258`).
- `track_field` can reuse `PointCloudFieldFilter`'s approach (compose stored
  `tf_visodom_from_cameraworld_` with the live camera transform, :117-135), or re-fit
  per tick if continuous free-roam re-detection is desired. Start with the compose
  approach for parity.

### 2. Config

- Add `FreeRoamFieldFilterConfiguration` to `include/field_filter/config.hpp` with a
  `PARSE_CONFIG_FIELDS` block: ROI params (`roi_radius_pixels` or
  `roi_width`/`roi_height`), plus the RANSAC params reused from
  `PointCloudFieldFilterConfiguration` (`distance_threshold`, `ransac_max_iterations`,
  `ransac_probability`, `depth_units_per_meter`).
- Add `REGISTER_CONFIG(FieldFilterConfiguration, FreeRoamFieldFilterConfiguration,
  "FreeRoamFieldFilter")` in `src/field_filter/config.cpp` (alongside :14-17) and a
  `make_field_filter` branch (:35).

### 3. Bypass the segmentation model in the runner

The runner unconditionally calls `field_model_->update` and returns early if the mask
is empty (`runner.cpp:249-255`). Two options:

- **Preferred:** pair `FreeRoamFieldFilter` with `[field_model] type = "FixedMaskModel"`,
  which returns a non-empty mask so the `field_mask.mask.mask.empty()` guard passes,
  while doing no real segmentation. No runner change; DeepLab never loads. Confirm
  `FixedMaskModel` returns a non-empty mask.
- **Alternative:** add a small guard in `initialize_field` to skip
  `field_model_->update` (and the empty-mask early-out) when the field filter declares
  it doesn't need a mask (e.g. a `needs_field_mask()` method on the interface,
  defaulting true). Cleaner separation but touches the interface and the runner.

Pick the preferred option first; fall back to the interface change only if a
non-segmentation mask model isn't viable.

### 4. Config wiring

Add a config (e.g. `config/free_roam.toml` or a profile) with:

```toml
[field_filter]
type = "FreeRoamFieldFilter"
roi_radius_pixels = 120
distance_threshold = 0.05

[field_model]
type = "FixedMaskModel"   # or whatever satisfies the runner guard
```

## Testing

- Unit test the center-ROI selector: synthetic depth image, verify only pixels within
  the ROI survive and out-of-ROI pixels become NaN.
- Unit test that a synthetic tilted plane in the ROI produces the expected
  `tf_camera_from_fieldcenter` normal/center (reuse patterns from any existing
  `PointCloudFieldFilter` tests in `tests/`).
- Playback regression: `./scripts/build_and_run.sh -c config/free_roam.toml` on an SVO,
  confirm a plane is found and tracked with DeepLab disabled, and that end-to-end
  projection downstream still works.

## Validation

```bash
./scripts/build_and_test.sh --gtest_filter=FreeRoamFieldFilter.*
git diff --name-only HEAD | grep '\.cpp$' | xargs -r clang-tidy -p build-test/
./scripts/apply_formatting
```

## Open questions

- ROI shape: circular radius vs. rectangular window around `(cx, cy)`. Radius is
  simplest and orientation-free — recommend starting there.
- Whether free-roam should continuously re-fit each tick (true "roaming") or fit once
  and track like the arena mode. Recommend fit-once + track for latency parity; add a
  `refit_period` option later if drift is a problem.
- Refactor scope: extracting the shared plane-fit helpers out of `PointCloudFieldFilter`
  is the cleanest path but touches an existing file — confirm that's acceptable vs.
  duplicating a small amount of fit code.
