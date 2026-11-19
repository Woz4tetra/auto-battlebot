# Migrating the field transform to an RGB homography

Status: **plan** (2026-09-09). Findings this rests on:
`field_transform_homography_2026-09-08.md`. Reference implementation:
`auto_battlebot/perception/field_pose.py`.

I am replacing the depth plane fit in `PointCloudFieldFilter::compute_field` with a
homography solved from the field outline in RGB and the known mat dimensions. This document
says why, what has to be true for it to work, and the steps to get it into C++.

## What changes

| | current `PointCloudFieldFilter` | new `HomographyFieldFilter` |
| --- | --- | --- |
| Input | field mask + depth image + intrinsics | field mask + intrinsics |
| Method | point cloud, RANSAC plane, flatten, min-area rectangle | four corners, `findHomography`, decompose with K |
| Field size | measured from the cloud | supplied from config |
| Cost | RANSAC over ~336k points | one 4-point solve |
| Fails when | depth is sparse or the plane is under-sampled | the field is clipped, or the outline is not a quadrilateral |

The two agree to **5.0 cm and 5.2 cm** in camera range on the two well-framed 2024-10-26
tripod recordings, with plane tilt inside 0.53 degrees and yaw inside 1.0 degrees. Against
synthetic ground truth both land within 3 to 9 mm.

## Why move off the depth path

1. **Depth is the expensive half and it is not needed.** The homography needs only the mask
   outline and K. That removes the RANSAC plane fit from field init.
2. **It works where depth does not.** Any fixed camera with a known mat can be calibrated,
   including the BrettZone `Cage-N-Overhead-High` feeds and MassD broadcast video, neither of
   which carries depth.
3. **The accuracy is there.** 5 cm agreement between two independent methods on real footage,
   3 to 9 mm against synthetic truth.

The trade is that the homography assumes the field size instead of measuring it, so it
inherits any error in that constant linearly. Get the number wrong by 4% and every range is
wrong by 4%.

## Three things that have to be right

### 1. The field size constant is the mat, not the cage

Bug: I used 2.4384 m (nominal 8 ft) and every range came out about 6% long.

Fix: use the floor mat. The mask covers the mat inside the cage, and the mat is smaller.

| source | field size |
| --- | --- |
| nominal 8 ft cage | 2.4384 m |
| depth path, measured | 2.303 to 2.374 m |
| the 2024 bags' own `/filter/field` | 2.361 to 2.402 m |

Measure this once per venue with the depth path and pin it in config. Do not read it off a
rulebook.

### 2. Corner extraction has to extrapolate through the rounded corners

Bug: `cv::approxPolyDP` returns vertices that lie on the outline, so it can only produce a
quadrilateral inscribed in the mask. The mat corners are physically rounded, so the fit
chorded across them and came out 19% short in area. That is 11% linearly, which put the
camera 11% too far away.

Fix: assign every contour point to whichever side of the current quad it is nearest, drop
the outer 20% of each side as belonging to the rounding, fit the survivors with total least
squares, and intersect adjacent lines. Four iterations. Mask area over quad area went from
1.24 to 0.97, and the range disagreement went from 22 cm to 5 cm.

`refine_quad_by_edges` in `auto_battlebot/perception/field_pose.py` is the reference.

### 3. Two guards, because residuals cannot see either failure

Reprojection error was **0.0 px on all three recordings**, before and after the corner fix.
The homography fits whatever four corners it is handed, however wrong they are. Both checks
are therefore structural:

- **Border contact.** If the mask reaches the image edge, the outline's corners sit where the
  field crosses the border, not where the field ends. On the clipped recording this cost
  14.8 cm and 11 degrees of tilt. Reject, do not fit.
- **Mask over quad area.** Above about 1.05 the outline is not the quadrilateral the method
  assumes. This caught all three recordings at 1.23 to 1.26 before the corner fix and clears
  at 0.96 to 0.98 after.

DeepLab is not on this list. Its outline tracks the mat and cage-frame boundary to within a
few pixels, and its 344x344 argmax upscaled to 1280x720 quantizes the boundary to 3.72 px
horizontally, which averages out across the several hundred points each edge fit uses.

## Steps to get this into C++

Everything below uses OpenCV and Eigen, both already linked into the field filter.

**1. Add `HomographyFieldFilter`.**
`include/field_filter/homography_field_filter.hpp` and
`src/field_filter/homography_field_filter.cpp`, implementing `FieldFilterInterface`.

**2. Reuse `track_field` unchanged.**
`PointCloudFieldFilter::track_field` only touches `tf_camera_from_fieldcenter` and `size`; it
never reads the cloud. Lift it and `reset` into a small shared base (both filters keep
`tf_visodom_from_cameraworld_`) rather than copying it. That keeps one definition of the
camera-world rebase that `field-mask-overlay-frozen-frames` already bit us on.

**3. Port corner extraction.** In `compute_field`:
   - `find_largest_contour_mask` already exists in `PointCloudFieldFilter`. Move it to a free
     function in the shared header so both filters call one copy.
   - `cv::convexHull`, then `cv::approxPolyDP` sweeping epsilon from 0.005 to 0.12 of the
     hull perimeter until it returns 4 points. That is the seed.
   - Iterative edge fit as in step 2 of the previous section. Total least squares per edge is
     `cv::SVD` on the mean-centred points; take the last row of `vt` as the direction.
   - Intersect adjacent lines with a 2x2 solve. Guard `|det| < 1e-9`.
   - Order the corners counter-clockwise from the smallest `x + y`. Winding must match the
     object corners or the recovered pose mirrors.

**4. Solve and decompose.**
   ```cpp
   cv::Mat H = cv::findHomography(object_corners, image_corners, 0);
   Eigen::Matrix3d h = K.inverse() * to_eigen(H);
   double lambda = 2.0 / (h.col(0).norm() + h.col(1).norm());
   Eigen::Vector3d r1 = h.col(0) * lambda, r2 = h.col(1) * lambda, t = h.col(2) * lambda;
   if (t.z() < 0) { r1 = -r1; r2 = -r2; t = -t; }   // field must be in front
   ```
   Build `[r1 r2 r1 x r2]`, orthonormalize with `Eigen::JacobiSVD` as `U * V^T`, and flip the
   last column if the determinant is negative. Averaging both column norms for `lambda` is
   deliberate: it is less sensitive to corner noise than trusting either alone.

   Object corners in the field frame are `(+-x/2, +-y/2)` from config, wound to match step 3.

**5. Leave `inlier_points` empty.**
   `FieldDescriptionWithInlierPoints::inlier_points` has exactly one consumer,
   `src/foxglove_adapters/scene.cpp:139`, and it already returns early on an empty cloud. No
   adapter change needed. The field rectangle still publishes.

**6. Add the config struct and register it.**
   In `include/field_filter/config.hpp`:
   ```cpp
   struct HomographyFieldFilterConfiguration : public FieldFilterConfiguration {
       double field_size_x = 2.35;          // mat, measured per venue
       double field_size_y = 2.35;
       double max_quad_coverage = 1.05;     // mask area / quad area
       int border_margin_px = 2;
       int refine_iterations = 4;
       double corner_skip_fraction = 0.20;
       HomographyFieldFilterConfiguration() { type = "HomographyFieldFilter"; }
       // PARSE_CONFIG_FIELDS(...) for each
   };
   ```
   Then `REGISTER_CONFIG(FieldFilterConfiguration, HomographyFieldFilterConfiguration,
   "HomographyFieldFilter")` in `src/field_filter/config.cpp:17`, plus a branch in
   `make_field_filter` alongside the existing three.

**7. Fail closed.**
   On border contact, coverage over `max_quad_coverage`, no 4-point approximation, or a
   degenerate intersection, return a default-constructed `FieldDescriptionWithInlierPoints`.
   `src/runner.cpp:261` already bails on an empty field mask and will not publish a bad
   field. Log which guard tripped at warn level; a silent bad pose is the failure mode this
   whole design is built to avoid.

**8. Switch the config over.**
   `config/_common.toml:18` currently sets the field filter type. Move `PointCloudFieldFilter`
   behind a per-venue override first and default `_common.toml` to `HomographyFieldFilter`
   only after step 9 passes.

**9. Verify on playback before trusting it.**
   ```bash
   ./scripts/build_and_test.sh --gtest_filter=HomographyFieldFilterTest.*
   ./scripts/build_and_run.sh -c config/playback/<recording>.toml
   ```
   Unit tests worth writing, all synthesizable without a recording:
   - A rendered square at a known pose recovers that pose within 10 mm. The Python synthetic
     harness covers overhead 3.0 m at 5 degrees through tripod 5.0 m at 45 degrees; mirror
     those cases.
   - A mask running off the image edge is rejected by the border guard.
   - A pentagonal mask is rejected by the coverage guard.
   - A mask with rounded corners recovers the full rectangle, not the inscribed one. This is
     the regression test for the bug in step 2 of the previous section.

   Then replay a recording that has both depth and a known mat and assert the two filters
   agree inside 10 cm. `playground/field_transform/compare_field_transform.py` already does
   this comparison offline and is the oracle to check the C++ against.

## Open risks

- **Every measurement so far is 3 frames from 3 recordings.** The 2024-10-26 bags never
  recorded the ZED image or depth streams; the only per-pixel data is a single
  `/camera_0/point_cloud/cloud_registered` per field init. Before switching the default,
  re-run the comparison on a recording set that carries depth per frame so the 5 cm number
  has more than three samples behind it.
- **The mat constant is per venue.** NHRL and MassD mats differ, and MassD's arena is wider
  than the broadcast frame, which trips the border guard outright. Plan on a per-venue config
  value and expect the guard to reject some venues entirely.
- **Yaw is ambiguous on a square mat.** A square is 90-degree symmetric and any rectangle is
  180-degree symmetric, so the recovered field frame can land in any of four orientations.
  The depth path has the same ambiguity. If downstream code ever depends on which corner is
  the origin, resolve it from the cage-corner assignment rather than from the fit.

## Next steps

1. Re-run `compare_field_transform.py` on a recording with per-frame depth to get more than
   3 samples behind the 5 cm agreement figure.
2. Measure the mat for each venue with the depth path and write the constants into config.
3. Implement steps 1 through 7, with the four unit tests from step 9.
4. Replay-compare both filters on the same recording, then flip `_common.toml`.
