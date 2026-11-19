# Field pose

## Four-corner homography

`H = K [r1 r2 t]` up to scale, so `K^-1 H` recovers two rotation columns and the translation, and the third column is their cross product. The result is orthonormalized, because measured corners never satisfy the constraint exactly.

## Reprojection residual

The mean reprojection residual is in pixels.

- Four-corner path: it measures something real. The homography is exact, but it carries eight degrees of freedom against a pose that has six, and the two spare ones show up when K is applied: r1 and r2 come back non-orthonormal, orthonormalizing them moves the pose, and the corners no longer reproject onto themselves. A large value means the outline is not the image of a rectangle under these intrinsics. Measured on NHRL cage footage: 13 to 18 px on fits that look right, 61 px on one whose mask followed a sloped wall.
- Three-line path: it measures nothing. That solve builds an orthonormal rotation by construction and lands on about 0 whatever it was handed. `FieldOutline`'s structural guards, the per-side straightness test above all, are what catch a bad fit there.

## Field outline

`FieldOutline` is the field mask reduced to four image-plane sides. Corner i sits between side i-1 and side i, wound counter-clockwise from the smallest x + y. Sides 0 and 2 are opposite, as are 1 and 3. Side i runs `corners[i]` to `corners[i+1]`, which is what `pose_from_three_lines` relies on to pair sides with field edges.

A clipped outline is the expected case on a cage-mounted camera: the near mat corners fall outside an 87 degree horizontal field of view at any practical mount height. So an unsupported side is reported, not rejected, and `corners` may hold coordinates off the sensor. Callers decide what to do with three sides; `FieldPose` solves the pose from them directly.

`side_rms_px` is the scatter of each side's own points about its fitted line, empty for a side that was never fitted because it had too few points. It is reported, not acted on: a curved side still counts as support. It is the number that says a fit should be distrusted.

## Three-line solve geometry

At a cage mount the near mat corners fall outside the frame, so there is no four-corner fit to make. Three sides are still enough:

- The two opposite sides meet at the vanishing point of their shared world direction, which fixes one rotation column outright.
- The third side's backprojected plane is perpendicular to the second column, which fixes that column.
- The two finite corners, a known distance apart, fix the translation and the scale.

That is six constraints for six degrees of freedom, with the missing corners falling out as intersections outside the image.

`side_supported` must have exactly three entries true. `observed_corners` is read only for its winding: a rectangle mirrored about one of its own axes is the same rectangle, so a mirrored field frame satisfies all three lines exactly and only the winding separates it from the right answer.

## Field x alignment

`align_field_x_with_camera_x` turns the field frame about its own z so +x runs left to right in the camera view. A mat outline fits equally well at every quarter turn of a square, or every half turn of a rectangle, so the corner order alone leaves +x wherever the image winding put it. The function picks the candidate closest to the camera's x axis tipped onto the field plane, the same choice `PointCloudFieldFilter` makes, so the two filters agree on field left and right. Translation is unchanged: every candidate shares the centre.

## Pixel to field-plane projection

`pixels_to_field_plane` takes `(n, 2)` rectified pixels and returns `(n, 3)` field-frame points on the plane `z = plane_height_m`. It raises `ValueError` for any other pixel or intrinsics shape. A ray that misses the plane in front of the camera (parallel, or pointing at the sky) comes back as NaN.

## Synthetic field renders

`tests/field_filter/synthetic_field.hpp` renders a field mask from a known pose, so the solvers are checked against ground truth.

- `camera_pose(range, tilt_deg, yaw_deg)` looks at the field centre from `range` metres, `tilt_deg` off vertical: 0 is straight overhead, 90 would be level with the mat. Its field frame has z pointing into the floor, so up is -z. The outline fit points z at the camera instead, so compare rotations only after `kHalfTurnAboutFieldX`.
- `field_outline_points` rounds the corners in the world, not in the image, because the mat's rounding is physical.
- The outline winds like `FieldOutline::corners`: (-,-), (-,+), (+,+), (+,-). Each corner arc sweeps from the side arriving at the corner to the side leaving it, so points stay in winding order. A reversed outline flips `signed_area` and the solver rejects the candidate, which fails every synthetic test for the wrong reason. `tests/field_filter/test_synthetic_field.cpp` pins both.

## Fixed field filter

`FixedFieldFilter` reports a configured arena size and is used by the kinematic sim, where the field is known a priori and no perception runs. It gives navigation the wall bounds it needs without a RANSAC field fit. It takes the camera-from-fieldcenter transform straight from the sim, which places its camera in the same frame it reports robot poses in. An identity transform is not harmless: the UI projects robot markers and the field border through it, and identity divides by a robot's roughly 1 cm height instead of its metres of range, throwing every marker far outside the image.

## Three-line solve (`src/field_filter/field_pose.cpp`)

- Beside `const Eigen::Vector3d vanishing =`:
- Beside `const Eigen::Vector3d between_plane = intrinsics.transpose() * to_eigen(lines[between]);`:
- Beside `const Eigen::Vector3d field_x =`:
- Beside `const Eigen::Vector3d span = object_span.x * field_x + object_span.y * field_y;`:
