# Field pose

## Four-corner homography

`H = K [r1 r2 t]` up to scale, so `K^-1 H` recovers two rotation columns and the translation, and the third column is their cross product. The result is orthonormalized, because measured corners never satisfy the constraint exactly.

## Reprojection residual

The mean reprojection residual is in pixels.

- Four-corner path: it measures something real. The homography is exact, but it carries eight degrees of freedom against a pose that has six, and the two spare ones show up when K is applied: r1 and r2 come back non-orthonormal, orthonormalizing them moves the pose, and the corners no longer reproject onto themselves. A large value means the outline is not the image of a rectangle under these intrinsics. Measured on NHRL cage footage: 13 to 18 px on fits that look right, 61 px on one whose mask followed a sloped wall.
- Three-line path: it measures nothing. That solve builds an orthonormal rotation by construction and lands on about 0 whatever it was handed. `FieldOutline`'s structural guards, the per-side straightness test above all, are what catch a bad fit there.

## Three-line solve (`src/field_filter/field_pose.cpp`)

- Beside `const Eigen::Vector3d vanishing =`:
- Beside `const Eigen::Vector3d between_plane = intrinsics.transpose() * to_eigen(lines[between]);`:
- Beside `const Eigen::Vector3d field_x =`:
- Beside `const Eigen::Vector3d span = object_span.x * field_x + object_span.y * field_y;`:
