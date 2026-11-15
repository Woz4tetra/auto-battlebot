# Field pose math

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/field_filter/field_pose.cpp`

Beside `const Eigen::Vector3d vanishing =`:

The two opposite sides are parallel in the world, so they meet at the vanishing point of their shared direction. Homogeneous throughout, so a camera looking square-on, where that point runs off to infinity, needs no special case.

### `src/field_filter/field_pose.cpp`

Beside `const Eigen::Vector3d between_plane = intrinsics.transpose() * to_eigen(lines[between]);`:

Every point on the third side backprojects into a plane with this normal, and that side's own vanishing point lies on it, so the remaining field axis is perpendicular to both.

### `src/field_filter/field_pose.cpp`

Beside `const Eigen::Vector3d field_x =`:

The two field axes are an orthonormal basis, so the camera-frame image of field +x is its component along each.

### `src/field_filter/field_pose.cpp`

Beside `const Eigen::Vector3d span = object_span.x * field_x + object_span.y * field_y;`:

The two corners are a known distance apart along the third side, which is what fixes the scale a homography would have taken from four corners.
