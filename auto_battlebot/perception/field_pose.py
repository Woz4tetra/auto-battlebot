"""Two ways to get the camera <-> field transform, for comparison.

`depth_plane_field` is a port of the shipped C++ path in
`src/field_filter/point_cloud_field_filter.cpp` (`PointCloudFieldFilter::compute_field`):
mask -> depth -> point cloud -> RANSAC plane -> flatten -> minimum-area rectangle. It
measures the field size rather than assuming it, and needs a depth image.

`homography_field` is the new one: the same mask, but only its outline in the RGB frame,
solved against a field of known metric size. Four image corners plus four object corners
give a homography, and with the intrinsics that decomposes straight into a pose. No depth.

Both return a 4x4 tf_camera_from_fieldcenter, so they are directly comparable.

A square field is 90-degree ambiguous in yaw for both methods, and 180-degree ambiguous
even when rectangular, because a bare rectangle carries no handedness. Compare yaw modulo
that symmetry; `yaw_difference_deg` does.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np
from scipy.optimize import least_squares

# Defaults mirror config/_common.toml and include/field_filter/config.hpp.
DISTANCE_THRESHOLD_M = 0.05
RANSAC_MAX_ITERATIONS = 1000
RANSAC_PROBABILITY = 0.999
# transform_from_plane's default up_vector in the C++ header.
UP_VECTOR = np.array([0.0, 0.0, -1.0])
# Mask-area / quad-area above this means the outline is not the quadrilateral the
# homography assumes, so its corners are not the field's corners.
QUAD_COVERAGE_TOLERANCE = 1.05


@dataclass
class FieldResult:
    """One method's answer for one frame."""

    method: str
    tf_camera_from_field: np.ndarray | None  # 4x4
    size_xy_m: tuple[float, float] | None
    corners_image: np.ndarray | None = None  # 4x2, for overlays
    plane_normal: np.ndarray | None = None
    inlier_count: int = 0
    notes: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.tf_camera_from_field is not None


# --------------------------------------------------------------------------- shared


def largest_contour_mask(mask: np.ndarray) -> np.ndarray:
    """Largest external contour, filled. Port of find_largest_contour_mask."""
    binary = (mask > 0).astype(np.uint8)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = np.zeros_like(binary)
    if not contours:
        return out
    largest = max(contours, key=cv2.contourArea)
    cv2.drawContours(out, [largest], -1, 255, cv2.FILLED)
    return out


def normalize_angle(angle: float) -> float:
    """Wrap to [-pi, pi]. Port of normalize_angle / input_modulus."""
    modulus = 2.0 * math.pi
    value = math.fmod(angle + math.pi, modulus)
    if value < 0.0:
        value += modulus
    return value - math.pi


def rectangle_extents(corners: np.ndarray) -> tuple[float, float]:
    """Port of get_rectangle_extents: |c0-c1| and |c1-c2|."""
    width = float(np.linalg.norm(corners[0] - corners[1]))
    height = float(np.linalg.norm(corners[1] - corners[2]))
    return width, height


def rectangle_angle(corners: np.ndarray) -> float:
    """Port of get_rectangle_angle.

    Takes the edge whose midpoint has the smallest y, then flips to the complementary
    angle when that is closer to zero. Reproduced exactly because the sign of this angle
    drives the extents swap in compute_field, and a different convention would rotate the
    field frame by 90 degrees.
    """
    if len(corners) != 4:
        return 0.0
    angles, midpoints = [], []
    for i in range(4):
        p0, p1 = corners[i], corners[(i + 1) % 4]
        delta = p1 - p0
        angles.append(math.atan2(float(delta[1]), float(delta[0])))
        midpoints.append((p0 + p1) / 2.0)
    min_y_index = int(np.argmin([m[1] for m in midpoints]))
    angle = angles[min_y_index]
    complementary = normalize_angle(angle + math.pi)
    return complementary if abs(complementary) < abs(angle) else angle


def rectangle_centroid(corners: np.ndarray) -> np.ndarray:
    return np.asarray(corners.mean(axis=0))


def transform_from_position_and_euler(position: np.ndarray, yaw: float) -> np.ndarray:
    """Port of transform_from_position_and_euler with roll = pitch = 0."""
    out = np.eye(4)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    out[:3, :3] = np.array([[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]])
    out[:3, 3] = position
    return out


def transform_from_plane(
    center: np.ndarray, normal: np.ndarray, up: np.ndarray = UP_VECTOR
) -> np.ndarray:
    """Port of transform_from_plane: rotate `up` onto `normal`, translate to `center`.

    Eigen's Quaterniond::FromTwoVectors is the shortest-arc rotation, which is what the
    cross-product construction below reproduces.
    """
    a = up / np.linalg.norm(up)
    b = normal / np.linalg.norm(normal)
    out = np.eye(4)
    out[:3, 3] = center
    axis = np.cross(a, b)
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-12:
        # Parallel or antiparallel: identity, or a pi rotation about any perpendicular axis.
        if dot > 0:
            return out
        perp = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            perp = np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, perp)
        axis /= np.linalg.norm(axis)
        rotation, _ = cv2.Rodrigues(axis * math.pi)
        out[:3, :3] = rotation
        return out
    axis = axis / axis_norm
    rotation, _ = cv2.Rodrigues(axis * math.atan2(axis_norm, dot))
    out[:3, :3] = rotation
    return out


# ------------------------------------------------------------------- depth + plane


def point_cloud_from_depth(
    depth_m: np.ndarray, intrinsics: np.ndarray, mask: np.ndarray | None = None
) -> np.ndarray:
    """Pinhole unprojection. Port of create_point_cloud_from_depth. Returns (n, 3)."""
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if mask is not None:
        valid &= mask > 0
    vs, us = np.nonzero(valid)
    z = depth_m[vs, us].astype(np.float64)
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def fit_plane_ransac(
    points: np.ndarray,
    distance_threshold: float = DISTANCE_THRESHOLD_M,
    max_iterations: int = RANSAC_MAX_ITERATIONS,
    probability: float = RANSAC_PROBABILITY,
    rng: np.random.Generator | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """RANSAC plane fit, standing in for pcl::SACSegmentation SACMODEL_PLANE.

    Returns (a, b, c, d) with the normal unit length, and the inlier index array. The
    adaptive iteration cap follows the same 1 - (1 - w^3)^k rule PCL uses.
    """
    rng = rng or np.random.default_rng(0)
    n = len(points)
    if n < 3:
        return np.zeros(4), np.empty(0, dtype=int)

    best_inliers = np.empty(0, dtype=int)
    best_model = np.zeros(4)
    iterations = max_iterations
    i = 0
    while i < min(iterations, max_iterations):
        i += 1
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        normal = normal / norm
        d = -float(np.dot(normal, p0))
        distances = np.abs(points @ normal + d)
        inliers = np.nonzero(distances <= distance_threshold)[0]
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_model = np.array([normal[0], normal[1], normal[2], d])
            w = len(inliers) / n
            if 0.0 < w < 1.0:
                denom = math.log(max(1e-12, 1.0 - w**3))
                iterations = min(max_iterations, int(math.log(1.0 - probability) / denom) + 1)

    if len(best_inliers) >= 3:
        # Least-squares refit on the inliers, as PCL does after the RANSAC vote.
        inlier_points = points[best_inliers]
        centroid = inlier_points.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
        normal = vh[2] / np.linalg.norm(vh[2])
        d = -float(np.dot(normal, centroid))
        best_model = np.array([normal[0], normal[1], normal[2], d])
        distances = np.abs(points @ normal + d)
        best_inliers = np.nonzero(distances <= distance_threshold)[0]
    return best_model, best_inliers


def depth_plane_field(
    mask: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: np.ndarray,
    distance_threshold: float = DISTANCE_THRESHOLD_M,
    seed: int = 0,
) -> FieldResult:
    """The shipped C++ pipeline, in Python. Measures the field size from the cloud."""
    contour_mask = largest_contour_mask(mask)
    if not contour_mask.any():
        return FieldResult("depth_plane", None, None, notes="empty field mask")

    cloud = point_cloud_from_depth(depth_m, intrinsics, contour_mask)
    if len(cloud) < 3:
        return FieldResult("depth_plane", None, None, notes="no valid depth under mask")

    coefficients, inliers = fit_plane_ransac(
        cloud, distance_threshold, rng=np.random.default_rng(seed)
    )
    if not coefficients.any() or len(inliers) < 3:
        return FieldResult("depth_plane", None, None, notes="plane fit failed")

    inlier_cloud = cloud[inliers]
    normal = coefficients[:3]
    center = inlier_cloud.mean(axis=0)

    plane_transform = transform_from_plane(center, normal)
    flat = (np.linalg.inv(plane_transform) @ np.c_[inlier_cloud, np.ones(len(inlier_cloud))].T).T
    flat_2d = flat[:, :2].astype(np.float32)

    rect = cv2.minAreaRect(flat_2d)
    corners = cv2.boxPoints(rect).astype(np.float64)

    extents = rectangle_extents(corners)
    angle = rectangle_angle(corners)
    if angle > 0:
        extents = (extents[1], extents[0])
    centroid = rectangle_centroid(corners)

    flat_transform = transform_from_position_and_euler(
        np.array([centroid[0], centroid[1], 0.0]), angle
    )
    tf = plane_transform @ flat_transform
    return FieldResult(
        "depth_plane",
        tf,
        (float(extents[0]), float(extents[1])),
        plane_normal=normal,
        inlier_count=int(len(inliers)),
        extra={"cloud_points": int(len(cloud)), "rect_angle_rad": float(angle)},
    )


# ----------------------------------------------------------------------- homography


def touches_border(mask: np.ndarray, margin: int = 2) -> bool:
    """True when the mask reaches the image edge, so the field is cut off.

    This matters only for the homography method. It solves for a pose from four corners,
    and if the cage runs out of frame the outline's corners sit where the field crosses
    the image border, not where the field actually ends. The fit still looks perfect
    (reprojection error near zero) while the pose is wrong, so the check has to be
    structural rather than residual-based.
    """
    binary = mask > 0
    return bool(
        binary[:margin, :].any()
        or binary[-margin:, :].any()
        or binary[:, :margin].any()
        or binary[:, -margin:].any()
    )


def order_corners(corners: np.ndarray) -> np.ndarray:
    """Counter-clockwise from the top-left in image coordinates.

    A consistent winding is what makes the homography's rotation reproducible; without it
    the recovered yaw jumps by multiples of 90 degrees frame to frame.
    """
    centre = corners.mean(axis=0)
    angles = np.arctan2(corners[:, 1] - centre[1], corners[:, 0] - centre[0])
    ordered = corners[np.argsort(angles)]
    start = int(np.argmin(ordered.sum(axis=1)))  # smallest x+y is top-left
    return np.roll(ordered, -start, axis=0)


def _fit_line(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Total-least-squares line as (unit normal, offset), with normal . x = offset."""
    centroid = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - centroid)
    direction = vh[0]
    normal = np.array([-direction[1], direction[0]])
    return normal, float(np.dot(normal, centroid))


def _fit_side_lines(
    points: np.ndarray, quad: np.ndarray, corner_skip: float
) -> list[tuple[np.ndarray, float]] | None:
    """One line per quad side, fitted to the contour points nearest that side."""
    edges = [(quad[i], quad[(i + 1) % 4]) for i in range(4)]
    units, lengths = [], []
    for a, b in edges:
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length < 1e-6:
            return None
        units.append(edge / length)
        lengths.append(length)

    # Perpendicular distance to each edge's infinite line, and position along it.
    perp = np.empty((len(points), 4))
    along = np.empty((len(points), 4))
    for i, (a, _) in enumerate(edges):
        rel = points - a
        along[:, i] = rel @ units[i] / lengths[i]
        perp[:, i] = np.abs(rel @ np.array([-units[i][1], units[i][0]]))
    owner = np.argmin(perp, axis=1)

    lines = []
    for i in range(4):
        mine = (owner == i) & (along[:, i] > corner_skip) & (along[:, i] < 1.0 - corner_skip)
        if mine.sum() < 20:
            return None
        selected = points[mine]
        normal, offset = _fit_line(selected)
        # One robust pass so a robot resting on the floor edge cannot pull the line in.
        residual = np.abs(selected @ normal - offset)
        keep = residual <= max(2.0, 2.5 * float(np.median(residual)))
        if keep.sum() >= 20:
            selected = selected[keep]
        lines.append(_fit_line(selected))
    return lines


def _intersect_side_lines(lines: list[tuple[np.ndarray, float]]) -> np.ndarray | None:
    """Corners of the quad whose sides are `lines`, in `order_corners` order."""
    corners = []
    for i in range(4):
        (n1, d1), (n2, d2) = lines[i], lines[(i + 1) % 4]
        matrix = np.stack([n1, n2])
        if abs(np.linalg.det(matrix)) < 1e-9:
            return None
        corners.append(np.linalg.solve(matrix, np.array([d1, d2])))
    # Intersections land in edge order, so corner i sits between edges i-1 and i.
    return np.roll(np.array(corners), 1, axis=0)


def refine_quad_by_edges(
    boundary: np.ndarray, seed: np.ndarray, iterations: int = 4, corner_skip: float = 0.20
) -> np.ndarray | None:
    """Fit the four field edges to the mask outline and intersect them.

    `approxPolyDP` returns vertices that lie *on* the outline, so it can only ever produce
    a quad inscribed in the mask. Where the floor mat has rounded corners that quad chords
    across them and comes out about 19% small in area on this footage, which is 11%
    linearly and puts the camera 11% too far away.

    The straight middle stretch of each side is the real field edge, so each contour point
    is assigned to whichever side of the current quad it is nearest, points within
    `corner_skip` of either end are dropped as belonging to the rounding, and the surviving
    points are fitted. Intersecting adjacent fitted lines extrapolates back to where the
    edges actually meet. Assignment is by nearest edge rather than a fixed distance band,
    because the seed can start far enough off that any fixed band excludes the true edge.
    """
    points = boundary.reshape(-1, 2).astype(np.float64)
    if len(points) < 64:
        return None

    quad = order_corners(seed)
    for _ in range(iterations):
        lines = _fit_side_lines(points, quad, corner_skip)
        if lines is None:
            return None
        refined = _intersect_side_lines(lines)
        if refined is None:
            return None
        quad = refined
    return quad


def quad_from_mask(contour_mask: np.ndarray) -> np.ndarray | None:
    """The field outline reduced to four image corners.

    Under perspective a rectangle projects to a general quadrilateral, not a rotated
    rectangle, so `minAreaRect` is wrong here: it returns the bounding box of the quad and
    the error grows with camera tilt. Simplify the convex hull instead, loosening the
    tolerance until exactly four vertices survive.
    """
    contours, _ = cv2.findContours(
        (contour_mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    hull = cv2.convexHull(max(contours, key=cv2.contourArea))
    if len(hull) < 4:
        return None
    dense, _ = cv2.findContours(
        (contour_mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    boundary = max(dense, key=cv2.contourArea)
    perimeter = cv2.arcLength(hull, True)
    for fraction in np.linspace(0.005, 0.12, 60):
        approx = cv2.approxPolyDP(hull, fraction * perimeter, True)
        if len(approx) == 4:
            seed = approx.reshape(4, 2).astype(np.float64)
            refined = refine_quad_by_edges(boundary, seed)
            return seed if refined is None else refined
    return None


def field_object_corners(field_size_xy: tuple[float, float]) -> np.ndarray:
    """Field corners on the z = 0 plane, wound to match `order_corners`.

    Image y grows downward, so object -y binds to the top of the frame. The same winding
    as `field_object_corners` in `src/field_filter/field_pose.cpp`, so a pose solved here
    is the pose the C++ filters would solve from the same corners.
    """
    half_w, half_h = field_size_xy[0] / 2.0, field_size_xy[1] / 2.0
    return np.array([[-half_w, -half_h], [-half_w, half_h], [half_w, half_h], [half_w, -half_h]])


def pose_from_corners(
    image_corners: np.ndarray,
    field_size_xy: tuple[float, float],
    intrinsics: np.ndarray,
) -> tuple[np.ndarray, float] | None:
    """Decompose the corner homography into tf_camera_from_field.

    H = K [r1 r2 t] up to scale, so K^-1 H recovers two rotation columns and the
    translation, and the third column is their cross product. The result is
    orthonormalized because measured corners never satisfy the constraint exactly.
    Returns (4x4 transform, mean corner reprojection error in px), or None when
    findHomography fails. Field z points toward the camera when the camera looks down
    at the field from above: r3 . t is negative for every pose this returns.
    """
    object_corners = field_object_corners(field_size_xy)
    homography, _ = cv2.findHomography(object_corners, np.asarray(image_corners), method=0)
    if homography is None or homography.shape != (3, 3):
        return None

    k_inv = np.linalg.inv(intrinsics)
    h = k_inv @ homography
    # One scale for both rotation columns; averaging the two norms is less sensitive to
    # corner noise than trusting either alone.
    lambda_ = 2.0 / (np.linalg.norm(h[:, 0]) + np.linalg.norm(h[:, 1]))
    r1, r2, t = h[:, 0] * lambda_, h[:, 1] * lambda_, h[:, 2] * lambda_
    if t[2] < 0:  # field must sit in front of the camera
        r1, r2, t = -r1, -r2, -t
    r3 = np.cross(r1, r2)
    u, _, vh = np.linalg.svd(np.stack([r1, r2, r3], axis=1))
    rotation = u @ vh
    if np.linalg.det(rotation) < 0:
        rotation = u @ np.diag([1.0, 1.0, -1.0]) @ vh

    tf = np.eye(4)
    tf[:3, :3] = rotation
    tf[:3, 3] = t

    # z = 0 plane, so the homography columns apply directly.
    projected = homography @ np.c_[object_corners, np.ones(4)].T
    projected = (projected[:2] / projected[2]).T
    reprojection_px = float(np.linalg.norm(projected - image_corners, axis=1).mean())
    return tf, reprojection_px


def project_field_corners(
    tf_camera_from_field: np.ndarray,
    field_size_xy: tuple[float, float],
    intrinsics: np.ndarray,
) -> np.ndarray:
    """Image positions (4x2) of the field corners under a pose, same order as above."""
    corners = field_object_corners(field_size_xy)
    points = np.c_[corners, np.zeros(4), np.ones(4)] @ tf_camera_from_field.T
    pixels = intrinsics @ points[:, :3].T
    return np.asarray((pixels[:2] / pixels[2]).T)


def homography_field(
    mask: np.ndarray,
    intrinsics: np.ndarray,
    field_size_xy: tuple[float, float],
) -> FieldResult:
    """Pose from the field outline in RGB plus known metric dimensions. No depth.

    The mask's outline gives four image corners; the known field size gives the matching
    object corners; `pose_from_corners` does the decomposition.
    """
    contour_mask = largest_contour_mask(mask)
    if not contour_mask.any():
        return FieldResult("homography", None, None, notes="empty field mask")

    clipped = touches_border(contour_mask)
    quad = quad_from_mask(contour_mask)
    coverage = None
    if quad is not None:
        quad_area = abs(cv2.contourArea(quad.astype(np.float32)))
        # Mask area over quad area. Above 1 the mask spills outside the four-sided fit, so
        # the outline is not the quadrilateral this method assumes and at least one edge is
        # not the field's real edge. Reprojection error cannot see this: the homography
        # fits whatever four corners it is handed, however wrong they are.
        coverage = float((contour_mask > 0).sum()) / max(1.0, quad_area)
    fallback = False
    if quad is None:
        # No clean four-sided outline. The bounding rectangle is a poor stand-in under
        # tilt, so the result is returned but flagged.
        points = cv2.findNonZero(contour_mask)
        if points is None or len(points) < 4:
            return FieldResult("homography", None, None, notes="mask too small")
        quad = cv2.boxPoints(cv2.minAreaRect(cv2.convexHull(points))).astype(np.float64)
        fallback = True
    image_corners = order_corners(quad)

    solved = pose_from_corners(image_corners, field_size_xy, intrinsics)
    if solved is None:
        return FieldResult("homography", None, None, notes="findHomography failed")
    tf, reprojection_px = solved
    width, height = field_size_xy

    notes = ["size assumed, not measured"]
    if fallback:
        notes.append("quad fallback to minAreaRect")
    if clipped:
        notes.append("FIELD CLIPPED BY IMAGE BORDER, corners unreliable")
    if coverage is not None and coverage > QUAD_COVERAGE_TOLERANCE:
        notes.append(
            f"mask spills {(coverage - 1.0) * 100:.0f}% outside the quad, "
            "outline is not a clean quadrilateral"
        )

    return FieldResult(
        "homography",
        tf,
        (float(width), float(height)),
        corners_image=image_corners,
        plane_normal=tf[:3, 2],
        notes="; ".join(notes),
        extra={
            "reprojection_px": reprojection_px,
            "clipped": clipped,
            "mask_over_quad_area": coverage,
        },
    )


# ------------------------------------------------------------------ partial outlines


def contour_points_off_border(mask: np.ndarray, border_margin_px: int = 4) -> np.ndarray:
    """Dense outline points of the largest blob, minus those lying along the image border.

    Where the field runs off the frame the contour follows the image edge, and those points
    describe the sensor, not the field. Dropping them leaves only real edge support.
    """
    binary = (mask > 0).astype(np.uint8)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return np.empty((0, 2))
    points = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    height, width = mask.shape[:2]
    keep = (
        (points[:, 0] >= border_margin_px)
        & (points[:, 0] <= width - 1 - border_margin_px)
        & (points[:, 1] >= border_margin_px)
        & (points[:, 1] <= height - 1 - border_margin_px)
    )
    return np.asarray(points[keep])


def side_lines_from_points(
    points: np.ndarray,
    seed_quad: np.ndarray,
    corner_skip: float = 0.15,
    min_points: int = 40,
) -> list[tuple[np.ndarray, float] | None]:
    """One fitted line per side of `seed_quad`, or None where the outline gives no support.

    Points are assigned to the nearest seed side, the ends of each side are skipped as
    corner rounding, and a robust total-least-squares line is fitted to the rest. The seed
    may have corners far outside the image (a pose prior projected into the frame): only the
    assignment depends on it, not the fitted lines.
    """
    if len(points) < min_points:
        return [None] * 4
    quad = order_corners(np.asarray(seed_quad, dtype=np.float64))
    edges = [(quad[i], quad[(i + 1) % 4]) for i in range(4)]
    units, lengths = [], []
    for a, b in edges:
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length < 1e-6:
            return [None] * 4
        units.append(edge / length)
        lengths.append(length)
    perp = np.empty((len(points), 4))
    along = np.empty((len(points), 4))
    for i, (a, _) in enumerate(edges):
        rel = points - a
        along[:, i] = rel @ units[i] / lengths[i]
        perp[:, i] = np.abs(rel @ np.array([-units[i][1], units[i][0]]))
    owner = np.argmin(perp, axis=1)

    lines: list[tuple[np.ndarray, float] | None] = []
    for i in range(4):
        mine = (owner == i) & (along[:, i] > corner_skip) & (along[:, i] < 1.0 - corner_skip)
        if mine.sum() < min_points:
            lines.append(None)
            continue
        selected = points[mine]
        normal, offset = _fit_line(selected)
        residual = np.abs(selected @ normal - offset)
        keep = residual <= max(2.0, 2.5 * float(np.median(residual)))
        if keep.sum() >= min_points:
            selected = selected[keep]
        lines.append(_fit_line(selected))
    return lines


def project_points(
    tf_camera_from_field: np.ndarray, points_field: np.ndarray, intrinsics: np.ndarray
) -> np.ndarray:
    """Field-frame points (n, 3) to pixels (n, 2)."""
    homogeneous = np.c_[points_field, np.ones(len(points_field))] @ tf_camera_from_field.T
    pixels = intrinsics @ homogeneous[:, :3].T
    return np.asarray((pixels[:2] / pixels[2]).T)


def pose_from_lines(
    lines: list[tuple[np.ndarray, float] | None],
    field_size_xy: tuple[float, float],
    intrinsics: np.ndarray,
    tf_init: np.ndarray,
) -> tuple[np.ndarray, float] | None:
    """tf_camera_from_field from three or four observed field edge lines.

    Each supported side contributes two residuals: the signed pixel distance of its two
    projected field corners from the observed line. Three sides give six equations for the
    six pose parameters, which is the clipped-near-edge case the cage-high camera presents;
    four sides overdetermine and refine. `tf_init` seeds the solve and must be on the right
    side of the field (a prior pose or a four-corner fit). Returns (4x4, RMS residual px).
    """
    supported = [i for i, line in enumerate(lines) if line is not None]
    if len(supported) < 3:
        return None
    object_corners = field_object_corners(field_size_xy)
    corners_3d = np.c_[object_corners, np.zeros(4)]

    def unpack(params: np.ndarray) -> np.ndarray:
        tf = np.eye(4)
        tf[:3, :3], _ = cv2.Rodrigues(params[:3].reshape(3, 1))
        tf[:3, 3] = params[3:]
        return tf

    def residuals(params: np.ndarray) -> np.ndarray:
        pixels = project_points(unpack(params), corners_3d, intrinsics)
        out = []
        for i in supported:
            line = lines[i]
            assert line is not None
            normal, offset = line
            for corner in (pixels[i], pixels[(i + 1) % 4]):
                out.append(float(corner @ normal - offset))
        return np.asarray(out, dtype=np.float64)

    rvec0, _ = cv2.Rodrigues(np.asarray(tf_init[:3, :3], dtype=np.float64))
    x0 = np.r_[rvec0.ravel(), tf_init[:3, 3]]
    solution = least_squares(residuals, x0, method="trf", x_scale="jac", max_nfev=2000)
    tf = np.asarray(unpack(solution.x))
    if tf[2, 3] <= 0:  # field centre behind the camera
        return None
    rms = float(np.sqrt(np.mean(solution.fun**2)))
    return tf, rms


def pixels_to_field_plane(
    pixels: np.ndarray,
    tf_camera_from_field: np.ndarray,
    intrinsics: np.ndarray,
    plane_height_m: float = 0.0,
) -> np.ndarray:
    """(n, 2) rectified pixels to (n, 3) field-frame points on the plane z = plane_height_m.

    Rays that miss the plane in front of the camera come back as NaN.
    """
    tf_field_from_camera = np.linalg.inv(tf_camera_from_field)
    origin = tf_field_from_camera[:3, 3]
    k_inv = np.linalg.inv(intrinsics)
    homogeneous = np.c_[np.asarray(pixels, dtype=np.float64).reshape(-1, 2), np.ones(len(pixels))]
    directions = (tf_field_from_camera[:3, :3] @ (k_inv @ homogeneous.T)).T
    out = np.full((len(pixels), 3), np.nan)
    for i, direction in enumerate(directions):
        if abs(direction[2]) < 1e-9:
            continue
        t = (plane_height_m - origin[2]) / direction[2]
        if t <= 0:
            continue
        out[i] = origin + t * direction
    return out


# ----------------------------------------------------------------------- comparison


def yaw_difference_deg(tf_a: np.ndarray, tf_b: np.ndarray, square: bool) -> float:
    """Yaw difference folded into the field rectangle's own symmetry.

    A bare rectangle is 180-degree symmetric, and a square is 90-degree symmetric, so a
    raw yaw difference of 90 or 180 degrees means the two methods agree on the field and
    disagree only on which corner they called first.
    """
    relative = np.linalg.inv(tf_a[:3, :3]) @ tf_b[:3, :3]
    yaw = math.degrees(math.atan2(relative[1, 0], relative[0, 0]))
    period = 90.0 if square else 180.0
    folded = math.fmod(abs(yaw), period)
    return min(folded, period - folded)


def compare(a: FieldResult, b: FieldResult, square: bool) -> dict[str, Any]:
    """Translation, normal and yaw agreement between two methods on one frame."""
    if a.tf_camera_from_field is None or b.tf_camera_from_field is None:
        return {"ok": False, "notes": f"{a.method}: {a.notes} | {b.method}: {b.notes}"}
    tf_a, tf_b = a.tf_camera_from_field, b.tf_camera_from_field
    ta, tb = tf_a[:3, 3], tf_b[:3, 3]
    na = tf_a[:3, :3] @ np.array([0.0, 0.0, 1.0])
    nb = tf_b[:3, :3] @ np.array([0.0, 0.0, 1.0])
    cos = float(np.clip(abs(np.dot(na, nb)), -1.0, 1.0))
    return {
        "ok": True,
        "translation_diff_m": float(np.linalg.norm(ta - tb)),
        "range_a_m": float(np.linalg.norm(ta)),
        "range_b_m": float(np.linalg.norm(tb)),
        "range_diff_m": float(np.linalg.norm(ta) - np.linalg.norm(tb)),
        "normal_angle_deg": math.degrees(math.acos(cos)),
        "yaw_diff_deg": yaw_difference_deg(tf_a, tf_b, square),
        "size_a_m": a.size_xy_m,
        "size_b_m": b.size_xy_m,
    }
