#include "field_filter/field_pose.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace auto_battlebot {
namespace {
Eigen::Vector3d to_eigen(const cv::Vec3d &vector) {
    return Eigen::Vector3d(vector[0], vector[1], vector[2]);
}

/** Rotation whose first two columns are the given field axes in camera coordinates. */
Eigen::Matrix3d rotation_from_axes(const Eigen::Vector3d &x_axis, const Eigen::Vector3d &y_axis) {
    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = x_axis.cross(y_axis);
    return rotation;
}

Eigen::Matrix4d to_transform(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}

/** Twice the signed area of a quad, so only its sign is used: which way it winds. */
double signed_area(const std::array<cv::Point2d, 4> &quad) {
    double total = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        const cv::Point2d &a = quad[i];
        const cv::Point2d &b = quad[(i + 1) % 4];
        total += a.x * b.y - b.x * a.y;
    }
    return total;
}

/** Mean distance from each projected corner to the fitted line of every supported side. */
double line_residual(const std::array<cv::Point2d, 4> &projected,
                     const std::array<cv::Vec3d, 4> &lines,
                     const std::array<bool, 4> &side_supported) {
    double total = 0.0;
    int count = 0;
    for (size_t side = 0; side < 4; ++side) {
        if (!side_supported[side]) {
            continue;
        }
        for (size_t offset = 0; offset < 2; ++offset) {
            const cv::Point2d &corner = projected[(side + offset) % 4];
            total +=
                std::abs(lines[side][0] * corner.x + lines[side][1] * corner.y + lines[side][2]);
            ++count;
        }
    }
    return count == 0 ? 0.0 : total / count;
}
}  // namespace

bool intrinsics_to_eigen(const cv::Mat &intrinsics, Eigen::Matrix3d &out) {
    if (intrinsics.rows != 3 || intrinsics.cols != 3) {
        return false;
    }
    cv::Mat as_double;
    intrinsics.convertTo(as_double, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out(row, col) = as_double.at<double>(row, col);
        }
    }
    return std::abs(out(0, 0)) > 1e-9 && std::abs(out(1, 1)) > 1e-9;
}

std::array<cv::Point2d, 4> field_object_corners(double size_x, double size_y) {
    const double half_x = size_x / 2.0;
    const double half_y = size_y / 2.0;
    // Same winding as FieldOutline::corners. Image y grows downward, so -y is the top of frame.
    return {cv::Point2d(-half_x, -half_y), cv::Point2d(-half_x, half_y),
            cv::Point2d(half_x, half_y), cv::Point2d(half_x, -half_y)};
}

std::array<cv::Point2d, 4> project_field_corners(const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                                                 double size_x, double size_y,
                                                 const Eigen::Matrix3d &intrinsics) {
    const auto object = field_object_corners(size_x, size_y);
    const Eigen::Matrix3d rotation = tf_camera_from_fieldcenter.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = tf_camera_from_fieldcenter.block<3, 1>(0, 3);
    std::array<cv::Point2d, 4> projected{};
    for (size_t i = 0; i < 4; ++i) {
        const Eigen::Vector3d point =
            rotation * Eigen::Vector3d(object[i].x, object[i].y, 0.0) + translation;
        const Eigen::Vector3d image = intrinsics * point;
        const double depth = std::abs(image.z()) < 1e-9 ? 1e-9 : image.z();
        projected[i] = cv::Point2d(image.x() / depth, image.y() / depth);
    }
    return projected;
}

FieldPoseResult pose_from_corners(const std::array<cv::Point2d, 4> &image_corners, double size_x,
                                  double size_y, const Eigen::Matrix3d &intrinsics) {
    FieldPoseResult result;
    const auto object = field_object_corners(size_x, size_y);
    std::vector<cv::Point2d> object_points(object.begin(), object.end());
    std::vector<cv::Point2d> image_points(image_corners.begin(), image_corners.end());

    // Method 0 is the exact four-point solve. Corners outside the sensor are fine here: nothing
    // in findHomography requires them to land on the image, and at a cage mount the near mat
    // corners never do.
    const cv::Mat homography_cv = cv::findHomography(object_points, image_points, 0);
    if (homography_cv.empty()) {
        result.failure = "findHomography failed";
        return result;
    }
    Eigen::Matrix3d homography;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            homography(row, col) = homography_cv.at<double>(row, col);
        }
    }

    const Eigen::Matrix3d h = intrinsics.inverse() * homography;
    // One scale for both rotation columns; averaging the two norms is less sensitive to corner
    // noise than trusting either alone.
    const double scale = 2.0 / (h.col(0).norm() + h.col(1).norm());
    Eigen::Vector3d r1 = h.col(0) * scale;
    Eigen::Vector3d r2 = h.col(1) * scale;
    Eigen::Vector3d translation = h.col(2) * scale;
    if (translation.z() < 0.0) {  // the field must sit in front of the camera
        r1 = -r1;
        r2 = -r2;
        translation = -translation;
    }

    // The measured corners never satisfy the orthonormality constraint exactly.
    Eigen::Matrix3d raw = rotation_from_axes(r1, r2);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d rotation = svd.matrixU() * svd.matrixV().transpose();
    if (rotation.determinant() < 0.0) {
        Eigen::Matrix3d flip = Eigen::Matrix3d::Identity();
        flip(2, 2) = -1.0;
        rotation = svd.matrixU() * flip * svd.matrixV().transpose();
    }

    result.tf_camera_from_fieldcenter = to_transform(rotation, translation);
    const auto projected =
        project_field_corners(result.tf_camera_from_fieldcenter, size_x, size_y, intrinsics);
    double total = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        total += cv::norm(projected[i] - image_corners[i]);
    }
    result.residual_px = total / 4.0;
    result.ok = true;
    return result;
}

FieldPoseResult pose_from_three_lines(const std::array<cv::Vec3d, 4> &lines,
                                      const std::array<bool, 4> &side_supported,
                                      const std::array<cv::Point2d, 4> &observed_corners,
                                      double size_x, double size_y,
                                      const Eigen::Matrix3d &intrinsics) {
    FieldPoseResult result;
    size_t missing = 4;
    for (size_t i = 0; i < 4; ++i) {
        if (!side_supported[i]) {
            if (missing != 4) {
                result.failure = "three-line solve needs exactly three supported sides";
                return result;
            }
            missing = i;
        }
    }
    if (missing == 4) {
        result.failure = "three-line solve needs exactly three supported sides";
        return result;
    }

    // The two sides opposite each other are the pair that survives whichever side is missing.
    // `between` is the third, running from one to the other.
    const size_t first = (missing + 1) % 4;
    const size_t between = (missing + 2) % 4;
    const size_t second = (missing + 3) % 4;

    const auto object = field_object_corners(size_x, size_y);
    const auto direction_of = [&object](size_t side) {
        const cv::Point2d delta = object[(side + 1) % 4] - object[side];
        return cv::Point2d(delta.x, delta.y) * (1.0 / cv::norm(delta));
    };
    const cv::Point2d along_pair = direction_of(first);
    const cv::Point2d along_between = direction_of(between);

    const Eigen::Matrix3d intrinsics_inverse = intrinsics.inverse();

    // The two opposite sides are parallel in the world, so they meet at the vanishing point of
    // their shared direction. Homogeneous throughout, so a camera looking square-on, where that
    // point runs off to infinity, needs no special case.
    const Eigen::Vector3d vanishing =
        intrinsics_inverse * to_eigen(lines[first].cross(lines[second]));
    if (vanishing.norm() < 1e-12) {
        result.failure = "opposite field sides are coincident";
        return result;
    }
    const Eigen::Vector3d pair_axis = vanishing.normalized();

    // Every point on the third side backprojects into a plane with this normal, and that side's
    // own vanishing point lies on it, so the remaining field axis is perpendicular to both.
    const Eigen::Vector3d between_plane = intrinsics.transpose() * to_eigen(lines[between]);
    const Eigen::Vector3d between_axis_raw = between_plane.cross(pair_axis);
    if (between_axis_raw.norm() < 1e-12) {
        result.failure = "field sides are degenerate in the image";
        return result;
    }
    const Eigen::Vector3d between_axis = between_axis_raw.normalized();

    // The two corners the third side still has. Both are finite: they are the ones inside frame.
    const auto finite_corner = [&](size_t left, size_t right, Eigen::Vector3d &out) {
        const cv::Vec3d point = lines[left].cross(lines[right]);
        if (std::abs(point[2]) < 1e-9) {
            return false;
        }
        out = intrinsics_inverse * Eigen::Vector3d(point[0] / point[2], point[1] / point[2], 1.0);
        return true;
    };
    Eigen::Vector3d ray_near;
    Eigen::Vector3d ray_far;
    if (!finite_corner(first, between, ray_near) || !finite_corner(between, second, ray_far)) {
        result.failure = "field side intersections are at infinity";
        return result;
    }
    // Corner i sits between sides i-1 and i, so these are corners `between` and `second`.
    const cv::Point2d object_near = object[between];
    const cv::Point2d object_far = object[second];
    const cv::Point2d object_span = object_far - object_near;

    // A rectangle mirrored about either of its own axes is the same rectangle, so a mirrored
    // field frame reprojects all three lines exactly and the residual cannot tell it from the
    // right answer. It does reverse the winding of the projected quad, and the outline's winding
    // is fixed, so that is the discriminator.
    const double observed_winding = signed_area(observed_corners);

    double best_residual = std::numeric_limits<double>::max();
    for (const double pair_sign : {1.0, -1.0}) {
        for (const double between_sign : {1.0, -1.0}) {
            const Eigen::Vector3d axis_pair = pair_sign * pair_axis;
            const Eigen::Vector3d axis_between = between_sign * between_axis;
            // The two field axes are an orthonormal basis, so the camera-frame image of field
            // +x is its component along each.
            const Eigen::Vector3d field_x =
                along_pair.x * axis_pair + along_between.x * axis_between;
            const Eigen::Vector3d field_y =
                along_pair.y * axis_pair + along_between.y * axis_between;

            // The two corners are a known distance apart along the third side, which is what
            // fixes the scale a homography would have taken from four corners.
            const Eigen::Vector3d span = object_span.x * field_x + object_span.y * field_y;
            Eigen::Matrix<double, 3, 2> basis;
            basis.col(0) = -ray_near;
            basis.col(1) = ray_far;
            const Eigen::Vector2d depths = basis.colPivHouseholderQr().solve(span);
            if (depths(0) <= 0.0 || depths(1) <= 0.0) {
                continue;
            }
            const Eigen::Vector3d translation =
                depths(0) * ray_near - object_near.x * field_x - object_near.y * field_y;
            if (translation.z() <= 0.0) {
                continue;
            }

            const Eigen::Matrix3d rotation = rotation_from_axes(field_x, field_y);
            const Eigen::Matrix4d transform = to_transform(rotation, translation);
            const auto projected = project_field_corners(transform, size_x, size_y, intrinsics);
            if (signed_area(projected) * observed_winding <= 0.0) {
                continue;
            }
            const double residual = line_residual(projected, lines, side_supported);
            if (residual < best_residual) {
                best_residual = residual;
                result.tf_camera_from_fieldcenter = transform;
                result.residual_px = residual;
                result.ok = true;
            }
        }
    }
    if (!result.ok) {
        result.failure =
            "no three-line solution places the field in front of the camera with the observed "
            "winding";
    }
    return result;
}

FieldPoseResult pose_from_outline(const FieldOutline &outline, double size_x, double size_y,
                                  const Eigen::Matrix3d &intrinsics) {
    if (!outline.ok) {
        FieldPoseResult result;
        result.failure = outline.failure;
        return result;
    }
    if (outline.supported_sides == 4) {
        return pose_from_corners(outline.corners, size_x, size_y, intrinsics);
    }
    return pose_from_three_lines(outline.lines, outline.side_supported, outline.corners, size_x,
                                 size_y, intrinsics);
}
}  // namespace auto_battlebot
