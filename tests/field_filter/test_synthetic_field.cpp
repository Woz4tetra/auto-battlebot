#include <gtest/gtest.h>

#include <array>
#include <utility>
#include <vector>

#include "field_filter/field_pose.hpp"
#include "field_filter/synthetic_field.hpp"

namespace auto_battlebot {
namespace {

constexpr double kFieldSize = 2.35;
constexpr double kCornerRadius = 0.2;

using Polygon = std::vector<std::pair<double, double>>;

double shoelace_area(const Polygon &polygon) {
    double total = 0.0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto &[ax, ay] = polygon[i];
        const auto &[bx, by] = polygon[(i + 1) % polygon.size()];
        total += ax * by - bx * ay;
    }
    return total;
}

TEST(SyntheticFieldTest, SquareOutlineWindsLikeFieldOutlineCorners) {
    const Eigen::Matrix4d truth = testing_support::camera_pose(3.0, 5.0, 12.0);
    const Eigen::Matrix3d intrinsics = testing_support::test_intrinsics();

    Polygon expected;
    for (const cv::Point2d &corner :
         project_field_corners(truth, kFieldSize, kFieldSize, intrinsics)) {
        expected.emplace_back(corner.x, corner.y);
    }
    Polygon outline;
    for (const cv::Point &point : testing_support::project(
             testing_support::field_outline_points(kFieldSize, kFieldSize, 0.0), truth,
             intrinsics)) {
        outline.emplace_back(point.x, point.y);
    }

    EXPECT_GT(shoelace_area(expected) * shoelace_area(outline), 0.0);
}

TEST(SyntheticFieldTest, RoundedCornersKeepTheOutlineConvexInWindingOrder) {
    const auto points =
        testing_support::field_outline_points(kFieldSize, kFieldSize, kCornerRadius);
    ASSERT_GT(points.size(), 4u);

    Polygon flat;
    for (const Eigen::Vector3d &point : points) flat.emplace_back(point.x(), point.y());
    const double winding = shoelace_area(flat);
    for (size_t i = 0; i < points.size(); ++i) {
        const Eigen::Vector3d &a = points[i];
        const Eigen::Vector3d &b = points[(i + 1) % points.size()];
        const Eigen::Vector3d &c = points[(i + 2) % points.size()];
        const double turn = (b - a).cross(c - b).z();
        EXPECT_GE(turn * winding, -1e-12) << "point " << i;
    }
}

}  // namespace
}  // namespace auto_battlebot
