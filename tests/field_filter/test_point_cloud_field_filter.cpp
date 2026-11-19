#ifdef BUILD_WITH_ZED
#include <gtest/gtest.h>

#include "field_filter/point_cloud_field_filter.hpp"

namespace auto_battlebot {

TEST(PlaneNormalFromCoefficients, SignIsConsistentForBothRansacOrientations) {
    const Eigen::Vector3f toward_camera = plane_normal_from_coefficients({0.0f, 0.0f, -2.0f, 1.0f});
    const Eigen::Vector3f away_from_camera =
        plane_normal_from_coefficients({0.0f, 0.0f, 2.0f, 1.0f});
    EXPECT_FLOAT_EQ(toward_camera.z(), 1.0f);
    EXPECT_FLOAT_EQ(away_from_camera.z(), 1.0f);
}

TEST(PlaneNormalFromCoefficients, DegenerateNormalAgreesWithTheFlippedSign) {
    const Eigen::Vector3f degenerate = plane_normal_from_coefficients(Eigen::Vector4f::Zero());
    const Eigen::Vector3f flipped = plane_normal_from_coefficients({0.0f, 0.0f, -1.0f, 0.0f});
    EXPECT_GE(degenerate.z(), 0.0f);
    EXPECT_TRUE(degenerate.isApprox(flipped));
}

}  // namespace auto_battlebot
#endif
