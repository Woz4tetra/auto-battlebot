#include <RTNeural/RTNeural.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

TEST(RtneuralEigenVersion, RtneuralSeesTheSystemEigenNotItsBundledCopy) {
    EXPECT_EQ(EIGEN_WORLD_VERSION, SYSTEM_EIGEN_WORLD_VERSION);
    EXPECT_EQ(EIGEN_MAJOR_VERSION, SYSTEM_EIGEN_MAJOR_VERSION);
    EXPECT_EQ(EIGEN_MINOR_VERSION, SYSTEM_EIGEN_MINOR_VERSION);
}
