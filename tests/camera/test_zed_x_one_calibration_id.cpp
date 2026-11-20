#include <gtest/gtest.h>

#include "rgbd_camera/camera_calibration.hpp"

namespace auto_battlebot {

TEST(ZedXOneCalibrationIdTest, ModesOfOneSerialGetDistinctIds) {
    EXPECT_NE(zed_x_one_calibration_id(300123, 1920, 1080),
              zed_x_one_calibration_id(300123, 1920, 1200));
    EXPECT_NE(zed_x_one_calibration_id(300123, 1920, 1080),
              zed_x_one_calibration_id(300123, 960, 540));
}

TEST(ZedXOneCalibrationIdTest, SerialNumbersGetDistinctIds) {
    EXPECT_NE(zed_x_one_calibration_id(300123, 1920, 1080),
              zed_x_one_calibration_id(300124, 1920, 1080));
}

TEST(ZedXOneCalibrationIdTest, IdIsSerialAndResolution) {
    EXPECT_EQ(zed_x_one_calibration_id(300123, 1920, 1080), "zed_x_one_300123_1920x1080");
}

}  // namespace auto_battlebot
