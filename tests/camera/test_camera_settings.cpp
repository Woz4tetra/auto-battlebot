#include <gtest/gtest.h>

#include "rgbd_camera/camera_settings.hpp"

namespace auto_battlebot {
namespace {

TEST(CameraSettingsTest, ExposureRangeCappedAtFramePeriod) {
    CameraSettingRange sdk_range{.supported = true, .min = 10, .max = 100'000};
    const CameraSettingRange at_30fps = exposure_range_capped_at_frame_period(sdk_range, 30);
    EXPECT_EQ(at_30fps.max, 33'333);
    EXPECT_EQ(at_30fps.min, 10);
    EXPECT_TRUE(at_30fps.supported);
}

TEST(CameraSettingsTest, ExposureRangeBelowTheFramePeriodIsKept) {
    CameraSettingRange sdk_range{.supported = true, .min = 10, .max = 20'000};
    EXPECT_EQ(exposure_range_capped_at_frame_period(sdk_range, 30).max, 20'000);
}

}  // namespace
}  // namespace auto_battlebot
