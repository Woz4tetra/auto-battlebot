#include <gtest/gtest.h>

#include <filesystem>

#include "config/config_parser.hpp"
#include "field_filter/cage_calibration.hpp"
#include "field_filter/synthetic_field.hpp"

namespace auto_battlebot {
TEST(CageCalibrationTest, RoundTripsThroughTheFile) {
    CageCalibration written;
    written.calibration_id = "nhrl_cage_north_2026-09-09";
    written.field_size_x = 2.34;
    written.field_size_y = 2.37;
    written.tf_camera_from_fieldcenter = testing_support::camera_pose(1.9, 48.0, 7.5);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "auto_battlebot_cage_calibration_test.toml";
    save_cage_calibration(path.string(), written);
    const CageCalibration read = load_cage_calibration(path.string());
    std::filesystem::remove(path);

    EXPECT_EQ(read.calibration_id, written.calibration_id);
    EXPECT_NEAR(read.field_size_x, written.field_size_x, 1e-9);
    EXPECT_NEAR(read.field_size_y, written.field_size_y, 1e-9);
    EXPECT_LT((read.tf_camera_from_fieldcenter - written.tf_camera_from_fieldcenter).norm(), 1e-9);
}

TEST(CageCalibrationTest, RejectsAMissingFile) {
    EXPECT_THROW(load_cage_calibration("/nonexistent/cage.toml"), ConfigValidationError);
}
}  // namespace auto_battlebot
