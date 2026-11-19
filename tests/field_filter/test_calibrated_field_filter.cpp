#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <filesystem>

#include "field_filter/calibrated_field_filter.hpp"
#include "field_filter/synthetic_field.hpp"

namespace auto_battlebot {
namespace {
CageCalibration load_saved_with_yaw(double yaw_rad, const std::string &name) {
    CageCalibration saved;
    saved.calibration_id = "yaw_test";
    saved.measured_mat_size_x_m = 2.35;
    saved.measured_mat_size_y_m = 2.35;
    saved.tf_camera_from_fieldcenter = testing_support::camera_pose(1.9, 48.0, 7.5);
    saved.tf_camera_from_fieldcenter.block<3, 3>(0, 0) =
        saved.tf_camera_from_fieldcenter.block<3, 3>(0, 0) *
        Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
    save_cage_calibration(path.string(), saved);
    const CageCalibration loaded = load_calibration_in_live_outline_convention(path.string());
    std::filesystem::remove(path);
    return loaded;
}
}  // namespace

TEST(CalibratedFieldFilterTest, SavedQuarterTurnYawLoadsToTheSameFrame) {
    const CageCalibration reference = load_saved_with_yaw(0.0, "auto_battlebot_yaw_0.toml");
    for (const double yaw : {M_PI / 2.0, M_PI, -M_PI / 2.0}) {
        const CageCalibration turned = load_saved_with_yaw(yaw, "auto_battlebot_yaw_turned.toml");
        EXPECT_LT((turned.tf_camera_from_fieldcenter - reference.tf_camera_from_fieldcenter).norm(),
                  1e-9)
            << "saved yaw " << yaw;
    }
}
}  // namespace auto_battlebot
