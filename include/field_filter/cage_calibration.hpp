#pragma once

#include <Eigen/Dense>
#include <string>

namespace auto_battlebot {
struct CageCalibration {
    std::string calibration_id;
    double measured_mat_size_x_m = 0.0;
    double measured_mat_size_y_m = 0.0;
    Eigen::Matrix4d tf_camera_from_fieldcenter = Eigen::Matrix4d::Identity();
};

CageCalibration load_cage_calibration(const std::string &path);

void save_cage_calibration(const std::string &path, const CageCalibration &calibration);
}  // namespace auto_battlebot
