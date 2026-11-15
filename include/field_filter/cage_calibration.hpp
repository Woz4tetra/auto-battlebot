#pragma once

#include <Eigen/Dense>
#include <string>

namespace auto_battlebot {
/**
 * @brief A cage measured once, so match day does not have to fit anything.
 *
 * Written by `playground/field_transform/calibrate_cage.py` from a clean empty-cage frame and
 * loaded unchanged at startup. Everything in it is a measurement of one physical cage with the
 * camera fixture seated, so it travels with the venue rather than with the build.
 */
struct CageCalibration {
    std::string calibration_id;
    /** The floor mat, measured, not the nominal cage size. */
    double field_size_x = 0.0;
    double field_size_y = 0.0;
    Eigen::Matrix4d tf_camera_from_fieldcenter = Eigen::Matrix4d::Identity();
};

CageCalibration load_cage_calibration(const std::string &path);

void save_cage_calibration(const std::string &path, const CageCalibration &calibration);
}  // namespace auto_battlebot
