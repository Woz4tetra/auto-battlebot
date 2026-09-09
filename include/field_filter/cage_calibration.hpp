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
    /** Stamped into MCAP metadata, so a recording says which calibration produced its field. */
    std::string calibration_id;
    /** The floor mat, measured, not the nominal cage size. */
    double field_size_x = 0.0;
    double field_size_y = 0.0;
    /** Field centre expressed in camera coordinates. */
    Eigen::Matrix4d tf_camera_from_fieldcenter = Eigen::Matrix4d::Identity();
};

/** Load from `config/cages/<venue>_<cage>.toml`, resolved like `hazards_file`: absolute, or
 *  relative to the project root. Throws ConfigValidationError on a missing or malformed file. */
CageCalibration load_cage_calibration(const std::string &path);

/** Write one out, for the calibration tool and for tests. */
void save_cage_calibration(const std::string &path, const CageCalibration &calibration);
}  // namespace auto_battlebot
