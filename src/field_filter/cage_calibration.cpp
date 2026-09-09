#include "field_filter/cage_calibration.hpp"

#include <toml++/toml.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "config/config_parser.hpp"
#include "directories.hpp"

namespace auto_battlebot {
namespace {
// Same rule as hazards_file: a plain path, absolute or relative to the project root. No implied
// directory and no implied extension, so the string in a config reads as the file it names.
std::filesystem::path resolve(const std::string &value) {
    std::filesystem::path path(value);
    if (path.is_absolute()) {
        return path;
    }
    return get_project_root() / path;
}

std::vector<double> required_array(const toml::table &table, const std::string &key, size_t size,
                                   const std::string &source) {
    const toml::array *array = table[key].as_array();
    if (!array || array->size() != size) {
        throw ConfigValidationError("'" + key + "' in " + source + " must be an array of " +
                                    std::to_string(size) + " numbers");
    }
    std::vector<double> values;
    values.reserve(size);
    for (size_t i = 0; i < size; ++i) {
        auto value = (*array)[i].value<double>();
        if (!value) {
            throw ConfigValidationError("'" + key + "[" + std::to_string(i) + "]' in " + source +
                                        " is not a number");
        }
        values.push_back(*value);
    }
    return values;
}
}  // namespace

CageCalibration load_cage_calibration(const std::string &path) {
    const std::filesystem::path resolved = resolve(path);
    if (!std::filesystem::exists(resolved)) {
        throw ConfigValidationError("calibration_file not found: " + resolved.string());
    }
    const toml::table data = toml::parse_file(resolved.string());
    const std::string source = resolved.string();

    CageCalibration calibration;
    calibration.calibration_id = data["calibration_id"].value_or(std::string());
    if (calibration.calibration_id.empty()) {
        throw ConfigValidationError("'calibration_id' missing from " + source);
    }
    calibration.field_size_x = data["field_size_x"].value_or(0.0);
    calibration.field_size_y = data["field_size_y"].value_or(0.0);
    if (calibration.field_size_x <= 0.0 || calibration.field_size_y <= 0.0) {
        throw ConfigValidationError("'field_size_x' and 'field_size_y' in " + source +
                                    " must be > 0");
    }

    const std::vector<double> translation = required_array(data, "translation", 3, source);
    // Row-major 3x3 rather than a quaternion, so nothing has to agree on a component order. The
    // file is machine-written; readability of the rotation is not what matters about it.
    const std::vector<double> rotation = required_array(data, "rotation", 9, source);

    calibration.tf_camera_from_fieldcenter = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            calibration.tf_camera_from_fieldcenter(row, col) =
                rotation[static_cast<size_t>(row * 3 + col)];
        }
        calibration.tf_camera_from_fieldcenter(row, 3) = translation[static_cast<size_t>(row)];
    }
    return calibration;
}

void save_cage_calibration(const std::string &path, const CageCalibration &calibration) {
    const std::filesystem::path resolved = resolve(path);
    std::filesystem::create_directories(resolved.parent_path());
    std::ofstream out(resolved);
    if (!out) {
        throw ConfigValidationError("cannot write calibration file: " + resolved.string());
    }
    out << std::setprecision(12);
    out << "# Written by the cage calibration tool. One physical cage, camera fixture seated.\n";
    out << "calibration_id = \"" << calibration.calibration_id << "\"\n";
    out << "field_size_x = " << calibration.field_size_x << "\n";
    out << "field_size_y = " << calibration.field_size_y << "\n";
    out << "# Field centre in camera coordinates.\n";
    out << "translation = [" << calibration.tf_camera_from_fieldcenter(0, 3) << ", "
        << calibration.tf_camera_from_fieldcenter(1, 3) << ", "
        << calibration.tf_camera_from_fieldcenter(2, 3) << "]\n";
    out << "# Row-major 3x3.\n";
    out << "rotation = [";
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out << calibration.tf_camera_from_fieldcenter(row, col);
            if (row != 2 || col != 2) {
                out << ", ";
            }
        }
    }
    out << "]\n";
}
}  // namespace auto_battlebot
