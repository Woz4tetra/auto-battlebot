#include "rgbd_camera/camera_calibration.hpp"

#include <toml++/toml.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <opencv2/calib3d.hpp>

#include "config/config_parser.hpp"
#include "directories.hpp"

namespace auto_battlebot {
namespace {
std::filesystem::path resolve(const std::string &value) {
    std::filesystem::path path(value);
    if (path.is_absolute()) {
        return path;
    }
    return get_project_root() / path;
}

double required_double(const toml::table &table, const std::string &key,
                       const std::string &source) {
    auto value = table[key].value<double>();
    if (!value) {
        throw ConfigValidationError("'" + key + "' missing from " + source);
    }
    return *value;
}
}  // namespace

cv::Mat CameraCalibration::camera_matrix() const {
    cv::Mat matrix = cv::Mat::eye(3, 3, CV_64F);
    matrix.at<double>(0, 0) = fx;
    matrix.at<double>(1, 1) = fy;
    matrix.at<double>(0, 2) = cx;
    matrix.at<double>(1, 2) = cy;
    return matrix;
}

cv::Mat CameraCalibration::distortion_coefficients() const {
    cv::Mat coefficients(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
        coefficients.at<double>(0, i) = distortion[static_cast<size_t>(i)];
    }
    return coefficients;
}

CameraCalibration load_camera_calibration(const std::string &path) {
    const std::filesystem::path resolved = resolve(path);
    if (!std::filesystem::exists(resolved)) {
        throw ConfigValidationError("calibration_file not found: " + resolved.string());
    }
    const toml::table data = toml::parse_file(resolved.string());
    const std::string source = resolved.string();

    CameraCalibration calibration;
    calibration.calibration_id = data["calibration_id"].value_or(std::string());
    if (calibration.calibration_id.empty()) {
        throw ConfigValidationError("'calibration_id' missing from " + source);
    }
    calibration.width = static_cast<int>(data["width"].value_or(0));
    calibration.height = static_cast<int>(data["height"].value_or(0));
    if (calibration.width <= 0 || calibration.height <= 0) {
        throw ConfigValidationError("'width' and 'height' in " + source + " must be > 0");
    }
    calibration.fx = required_double(data, "fx", source);
    calibration.fy = required_double(data, "fy", source);
    calibration.cx = required_double(data, "cx", source);
    calibration.cy = required_double(data, "cy", source);
    calibration.distortion = {
        required_double(data, "k1", source), required_double(data, "k2", source),
        required_double(data, "p1", source), required_double(data, "p2", source),
        required_double(data, "k3", source)};
    return calibration;
}

void save_camera_calibration(const std::string &path, const CameraCalibration &calibration) {
    const std::filesystem::path resolved = resolve(path);
    std::filesystem::create_directories(resolved.parent_path());
    std::ofstream out(resolved);
    if (!out) {
        throw ConfigValidationError("cannot write calibration file: " + resolved.string());
    }
    out << std::setprecision(12);
    out << "# One physical camera and lens, shot through an offcut of the cage panel at the\n";
    out << "# mounted standoff. Free-air numbers fold the panel's refraction into the field "
           "pose.\n";
    out << "calibration_id = \"" << calibration.calibration_id << "\"\n";
    out << "width = " << calibration.width << "\n";
    out << "height = " << calibration.height << "\n";
    out << "fx = " << calibration.fx << "\n";
    out << "fy = " << calibration.fy << "\n";
    out << "cx = " << calibration.cx << "\n";
    out << "cy = " << calibration.cy << "\n";
    static const char *kNames[] = {"k1", "k2", "p1", "p2", "k3"};
    for (size_t i = 0; i < calibration.distortion.size(); ++i) {
        out << kNames[i] << " = " << calibration.distortion[i] << "\n";
    }
}

void Rectifier::build(const CameraCalibration &calibration, cv::Size size, double alpha) {
    size_ = size;
    cv::Mat matrix = calibration.camera_matrix();
    // A calibration shot at one resolution still rectifies another: the intrinsics scale, the
    // distortion coefficients are dimensionless and do not.
    if (calibration.width != size.width || calibration.height != size.height) {
        const double scale_x = static_cast<double>(size.width) / calibration.width;
        const double scale_y = static_cast<double>(size.height) / calibration.height;
        matrix.at<double>(0, 0) *= scale_x;
        matrix.at<double>(0, 2) *= scale_x;
        matrix.at<double>(1, 1) *= scale_y;
        matrix.at<double>(1, 2) *= scale_y;
    }
    const cv::Mat distortion = calibration.distortion_coefficients();
    rectified_matrix_ = cv::getOptimalNewCameraMatrix(matrix, distortion, size, alpha, size);
    cv::initUndistortRectifyMap(matrix, distortion, cv::Mat(), rectified_matrix_, size, CV_16SC2,
                                map_x_, map_y_);
}

void Rectifier::apply(const cv::Mat &source, cv::Mat &destination) const {
    if (map_x_.empty()) {
        destination = source;
        return;
    }
    cv::remap(source, destination, map_x_, map_y_, cv::INTER_LINEAR);
}

CameraInfo Rectifier::camera_info() const {
    CameraInfo info;
    info.width = size_.width;
    info.height = size_.height;
    info.intrinsics = rectified_matrix_.clone();
    // Zeros, not empty: camera_info.cpp keeps emitting a plumb_bob model, which now happens to be
    // trivial because the frames are already rectified.
    info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    return info;
}
}  // namespace auto_battlebot
