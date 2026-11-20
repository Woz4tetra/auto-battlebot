#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <string>

#include "data_structures.hpp"

namespace auto_battlebot {
struct CameraCalibration {
    std::string calibration_id;
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    std::array<double, 5> distortion_k1_k2_p1_p2_k3{};

    cv::Mat camera_matrix() const;
    cv::Mat distortion_coefficients() const;
};

inline constexpr const char *kCameraCalibrationMetadataKey = "camera_calibration";

std::string zed_x_one_calibration_id(uint32_t serial_number, int width, int height);

CameraCalibration load_camera_calibration(const std::string &path);
void save_camera_calibration(const std::string &path, const CameraCalibration &calibration);

std::string camera_calibration_to_toml(const CameraCalibration &calibration);
CameraCalibration parse_camera_calibration(const std::string &toml_text, const std::string &source);

class Rectifier {
   public:
    Rectifier() = default;
    void build(const CameraCalibration &calibration, cv::Size size, double alpha = 1.0);
    bool ready() const { return !map_x_.empty(); }
    void apply(const cv::Mat &source, cv::Mat &destination) const;

    CameraInfo camera_info() const;

   private:
    cv::Mat map_x_;
    cv::Mat map_y_;
    cv::Mat rectified_matrix_;
    cv::Size size_;
};
}  // namespace auto_battlebot
