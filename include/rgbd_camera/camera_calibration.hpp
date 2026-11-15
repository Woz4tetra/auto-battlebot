#pragma once

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
    /** Plumb-bob: k1 k2 p1 p2 k3. */
    std::array<double, 5> distortion{};

    cv::Mat camera_matrix() const;
    cv::Mat distortion_coefficients() const;
};

inline constexpr const char *kCameraCalibrationMetadataKey = "camera_calibration";

CameraCalibration load_camera_calibration(const std::string &path);
void save_camera_calibration(const std::string &path, const CameraCalibration &calibration);

std::string camera_calibration_to_toml(const CameraCalibration &calibration);
CameraCalibration parse_camera_calibration(const std::string &toml_text, const std::string &source);

/**
 * @brief Undistortion maps built once at open, applied per frame with cv::remap.
 *
 * `alpha` is 1.0 rather than 0.0 deliberately: a cropping alpha at this field of view cuts
 * exactly the mat edges the field fit needs. The rectified matrix it produces is what the camera
 * publishes, with D set to zeros so camera_info keeps emitting a plumb_bob model that happens to
 * be trivial.
 */
class Rectifier {
   public:
    Rectifier() = default;
    /** Build the maps. `size` may differ from the calibration's, in which case the intrinsics are
     *  scaled to it, so a calibration shot at 1920x1200 still rectifies a 1280x720 capture. */
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
