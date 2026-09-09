#pragma once

#include <opencv2/opencv.hpp>
#include <string>

#include "data_structures.hpp"

namespace auto_battlebot {
/**
 * @brief Intrinsics and distortion for one physical camera, measured once.
 *
 * The ZED handed us rectified frames and near-zero distortion coefficients. A 104.6 degree M12
 * lens does not, and every downstream consumer assumes a pinhole: the homography fit, keypoint
 * projection, the UI marker overlay, and the trained detectors. So the camera rectifies before
 * publishing, and the `CameraInfo` it emits carries the rectified matrix with D zeroed.
 *
 * Calibrate through polycarbonate. The camera sits behind a cage panel, and a flat sheet in the
 * optical path refracts increasingly toward the frame edges, which is exactly where the field
 * boundary sits. Calibrating in free air folds that error into the field pose where no
 * reprojection residual will ever show it.
 */
struct CameraCalibration {
    /** Recorded in MCAP metadata, so a recording says which calibration rectified it. */
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

/** Load from `config/cameras/<serial>.toml`, resolved absolute or relative to the project root. */
CameraCalibration load_camera_calibration(const std::string &path);
void save_camera_calibration(const std::string &path, const CameraCalibration &calibration);

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

    /** Intrinsics after rectification, with distortion zeroed. */
    CameraInfo camera_info() const;

   private:
    cv::Mat map_x_;
    cv::Mat map_y_;
    cv::Mat rectified_matrix_;
    cv::Size size_;
};
}  // namespace auto_battlebot
