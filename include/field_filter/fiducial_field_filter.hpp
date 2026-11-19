#pragma once

#include <memory>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "field_filter/camera_world_field_filter.hpp"
#include "field_filter/config.hpp"

namespace auto_battlebot {
std::vector<int> floor_board_ids(int cols, int rows, int first_id);

cv::aruco::DetectorParameters small_apriltag_detector_parameters();

cv::Mat auto_gamma(const cv::Mat &frame, double target_mean = 120.0, double min_mean = 110.0);

class FiducialFieldFilter : public CameraWorldFieldFilter {
   public:
    explicit FiducialFieldFilter(const FiducialFieldFilterConfiguration &config);
    ~FiducialFieldFilter() override = default;

    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(
        const CameraData &camera_data, const MaskStamped &field_mask) override;

    Eigen::Matrix4d tf_fieldcenter_from_board() const;

   protected:
    const char *filter_name_for_log() const override { return "FiducialFieldFilter"; }

   private:
    FiducialFieldFilterConfiguration config_;
    cv::aruco::Dictionary dictionary_;
    cv::aruco::GridBoard board_;
    cv::aruco::ArucoDetector detector_;
    std::vector<int> board_ids_;

    std::vector<cv::Point3f> accumulated_object_;
    std::vector<cv::Point2f> accumulated_image_;
    int accumulated_frames_ = 0;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
