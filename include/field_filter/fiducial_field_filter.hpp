#pragma once

#include <memory>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "field_filter/camera_world_field_filter.hpp"
#include "field_filter/config.hpp"

namespace auto_battlebot {
/** Marker ids for a manufactured grid board, reversed within each row.
 *
 * The physical board numbers markers right to left within a row; cv::aruco::GridBoard fills them
 * left to right. Pairing them the wrong way round gives a reflection rather than a rotation, and
 * a reflection still decodes: the measured cost was 440 px of reprojection error against 16 px
 * when correct. This is the same class of failure as the mirrored robot tag mesh, and the worse
 * version of it, because everything upstream looks like it is working. */
std::vector<int> floor_board_ids(int cols, int rows, int first_id);

/** Brighten an underexposed frame with a gamma curve. Arena lighting is often dim enough that a
 *  raw frame decodes zero tags because the marker contrast is crushed into shadow; the stretch is
 *  monotonic and per channel, so corner geometry and the recovered pose are unchanged. */
cv::Mat auto_gamma(const cv::Mat &frame, double target_mean = 120.0, double min_mean = 110.0);

/**
 * @brief Field pose from an AprilTag board, ignoring the field mask.
 *
 * The mode for a venue nobody has surveyed. It depends on nothing we have to segment and on no
 * assumed mat size, so a clipped, cluttered or badly segmented outline costs it nothing. It also
 * replays: detection runs off `camera_data.rgb`, so a recording whose first frames contain the
 * board carries everything needed to re-derive the field transform later. Record ten seconds
 * with the board down before pulling it, every time.
 */
class FiducialFieldFilter : public CameraWorldFieldFilter {
   public:
    explicit FiducialFieldFilter(const FiducialFieldFilterConfiguration &config);
    ~FiducialFieldFilter() override = default;

    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(
        const CameraData &camera_data, const MaskStamped &field_mask) override;

    /** Board origin (the grid's minimum-x, minimum-y printed corner) expressed in the field
     *  centre frame, composed from the configured corner, offsets and orientation. */
    Eigen::Matrix4d tf_fieldcenter_from_board() const;

   protected:
    const char *name() const override { return "FiducialFieldFilter"; }

   private:
    FiducialFieldFilterConfiguration config_;
    cv::aruco::Dictionary dictionary_;
    cv::aruco::GridBoard board_;
    cv::aruco::ArucoDetector detector_;
    std::vector<int> board_ids_;

    /** Correspondences stacked across frames, so detection noise averages out before the pose
     *  latches. Cleared by reset. */
    std::vector<cv::Point3f> accumulated_object_;
    std::vector<cv::Point2f> accumulated_image_;
    int accumulated_frames_ = 0;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
