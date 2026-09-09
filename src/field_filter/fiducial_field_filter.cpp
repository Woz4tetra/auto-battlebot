#include "field_filter/fiducial_field_filter.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <numeric>
#include <opencv2/calib3d.hpp>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "field_filter/field_pose.hpp"

namespace auto_battlebot {
namespace {
/** Detector tuned for small, distant 36h11 tags. The hard limit is pixels on target: face on,
 *  36h11 decodes reliably only above about 18 px of edge and no parameter recovers a marker below
 *  about 15 px. Within that budget these settings buy margin. The adaptive-threshold window range
 *  is deliberately tight: windows above 21 px cost 33 ms per 1080p frame against 11 ms and added
 *  no detections once auto_gamma normalized brightness. */
cv::aruco::DetectorParameters tuned_parameters() {
    cv::aruco::DetectorParameters parameters;
    parameters.adaptiveThreshWinSizeMin = 5;
    parameters.adaptiveThreshWinSizeMax = 21;
    parameters.adaptiveThreshWinSizeStep = 8;
    parameters.minMarkerPerimeterRate = 0.02;
    parameters.polygonalApproxAccuracyRate = 0.05;
    parameters.errorCorrectionRate = 1.0;
    parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters.cornerRefinementWinSize = 5;
    parameters.cornerRefinementMinAccuracy = 0.05;
    return parameters;
}

Eigen::Matrix3d rotation_z(double radians) {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation(0, 0) = std::cos(radians);
    rotation(0, 1) = -std::sin(radians);
    rotation(1, 0) = std::sin(radians);
    rotation(1, 1) = std::cos(radians);
    return rotation;
}

Eigen::Matrix3d rotation_x(double radians) {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation(1, 1) = std::cos(radians);
    rotation(1, 2) = -std::sin(radians);
    rotation(2, 1) = std::sin(radians);
    rotation(2, 2) = std::cos(radians);
    return rotation;
}
}  // namespace

std::vector<int> floor_board_ids(int cols, int rows, int first_id) {
    std::vector<int> ids;
    ids.reserve(static_cast<size_t>(cols) * static_cast<size_t>(rows));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            // Reverse within the row: the printed board numbers right to left, GridBoard fills
            // left to right.
            ids.push_back(first_id + row * cols + (cols - 1 - col));
        }
    }
    return ids;
}

cv::Mat auto_gamma(const cv::Mat &frame, double target_mean, double min_mean) {
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }
    const double mean = cv::mean(gray)[0];
    if (mean >= min_mean || mean < 1.0) {
        return frame;
    }
    const double power = std::log(target_mean / 255.0) / std::log(mean / 255.0);
    cv::Mat lookup(1, 256, CV_8U);
    for (int value = 0; value < 256; ++value) {
        lookup.at<uint8_t>(0, value) =
            cv::saturate_cast<uint8_t>(255.0 * std::pow(value / 255.0, power));
    }
    cv::Mat out;
    cv::LUT(frame, lookup, out);
    return out;
}

FiducialFieldFilter::FiducialFieldFilter(const FiducialFieldFilterConfiguration &config)
    : config_(config),
      dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)),
      board_(cv::Size(config.board_cols, config.board_rows), static_cast<float>(config.marker_size),
             static_cast<float>(config.marker_separation), dictionary_,
             cv::Mat(floor_board_ids(config.board_cols, config.board_rows, config.first_marker_id),
                     true)),
      detector_(dictionary_, tuned_parameters()),
      board_ids_(floor_board_ids(config.board_cols, config.board_rows, config.first_marker_id)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("fiducial_field_filter")) {
    spdlog::info("FiducialFieldFilter: {}x{} board, ids {}..{}, {:.0f} mm markers",
                 config.board_cols, config.board_rows, config.first_marker_id,
                 config.first_marker_id + config.board_cols * config.board_rows - 1,
                 config.marker_size * 1000.0);
}

Eigen::Matrix4d FiducialFieldFilter::tf_fieldcenter_from_board() const {
    const double sign_x =
        (config_.corner == FieldCorner::NEG_X_NEG_Y || config_.corner == FieldCorner::NEG_X_POS_Y)
            ? -1.0
            : 1.0;
    const double sign_y =
        (config_.corner == FieldCorner::NEG_X_NEG_Y || config_.corner == FieldCorner::POS_X_NEG_Y)
            ? -1.0
            : 1.0;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    // Field z points away from the camera, into the floor, which is the convention the mask
    // outline fit produces. A board lying flat therefore shares the field's axes outright, and
    // pitch is negated so that +90 stands the board up with its printed y axis pointing away
    // from the floor rather than into it.
    transform.block<3, 3>(0, 0) = rotation_z(config_.board_yaw_deg * CV_PI / 180.0) *
                                  rotation_x(-config_.board_pitch_deg * CV_PI / 180.0);
    transform(0, 3) = sign_x * config_.field_size_x / 2.0 + config_.board_offset_x;
    transform(1, 3) = sign_y * config_.field_size_y / 2.0 + config_.board_offset_y;
    transform(2, 3) = config_.board_offset_z;
    return transform;
}

std::shared_ptr<FieldDescriptionWithInlierPoints> FiducialFieldFilter::compute_field(
    const CameraData &camera_data, [[maybe_unused]] const MaskStamped &field_mask) {
    FunctionTimer timer(diagnostics_logger_, "compute_field");
    auto failed = std::make_shared<FieldDescriptionWithInlierPoints>();

    if (camera_data.rgb.image.empty()) {
        spdlog::warn("FiducialFieldFilter: empty camera image");
        return failed;
    }
    const cv::Mat intrinsics = camera_data.camera_info.intrinsics;
    Eigen::Matrix3d intrinsics_eigen;
    if (!intrinsics_to_eigen(intrinsics, intrinsics_eigen)) {
        spdlog::warn("FiducialFieldFilter: camera_info carries no usable intrinsics");
        return failed;
    }

    const cv::Mat image = auto_gamma(camera_data.rgb.image);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detector_.detectMarkers(image, corners, ids);

    std::vector<std::vector<cv::Point2f>> kept_corners;
    std::vector<int> kept_ids;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (std::find(board_ids_.begin(), board_ids_.end(), ids[i]) != board_ids_.end()) {
            kept_corners.push_back(corners[i]);
            kept_ids.push_back(ids[i]);
        }
    }
    if (static_cast<int>(kept_ids.size()) < config_.min_markers) {
        spdlog::warn("FiducialFieldFilter: saw {} of the board's markers, need {}", kept_ids.size(),
                     config_.min_markers);
        return failed;
    }

    cv::Mat object_points;
    cv::Mat image_points;
    board_.matchImagePoints(kept_corners, kept_ids, object_points, image_points);
    if (object_points.empty() || image_points.empty()) {
        spdlog::warn("FiducialFieldFilter: no image points matched the board");
        return failed;
    }
    for (int i = 0; i < object_points.rows; ++i) {
        accumulated_object_.push_back(object_points.at<cv::Point3f>(i));
        accumulated_image_.push_back(image_points.at<cv::Point2f>(i));
    }
    ++accumulated_frames_;
    if (accumulated_frames_ < config_.accumulate_frames) {
        spdlog::info("FiducialFieldFilter: {}/{} frames accumulated", accumulated_frames_,
                     config_.accumulate_frames);
        return failed;
    }

    // The recording carries raw frames, so distortion has already been removed upstream by the
    // camera's rectification and D is zero here.
    const cv::Mat distortion = camera_data.camera_info.distortion.empty()
                                   ? cv::Mat::zeros(1, 5, CV_64F)
                                   : camera_data.camera_info.distortion;
    cv::Mat rotation_vector;
    cv::Mat translation_vector;
    if (!cv::solvePnP(accumulated_object_, accumulated_image_, intrinsics, distortion,
                      rotation_vector, translation_vector)) {
        spdlog::warn("FiducialFieldFilter: PnP failed; check the intrinsics and marker_size");
        accumulated_object_.clear();
        accumulated_image_.clear();
        accumulated_frames_ = 0;
        return failed;
    }

    // 15 markers give 60 correspondences against 6 unknowns, so this residual measures something,
    // unlike the four-corner homography's. It catches a mis-measured marker_size, a board printed
    // at "fit to page" rather than 100%, a mirrored id mapping, and a board that was not flat.
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(accumulated_object_, rotation_vector, translation_vector, intrinsics,
                      distortion, reprojected);
    double error = 0.0;
    for (size_t i = 0; i < reprojected.size(); ++i) {
        error += cv::norm(reprojected[i] - accumulated_image_[i]);
    }
    error /= static_cast<double>(reprojected.size());

    const size_t correspondences = accumulated_object_.size();
    const int frames = accumulated_frames_;
    accumulated_object_.clear();
    accumulated_image_.clear();
    accumulated_frames_ = 0;

    diagnostics_logger_->debug("compute_field",
                               {{"frames", std::to_string(frames)},
                                {"correspondences", std::to_string(correspondences)},
                                {"reprojection_px", std::to_string(error)}});
    if (error > config_.max_reprojection_error_px) {
        spdlog::warn(
            "FiducialFieldFilter: {:.1f} px reprojection error over {} correspondences (limit "
            "{:.1f} px). Check marker_size, that the board printed at 100%, and that it was flat.",
            error, correspondences, config_.max_reprojection_error_px);
        return failed;
    }

    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);
    Eigen::Matrix4d tf_camera_from_board = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            tf_camera_from_board(row, col) = rotation_matrix.at<double>(row, col);
        }
        tf_camera_from_board(row, 3) = translation_vector.at<double>(row);
    }

    auto description = std::make_shared<FieldDescriptionWithInlierPoints>();
    description->header.stamp = camera_data.rgb.header.stamp;
    description->header.frame_id = FrameId::CAMERA_WORLD;
    description->child_frame_id = FrameId::FIELD;
    description->tf_camera_from_fieldcenter.tf =
        tf_camera_from_board * tf_fieldcenter_from_board().inverse();
    description->size.header = camera_data.rgb.header;
    description->size.size = Size{config_.field_size_x, config_.field_size_y, 0.0};
    spdlog::info("FiducialFieldFilter: field locked from {} frames, {:.2f} px reprojection error",
                 frames, error);
    return description;
}
}  // namespace auto_battlebot
