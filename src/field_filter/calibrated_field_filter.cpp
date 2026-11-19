#include "field_filter/calibrated_field_filter.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "field_filter/field_outline.hpp"
#include "field_filter/field_pose.hpp"

namespace auto_battlebot {
CageCalibration load_calibration_in_live_outline_convention(const std::string &path) {
    CageCalibration calibration = load_cage_calibration(path);
    calibration.tf_camera_from_fieldcenter = align_field_x_with_camera_x(
        calibration.tf_camera_from_fieldcenter, calibration.measured_mat_size_x_m,
        calibration.measured_mat_size_y_m);
    return calibration;
}

CalibratedFieldFilter::CalibratedFieldFilter(const CalibratedFieldFilterConfiguration &config)
    : config_(config),
      calibration_(load_calibration_in_live_outline_convention(config.calibration_file)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("calibrated_field_filter")) {
    spdlog::info("Loaded cage calibration '{}' ({:.3f} x {:.3f} m) from {}",
                 calibration_.calibration_id, calibration_.measured_mat_size_x_m,
                 calibration_.measured_mat_size_y_m, config.calibration_file);
}

void CalibratedFieldFilter::run_seating_check(const CameraData &camera_data,
                                              const MaskStamped &field_mask) {
    Eigen::Matrix3d intrinsics;
    if (!intrinsics_to_eigen(camera_data.camera_info.intrinsics, intrinsics)) {
        spdlog::warn("Seating check skipped: camera_info carries no usable intrinsics");
        return;
    }
    const cv::Mat contour_mask = find_largest_contour_mask(field_mask.mask.mask);
    if (cv::countNonZero(contour_mask) == 0) {
        spdlog::warn("Seating check skipped: empty field mask");
        return;
    }

    FieldOutlineParams outline_params;
    outline_params.border_margin_px = config_.border_margin_px;
    outline_params.refine_iterations = config_.refine_iterations;
    outline_params.corner_skip_fraction_per_side_end = config_.corner_skip_fraction;
    outline_params.min_side_points = config_.min_side_points;
    const FieldOutline outline = extract_field_outline(contour_mask, outline_params);
    if (!outline.ok) {
        spdlog::warn("Seating check skipped: {}", outline.failure);
        return;
    }

    const auto expected = project_field_corners(calibration_.tf_camera_from_fieldcenter,
                                                calibration_.measured_mat_size_x_m,
                                                calibration_.measured_mat_size_y_m, intrinsics);
    const FieldPoseResult live = pose_from_outline(outline, calibration_.measured_mat_size_x_m,
                                                   calibration_.measured_mat_size_y_m, intrinsics);
    if (!live.ok) {
        spdlog::warn("Seating check skipped: {}", live.failure);
        return;
    }
    const auto observed =
        project_field_corners(live.tf_camera_from_fieldcenter, calibration_.measured_mat_size_x_m,
                              calibration_.measured_mat_size_y_m, intrinsics);

    double worst = 0.0;
    for (size_t i = 0; i < expected.size(); ++i) {
        worst = std::max(worst, cv::norm(expected[i] - observed[i]));
    }
    diagnostics_logger_->debug("seating_check",
                               {{"worst_corner_error_px", std::to_string(worst)},
                                {"supported_sides", std::to_string(outline.supported_sides)}});
    if (worst > config_.max_seating_error_px) {
        spdlog::warn(
            "Seating check: live field outline is {:.1f} px from calibration '{}' (limit {:.1f} "
            "px). The fixture may not have seated or the cage may have moved. Publishing the "
            "stored calibration anyway.",
            worst, calibration_.calibration_id, config_.max_seating_error_px);
    } else {
        spdlog::info("Seating check: live field outline within {:.1f} px of calibration '{}'",
                     worst, calibration_.calibration_id);
    }
}

std::shared_ptr<FieldDescriptionWithInlierPoints> CalibratedFieldFilter::compute_field(
    const CameraData &camera_data, const MaskStamped &field_mask) {
    FunctionTimer timer(diagnostics_logger_, "compute_field");
    if (config_.seating_check) {
        run_seating_check(camera_data, field_mask);
    }

    auto description = std::make_shared<FieldDescriptionWithInlierPoints>();
    description->header.stamp = camera_data.rgb.header.stamp;
    description->header.frame_id = FrameId::CAMERA_WORLD;
    description->child_frame_id = FrameId::FIELD;
    description->tf_camera_from_fieldcenter.tf = calibration_.tf_camera_from_fieldcenter;
    description->size.header = camera_data.rgb.header;
    description->size.size =
        Size{calibration_.measured_mat_size_x_m, calibration_.measured_mat_size_y_m, 0.0};
    return description;
}
}  // namespace auto_battlebot
