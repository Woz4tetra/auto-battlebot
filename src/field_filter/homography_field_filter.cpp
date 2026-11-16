#include "field_filter/homography_field_filter.hpp"

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include <algorithm>

#include "diagnostics_logger/function_timer.hpp"
#include "field_filter/field_outline.hpp"
#include "field_filter/field_pose.hpp"

namespace auto_battlebot {
HomographyFieldFilter::HomographyFieldFilter(const HomographyFieldFilterConfiguration &config)
    : config_(config),
      diagnostics_logger_(DiagnosticsLogger::get_logger("homography_field_filter")) {}

std::shared_ptr<FieldDescriptionWithInlierPoints> HomographyFieldFilter::compute_field(
    const CameraData &camera_data, const MaskStamped &field_mask) {
    FunctionTimer timer(diagnostics_logger_, "compute_field");
    auto failed = std::make_shared<FieldDescriptionWithInlierPoints>();
    const auto report_rate_limited_failure = [this](const std::string &message) {
        if (consecutive_fit_failures_++ == 0) {
            spdlog::warn("HomographyFieldFilter: {}", message);
        } else {
            spdlog::debug("HomographyFieldFilter: {}", message);
        }
    };

    Eigen::Matrix3d intrinsics;
    if (!intrinsics_to_eigen(camera_data.camera_info.intrinsics, intrinsics)) {
        report_rate_limited_failure("camera_info carries no usable intrinsics");
        return failed;
    }

    const cv::Mat contour_mask = find_largest_contour_mask(field_mask.mask.mask);
    if (cv::countNonZero(contour_mask) == 0) {
        report_rate_limited_failure("empty field mask");
        return failed;
    }

    FieldOutlineParams outline_params;
    outline_params.border_margin_px = config_.border_margin_px;
    outline_params.refine_iterations = config_.refine_iterations;
    outline_params.corner_skip_fraction = config_.corner_skip_fraction;
    outline_params.min_side_points = config_.min_side_points;
    const FieldOutline outline = extract_field_outline(contour_mask, outline_params);
    if (!outline.ok) {
        report_rate_limited_failure(outline.failure);
        return failed;
    }
    if (outline.mask_over_quad_area > config_.max_quad_coverage) {
        // Fail closed. A silent bad pose is the failure mode this design exists to avoid, and
        // the residual cannot see this one: the fit matches whatever corners it is handed.
        report_rate_limited_failure(
            fmt::format("mask fills {:.2f} of its own quad (limit {:.2f}); the outline is not the "
                        "quadrilateral this fit assumes",
                        outline.mask_over_quad_area, config_.max_quad_coverage));
        return failed;
    }

    const FieldPoseResult pose =
        pose_from_outline(outline, config_.field_size_x, config_.field_size_y, intrinsics);
    if (!pose.ok) {
        report_rate_limited_failure(pose.failure);
        return failed;
    }

    diagnostics_logger_->debug(
        "compute_field", {{"supported_sides", std::to_string(outline.supported_sides)},
                          {"mask_over_quad_area", std::to_string(outline.mask_over_quad_area)},
                          {"residual_px", std::to_string(pose.residual_px)},
                          {"side_rms_px", std::to_string(outline.side_rms_px[0]) + " " +
                                              std::to_string(outline.side_rms_px[1]) + " " +
                                              std::to_string(outline.side_rms_px[2]) + " " +
                                              std::to_string(outline.side_rms_px[3])}});
    consecutive_fit_failures_ = 0;
    spdlog::info(
        "HomographyFieldFilter: {} sides, {:.1f} px reprojection error, side scatter {:.1f} {:.1f} "
        "{:.1f} {:.1f} px, mask fills {:.2f} of its quad",
        outline.supported_sides, pose.residual_px, outline.side_rms_px[0], outline.side_rms_px[1],
        outline.side_rms_px[2], outline.side_rms_px[3], outline.mask_over_quad_area);
    if (outline.supported_sides == 3) {
        const size_t dropped = static_cast<size_t>(
            std::find(outline.side_supported.begin(), outline.side_supported.end(), false) -
            outline.side_supported.begin());
        spdlog::info(
            "HomographyFieldFilter: pose solved from three edges; side {} was {}", dropped,
            outline.side_rms_px[dropped] < 0.0 ? "off the frame" : "not straight enough to trust");
    }

    auto description = std::make_shared<FieldDescriptionWithInlierPoints>();
    description->header.stamp = camera_data.rgb.header.stamp;
    description->header.frame_id = FrameId::CAMERA_WORLD;
    description->child_frame_id = FrameId::FIELD;
    description->tf_camera_from_fieldcenter.tf = pose.tf_camera_from_fieldcenter;
    description->size.header = camera_data.rgb.header;
    description->size.size = Size{config_.field_size_x, config_.field_size_y, 0.0};
    return description;
}
}  // namespace auto_battlebot
