#include "field_filter/homography_field_filter.hpp"

#include <spdlog/spdlog.h>

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

    Eigen::Matrix3d intrinsics;
    if (!intrinsics_to_eigen(camera_data.camera_info.intrinsics, intrinsics)) {
        spdlog::warn("HomographyFieldFilter: camera_info carries no usable intrinsics");
        return failed;
    }

    const cv::Mat contour_mask = find_largest_contour_mask(field_mask.mask.mask);
    if (cv::countNonZero(contour_mask) == 0) {
        spdlog::warn("HomographyFieldFilter: empty field mask");
        return failed;
    }

    FieldOutlineParams outline_params;
    outline_params.border_margin_px = config_.border_margin_px;
    outline_params.refine_iterations = config_.refine_iterations;
    outline_params.corner_skip_fraction = config_.corner_skip_fraction;
    outline_params.min_side_points = config_.min_side_points;
    const FieldOutline outline = extract_field_outline(contour_mask, outline_params);
    if (!outline.ok) {
        spdlog::warn("HomographyFieldFilter: {}", outline.failure);
        return failed;
    }
    if (outline.mask_over_quad_area > config_.max_quad_coverage) {
        // Fail closed. A silent bad pose is the failure mode this design exists to avoid, and
        // the residual cannot see this one: the fit matches whatever corners it is handed.
        spdlog::warn(
            "HomographyFieldFilter: mask fills {:.2f} of its own quad (limit {:.2f}); the outline "
            "is not the quadrilateral this fit assumes",
            outline.mask_over_quad_area, config_.max_quad_coverage);
        return failed;
    }

    const FieldPoseResult pose =
        pose_from_outline(outline, config_.field_size_x, config_.field_size_y, intrinsics);
    if (!pose.ok) {
        spdlog::warn("HomographyFieldFilter: {}", pose.failure);
        return failed;
    }

    diagnostics_logger_->debug(
        "compute_field", {{"supported_sides", std::to_string(outline.supported_sides)},
                          {"mask_over_quad_area", std::to_string(outline.mask_over_quad_area)},
                          {"residual_px", std::to_string(pose.residual_px)}});
    if (outline.supported_sides == 3) {
        spdlog::info(
            "HomographyFieldFilter: field clipped by the frame, pose solved from three edges");
    }

    auto description = std::make_shared<FieldDescriptionWithInlierPoints>();
    description->header.stamp = camera_data.rgb.header.stamp;
    description->header.frame_id = FrameId::CAMERA_WORLD;
    description->child_frame_id = FrameId::FIELD;
    description->tf_camera_from_fieldcenter.tf = pose.tf_camera_from_fieldcenter;
    description->size.header = camera_data.rgb.header;
    description->size.size = Size{config_.field_size_x, config_.field_size_y, 0.0};
    // inlier_points stays empty. Its one consumer, to_field_point_cloud, returns nullopt on an
    // empty cloud and the publisher already handles that, so /field_points simply does not
    // publish. The field rectangle still does.
    return description;
}
}  // namespace auto_battlebot
