#pragma once

#include <array>
#include <opencv2/opencv.hpp>
#include <string>

namespace auto_battlebot {
/** Largest external contour of a mask, filled. Shared by every filter that works off the field
 *  mask so there is one definition of "the field blob". */
cv::Mat find_largest_contour_mask(const cv::Mat &mask);

struct FieldOutlineParams {
    /** Contour points this close to the frame edge are dropped before the edge fit. They sit
     *  where the field crosses the image border, not where the field ends, and a line fitted
     *  through them is the border rather than the mat edge. */
    int border_margin_px = 2;
    int refine_iterations = 4;
    /** Fraction of each side dropped at both ends, as belonging to the mat's rounded corners. */
    double corner_skip_fraction = 0.20;
    /** A side with fewer surviving points than this is unsupported: no fit, no constraint. */
    int min_side_points = 20;
};

/**
 * @brief The field mask reduced to four image-plane sides.
 *
 * Corner i sits between side i-1 and side i, wound counter-clockwise from the smallest x + y.
 * Sides 0 and 2 are opposite, as are 1 and 3.
 *
 * A clipped outline is the expected case on a cage-mounted camera: the near mat corners fall
 * outside a 87-degree horizontal field at any practical mount height. So an unsupported side is
 * reported rather than rejected, and `corners` may hold coordinates off the sensor. Callers
 * decide what to do with three sides; `FieldPose` solves the pose from them directly.
 */
struct FieldOutline {
    bool ok = false;
    std::string failure;
    std::array<cv::Point2d, 4> corners{};
    /** Homogeneous image line for each side. Side i runs corners[i] -> corners[i+1]. */
    std::array<cv::Vec3d, 4> lines{};
    std::array<bool, 4> side_supported{};
    /** Scatter of each side's own points about its fitted line, or -1 for a side that was never
     *  fitted because it had too few points. Reported, not acted on: a curved side still counts
     *  as support. It is the number that says a fit should be distrusted. */
    std::array<double, 4> side_rms_px{-1.0, -1.0, -1.0, -1.0};
    int supported_sides = 0;
    /** Mask area over quad area. Above ~1.05 the outline is not the quadrilateral the fit
     *  assumes, so at least one side is not a field edge. Reprojection error cannot see this. */
    double mask_over_quad_area = 0.0;
};

FieldOutline extract_field_outline(const cv::Mat &contour_mask, const FieldOutlineParams &params);
}  // namespace auto_battlebot
