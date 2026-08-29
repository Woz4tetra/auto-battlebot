#include "field_filter/point_cloud_field_filter_debug.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <utility>

namespace auto_battlebot {
namespace {

using DebugPanels = std::vector<std::pair<std::string, cv::Mat>>;

constexpr int kScatterWidth = 800;
constexpr int kScatterHeight = 600;
constexpr int kCellWidth = 480;
constexpr int kCellHeight = 360;
constexpr float kFlattenedPadFraction = 0.1f;

const cv::Scalar kBackgroundColor(30, 30, 30);
const cv::Scalar kAxisColor(100, 220, 100);
const cv::Scalar kLabelColor(0, 255, 0);
const cv::Scalar kLegendColor(200, 200, 200);
const cv::Scalar kRectangleColor(50, 200, 255);
const cv::Scalar kCornerColor(255, 100, 50);
const cv::Scalar kNearestEdgeColor(0, 50, 255);
const cv::Scalar kYawLabelColor(0, 200, 255);
const cv::Scalar kCellBorderColor(60, 60, 60);
const cv::Vec3b kPointColor(200, 200, 200);
const cv::Vec3b kInlierColor(50, 200, 50);
const cv::Vec3b kOutlierColor(50, 50, 200);

// Fixed-size canvas that maps a 2D point range onto pixels, with Y growing upward.
// Every scatter panel shares this so the three stages stay visually comparable.
class ScatterCanvas {
   public:
    ScatterCanvas(const cv::Point2f &min_corner, const cv::Point2f &max_corner,
                  float pad_fraction = 0.0f)
        : image_(kScatterHeight, kScatterWidth, CV_8UC3, kBackgroundColor) {
        cv::Point2f padding = (max_corner - min_corner) * pad_fraction;
        min_corner_ = min_corner - padding;
        cv::Point2f padded_max = max_corner + padding;
        range_ = cv::Point2f(std::max(padded_max.x - min_corner_.x, kMinimumRange),
                             std::max(padded_max.y - min_corner_.y, kMinimumRange));
    }

    cv::Point to_pixel(float x, float y) const {
        return {static_cast<int>((x - min_corner_.x) / range_.x * (kScatterWidth - 1)),
                kScatterHeight - 1 -
                    static_cast<int>((y - min_corner_.y) / range_.y * (kScatterHeight - 1))};
    }

    void plot(const cv::Point2f &point, const cv::Vec3b &color) {
        cv::Point pixel = to_pixel(point.x, point.y);
        if (pixel.x < 0 || pixel.x >= kScatterWidth || pixel.y < 0 || pixel.y >= kScatterHeight) {
            return;
        }
        image_.at<cv::Vec3b>(pixel) = color;
    }

    void label_axes(const std::string &x_label, const std::string &y_label) {
        cv::putText(image_, x_label, cv::Point(kScatterWidth - 75, kScatterHeight - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, kAxisColor, 1);
        cv::putText(image_, y_label, cv::Point(5, 22), cv::FONT_HERSHEY_SIMPLEX, 0.5, kAxisColor,
                    1);
    }

    const cv::Mat &image() const { return image_; }

   private:
    static constexpr float kMinimumRange = 1e-6f;

    cv::Mat image_;
    cv::Point2f min_corner_;
    cv::Point2f range_;
};

// Writes the bounds through out-params rather than returning a pair. cv::Point2f is not
// trivially copyable, so an aggregate holding two of them lands on the aarch64 calling
// convention that changed between C++14 and C++17 and every caller draws a -Wpsabi note.
void compute_extents(const std::vector<cv::Point2f> &points, cv::Point2f &min_corner,
                     cv::Point2f &max_corner) {
    if (points.empty()) {
        min_corner = cv::Point2f(0.0f, 0.0f);
        max_corner = cv::Point2f(1.0f, 1.0f);
        return;
    }
    min_corner = cv::Point2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    max_corner =
        cv::Point2f(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    for (const cv::Point2f &point : points) {
        min_corner.x = std::min(min_corner.x, point.x);
        min_corner.y = std::min(min_corner.y, point.y);
        max_corner.x = std::max(max_corner.x, point.x);
        max_corner.y = std::max(max_corner.y, point.y);
    }
}

// Project the cloud onto the camera X-Z plane, the view that shows the field edge-on.
std::vector<cv::Point2f> cloud_to_xz(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    std::vector<cv::Point2f> points;
    points.reserve(cloud->points.size());
    for (const pcl::PointXYZ &point : cloud->points) {
        points.emplace_back(point.x, point.z);
    }
    return points;
}

std::vector<cv::Point2f> to_points(const std::vector<Eigen::Vector2f> &values) {
    std::vector<cv::Point2f> points;
    points.reserve(values.size());
    for (const Eigen::Vector2f &value : values) {
        points.emplace_back(value.x(), value.y());
    }
    return points;
}

cv::Mat colorize_depth(const cv::Mat &depth_image) {
    cv::Mat normalized;
    cv::normalize(depth_image, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);

    // NaN pixels normalize to arbitrary values, so blank them out.
    for (int v = 0; v < depth_image.rows; ++v) {
        for (int u = 0; u < depth_image.cols; ++u) {
            if (std::isnan(depth_image.at<float>(v, u))) {
                colorized.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    return colorized;
}

cv::Mat colorize_mask(const cv::Mat &mask) {
    cv::Mat binary;
    cv::threshold(mask, binary, 0, 255, cv::THRESH_BINARY);
    cv::Mat colorized;
    cv::cvtColor(binary, colorized, cv::COLOR_GRAY2BGR);
    return colorized;
}

// Mirrors get_rectangle_angle: the edge with the lowest midpoint Y sets the field yaw.
size_t nearest_edge_index(const std::vector<Eigen::Vector2f> &corners) {
    size_t nearest = 0;
    float lowest_midpoint_y = std::numeric_limits<float>::max();
    for (size_t i = 0; i < corners.size(); ++i) {
        float midpoint_y = (corners[i].y() + corners[(i + 1) % corners.size()].y()) / 2.0f;
        if (midpoint_y < lowest_midpoint_y) {
            lowest_midpoint_y = midpoint_y;
            nearest = i;
        }
    }
    return nearest;
}

void add_mask_panels(const FieldFilterDebugFrame &frame, DebugPanels &panels) {
    panels.emplace_back("Original Mask", colorize_mask(frame.original_mask));
    panels.emplace_back("Largest Contour", colorize_mask(frame.largest_contour_mask));
    panels.emplace_back("Masked Depth", colorize_depth(frame.masked_depth_image));
}

void add_point_cloud_panels(const FieldFilterDebugFrame &frame, DebugPanels &panels) {
    std::vector<cv::Point2f> points = cloud_to_xz(frame.field_cloud);
    cv::Point2f min_corner;
    cv::Point2f max_corner;
    compute_extents(points, min_corner, max_corner);

    std::vector<bool> is_inlier(points.size(), false);
    for (int index : frame.inlier_indices) {
        if (index >= 0 && static_cast<size_t>(index) < is_inlier.size()) {
            is_inlier[static_cast<size_t>(index)] = true;
        }
    }

    // Both panels share extents so the inlier split lines up with the raw cloud.
    ScatterCanvas raw_cloud(min_corner, max_corner);
    ScatterCanvas classified_cloud(min_corner, max_corner);
    for (size_t i = 0; i < points.size(); ++i) {
        raw_cloud.plot(points[i], kPointColor);
        classified_cloud.plot(points[i], is_inlier[i] ? kInlierColor : kOutlierColor);
    }
    raw_cloud.label_axes("X ->", "^ Z (depth)");
    classified_cloud.label_axes("X ->", "^ Z (depth)");

    cv::Mat classified_image = classified_cloud.image().clone();
    cv::putText(classified_image, "green = inliers, red = outliers",
                cv::Point(10, kScatterHeight - 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, kLegendColor, 1);

    panels.emplace_back("Point Cloud (X-Z)", raw_cloud.image().clone());
    panels.emplace_back("RANSAC Inliers", classified_image);
}

void add_rectangle_panels(const FieldFilterDebugFrame &frame, DebugPanels &panels) {
    std::vector<cv::Point2f> points = to_points(frame.flattened_cloud_2d);
    cv::Point2f min_corner;
    cv::Point2f max_corner;
    compute_extents(points, min_corner, max_corner);

    ScatterCanvas flattened(min_corner, max_corner, kFlattenedPadFraction);
    for (const cv::Point2f &point : points) {
        flattened.plot(point, kPointColor);
    }
    flattened.label_axes("X ->", "^ Y");
    panels.emplace_back("Flattened Cloud", flattened.image().clone());

    std::vector<cv::Point> corner_pixels;
    corner_pixels.reserve(frame.rectangle_corners.size());
    for (const Eigen::Vector2f &corner : frame.rectangle_corners) {
        corner_pixels.push_back(flattened.to_pixel(corner.x(), corner.y()));
    }

    cv::Mat rectangle_image = flattened.image().clone();
    for (size_t i = 0; i < corner_pixels.size(); ++i) {
        cv::line(rectangle_image, corner_pixels[i], corner_pixels[(i + 1) % corner_pixels.size()],
                 kRectangleColor, 2);
        cv::circle(rectangle_image, corner_pixels[i], 5, kCornerColor, -1);
    }
    panels.emplace_back("Minimum Rectangle", rectangle_image);

    cv::Mat angle_image = rectangle_image.clone();
    if (!corner_pixels.empty()) {
        size_t nearest = nearest_edge_index(frame.rectangle_corners);
        cv::line(angle_image, corner_pixels[nearest],
                 corner_pixels[(nearest + 1) % corner_pixels.size()], kNearestEdgeColor, 3);
    }
    std::string yaw_label =
        "yaw: " + std::to_string(static_cast<int>(frame.rectangle_angle * 180.0 / M_PI)) + " deg";
    cv::putText(angle_image, yaw_label, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                kYawLabelColor, 2);
    panels.emplace_back("Field Edge and Yaw", angle_image);
}

// Reproject the fitted rectangle back onto the depth image. If the corners do not land on
// the field edges, the plane fit or the flattening transform is wrong.
void add_reprojection_panel(const FieldFilterDebugFrame &frame, DebugPanels &panels) {
    cv::Mat reprojected = colorize_depth(frame.masked_depth_image);
    double fx = frame.intrinsics.at<double>(0, 0);
    double fy = frame.intrinsics.at<double>(1, 1);
    double cx = frame.intrinsics.at<double>(0, 2);
    double cy = frame.intrinsics.at<double>(1, 2);

    std::vector<cv::Point> projected;
    projected.reserve(frame.rectangle_corners.size());
    for (const Eigen::Vector2f &corner : frame.rectangle_corners) {
        Eigen::Vector4d corner_flat(corner.x(), corner.y(), 0.0, 1.0);
        Eigen::Vector4d corner_camera = frame.plane_transform * corner_flat;
        double depth = corner_camera[2];
        if (depth <= 0.0) {
            continue;
        }
        projected.emplace_back(static_cast<int>(corner_camera[0] / depth * fx + cx),
                               static_cast<int>(corner_camera[1] / depth * fy + cy));
    }

    if (projected.size() == frame.rectangle_corners.size()) {
        for (size_t i = 0; i < projected.size(); ++i) {
            cv::line(reprojected, projected[i], projected[(i + 1) % projected.size()], kLabelColor,
                     2);
            cv::circle(reprojected, projected[i], 6, kCornerColor, -1);
        }
    }
    panels.emplace_back("Field Reprojected", reprojected);
}

// Lay the panels out in a grid, letterboxing each one so mixed aspect ratios keep their shape.
cv::Mat tile_panels(const DebugPanels &panels) {
    int rows = std::max(1, static_cast<int>(std::floor(std::sqrt(panels.size()))));
    int columns = (static_cast<int>(panels.size()) + rows - 1) / rows;
    cv::Mat mosaic(rows * kCellHeight, columns * kCellWidth, CV_8UC3, cv::Scalar(0, 0, 0));

    for (size_t i = 0; i < panels.size(); ++i) {
        const cv::Mat &panel = panels[i].second;
        double scale = std::min(static_cast<double>(kCellWidth) / panel.cols,
                                static_cast<double>(kCellHeight) / panel.rows);
        cv::Size scaled_size(std::clamp(static_cast<int>(panel.cols * scale), 1, kCellWidth),
                             std::clamp(static_cast<int>(panel.rows * scale), 1, kCellHeight));
        cv::Mat scaled;
        cv::resize(panel, scaled, scaled_size, 0, 0, cv::INTER_AREA);

        int row = static_cast<int>(i) / columns;
        int column = static_cast<int>(i) % columns;
        cv::Rect cell(column * kCellWidth, row * kCellHeight, kCellWidth, kCellHeight);
        cv::Rect placement(cell.x + (kCellWidth - scaled.cols) / 2,
                           cell.y + (kCellHeight - scaled.rows) / 2, scaled.cols, scaled.rows);
        scaled.copyTo(mosaic(placement));

        cv::putText(mosaic, panels[i].first, cv::Point(placement.x + 10, placement.y + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, kLabelColor, 2);
        cv::rectangle(mosaic, cell, kCellBorderColor, 1);
    }
    return mosaic;
}

}  // namespace

void visualize_debug_mosaic(const FieldFilterDebugFrame &frame) {
    DebugPanels panels;
    add_mask_panels(frame, panels);
    add_point_cloud_panels(frame, panels);
    add_rectangle_panels(frame, panels);
    add_reprojection_panel(frame, panels);

    cv::imshow("Field Filter Debug", tile_panels(panels));
    cv::waitKey(-1);
    cv::destroyAllWindows();
}

}  // namespace auto_battlebot
