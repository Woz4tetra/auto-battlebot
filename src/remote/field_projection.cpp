#include "remote/field_projection.hpp"

#include <array>
#include <cmath>
#include <optional>

namespace auto_battlebot::remote {

namespace {

/** Closer than this to the camera plane, a point projects to a meaningless far-away pixel. */
constexpr double kMinDepthM = 0.05;

std::optional<ImagePoint> project(const FieldDescription &field, const CameraInfo &camera_info,
                                  double x, double y) {
    const auto &tf = field.tf_camera_from_fieldcenter.tf;
    const double cx = tf(0, 0) * x + tf(0, 1) * y + tf(0, 3);
    const double cy = tf(1, 0) * x + tf(1, 1) * y + tf(1, 3);
    const double cz = tf(2, 0) * x + tf(2, 1) * y + tf(2, 3);
    if (cz < kMinDepthM) return std::nullopt;
    const cv::Mat &k = camera_info.intrinsics;
    const double u = k.at<double>(0, 0) * (cx / cz) + k.at<double>(0, 2);
    const double v = k.at<double>(1, 1) * (cy / cz) + k.at<double>(1, 2);
    if (!std::isfinite(u) || !std::isfinite(v)) return std::nullopt;
    return ImagePoint{.u = u / camera_info.width, .v = v / camera_info.height};
}

}  // namespace

std::vector<std::vector<ImagePoint>> project_field_outline(const FieldDescription &field,
                                                           const CameraInfo &camera_info,
                                                           int samples_per_side) {
    std::vector<std::vector<ImagePoint>> out;
    const auto &tf = field.tf_camera_from_fieldcenter.tf;
    if (tf.rows() < 3 || tf.cols() < 4) return out;
    if (camera_info.intrinsics.rows != 3 || camera_info.intrinsics.cols != 3) return out;
    if (camera_info.intrinsics.type() != CV_64F) return out;
    if (camera_info.width <= 0 || camera_info.height <= 0) return out;
    const double hx = field.size.size.x / 2.0;
    const double hy = field.size.size.y / 2.0;
    if (hx <= 0.0 || hy <= 0.0) return out;
    if (samples_per_side < 1) samples_per_side = 1;

    const std::array<std::array<double, 2>, 4> corners = {
        {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}}};
    std::vector<ImagePoint> run;
    auto flush = [&] {
        if (run.size() >= 2) out.push_back(std::move(run));
        run.clear();
    };
    // Walk the closed loop once, ending back on the first corner.
    for (size_t side = 0; side < corners.size(); ++side) {
        const auto &a = corners[side];
        const auto &b = corners[(side + 1) % corners.size()];
        const int last = side + 1 == corners.size() ? samples_per_side : samples_per_side - 1;
        for (int i = 0; i <= last; ++i) {
            const double t = static_cast<double>(i) / samples_per_side;
            if (auto p = project(field, camera_info, a[0] + t * (b[0] - a[0]),
                                 a[1] + t * (b[1] - a[1]))) {
                run.push_back(*p);
            } else {
                flush();
            }
        }
    }
    flush();
    // A loop cut behind the camera starts and ends mid-run; join the two ends into one line.
    if (out.size() >= 2 && out.front().front().u == out.back().back().u &&
        out.front().front().v == out.back().back().v) {
        auto &tail = out.back();
        tail.insert(tail.end(), out.front().begin() + 1, out.front().end());
        out.erase(out.begin());
    }
    return out;
}

}  // namespace auto_battlebot::remote
