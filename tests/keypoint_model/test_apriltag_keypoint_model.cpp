#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <random>
#include <vector>

#include "keypoint_model/apriltag_keypoint_model.hpp"

namespace auto_battlebot {
namespace {

constexpr double kTagSize = 0.064;
constexpr double kAxleHeight = 0.025;
constexpr double kRestPitch = 0.198025;
constexpr int kWidth = 1920;
constexpr int kHeight = 1200;

// Mr Stabs Mk2 mountings from simulation/assets/robots/mr_stabs_mk2/mass_properties.toml.
AprilTagMountConfiguration top_tag() {
    AprilTagMountConfiguration tag;
    tag.id = 76;
    tag.translation_m = {0.035553, -1.1e-05, 0.014164};
    tag.rotation = {0.0, -0.986029, 0.166572, 1.0, -2e-06, -1.1e-05, 1.1e-05, 0.166572, 0.986029};
    return tag;
}

AprilTagMountConfiguration bottom_tag() {
    AprilTagMountConfiguration tag;
    tag.id = 41;
    tag.translation_m = {0.035391, 1.9e-05, -0.012076};
    tag.rotation = {0.0,      -0.983904, 0.178696,  -1.0,     -3e-06,
                    -1.6e-05, 1.6e-05,   -0.178696, -0.983904};
    return tag;
}

AprilTagKeypointModelConfiguration model_config() {
    AprilTagKeypointModelConfiguration config;
    config.tag_size_m = kTagSize;
    config.robot_tag_ids = {41, 76};
    config.roi_margin_px = 80;
    config.max_reprojection_error_px = 2.0;
    config.front_keypoint_m = {0.05, 0.0, 0.0};
    config.back_keypoint_m = {-0.05, 0.0, 0.0};
    config.axle_height_m = kAxleHeight;
    config.rest_pitch_rad = kRestPitch;
    config.tags = {bottom_tag(), top_tag()};
    return config;
}

cv::Matx33d rot_x(double a) {
    return {1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a)};
}
cv::Matx33d rot_y(double a) {
    return {std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a)};
}
cv::Matx33d rot_z(double a) {
    return {std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1};
}

struct Pose {
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d translation{0, 0, 0};
    Pose operator*(const Pose &other) const {
        return {rotation * other.rotation, rotation * other.translation + translation};
    }
    cv::Vec3d apply(const cv::Vec3d &point) const { return rotation * point + translation; }
};

/**
 * A camera `height` above the floor, pitched `tilt` off straight down about its x axis. The floor
 * frame is FLU with z up; with no tilt, floor x is image right and floor y is image up.
 */
Pose camera_from_floor(double height, double tilt) {
    Pose overhead{cv::Matx33d(1, 0, 0, 0, -1, 0, 0, 0, -1), cv::Vec3d(0, 0, height)};
    return Pose{rot_x(tilt), cv::Vec3d(0, 0, 0)} * overhead;
}

/** Robot resting on the floor at (x, y) with heading yaw, upright or on its back. */
Pose floor_from_body(double x, double y, double yaw, bool upside_down) {
    if (upside_down) {
        return {rot_z(yaw) * rot_x(M_PI), cv::Vec3d(x, y, kAxleHeight)};
    }
    return {rot_z(yaw) * rot_y(kRestPitch), cv::Vec3d(x, y, kAxleHeight)};
}

Pose body_from_tag(const AprilTagMountConfiguration &tag) {
    const auto &r = tag.rotation;
    return {cv::Matx33d(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]),
            cv::Vec3d(tag.translation_m[0], tag.translation_m[1], tag.translation_m[2])};
}

cv::Mat intrinsics() {
    return (cv::Mat_<double>(3, 3) << 665.0, 0.0, 960.0, 0.0, 716.0, 600.0, 0.0, 0.0, 1.0);
}

CameraInfo camera_info() {
    CameraInfo info;
    info.width = kWidth;
    info.height = kHeight;
    info.intrinsics = intrinsics();
    info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    return info;
}

cv::Point2d project(const cv::Vec3d &point_camera) {
    const cv::Mat k = intrinsics();
    return {k.at<double>(0, 0) * point_camera[0] / point_camera[2] + k.at<double>(0, 2),
            k.at<double>(1, 1) * point_camera[1] / point_camera[2] + k.at<double>(1, 2)};
}

/** The tag's four corners in aruco order (top-left clockwise), projected. */
std::array<cv::Point2f, 4> tag_corners(const Pose &camera_from_tag) {
    const double h = kTagSize / 2.0;
    const std::array<cv::Vec3d, 4> object = {cv::Vec3d(-h, h, 0), cv::Vec3d(h, h, 0),
                                             cv::Vec3d(h, -h, 0), cv::Vec3d(-h, -h, 0)};
    std::array<cv::Point2f, 4> corners;
    for (size_t i = 0; i < 4; ++i) {
        const cv::Point2d p = project(camera_from_tag.apply(object[i]));
        corners[i] = cv::Point2f(static_cast<float>(p.x), static_cast<float>(p.y));
    }
    return corners;
}

/** A grey frame with the 36h11 marker warped to where the pose puts it. */
cv::Mat render_tag(int id, const Pose &camera_from_tag) {
    constexpr int kMarkerPx = 400;
    constexpr int kPad = 100;
    cv::Mat marker;
    cv::aruco::generateImageMarker(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, kMarkerPx, marker,
        1);
    cv::Mat source(kMarkerPx + 2 * kPad, kMarkerPx + 2 * kPad, CV_8UC1, cv::Scalar(255));
    marker.copyTo(source(cv::Rect(kPad, kPad, kMarkerPx, kMarkerPx)));

    const std::vector<cv::Point2f> source_corners = {{kPad, kPad},
                                                     {kPad + kMarkerPx, kPad},
                                                     {kPad + kMarkerPx, kPad + kMarkerPx},
                                                     {kPad, kPad + kMarkerPx}};
    const auto corners = tag_corners(camera_from_tag);
    const cv::Mat homography = cv::getPerspectiveTransform(
        source_corners, std::vector<cv::Point2f>(corners.begin(), corners.end()));

    cv::Mat frame(kHeight, kWidth, CV_8UC1, cv::Scalar(150));
    cv::warpPerspective(source, frame, homography, frame.size(), cv::INTER_LINEAR,
                        cv::BORDER_TRANSPARENT);
    cv::Mat bgr;
    cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

RgbImage make_image(const cv::Mat &frame, double stamp) {
    RgbImage image;
    image.header.stamp = stamp;
    image.header.frame_id = FrameId::CAMERA;
    image.image = frame;
    return image;
}

double rotation_angle_between(const cv::Matx33d &a, const cv::Matx33d &b) {
    const cv::Matx33d delta = a.t() * b;
    const double trace = delta(0, 0) + delta(1, 1) + delta(2, 2);
    return std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
}

cv::Matx33d rotation_of(const TagPoseSolution &solution) {
    cv::Matx33d rotation;
    cv::Rodrigues(solution.rvec, rotation);
    return rotation;
}

TEST(AprilTagKeypointModelTest, DetectsRenderedTagAndRecoversPose) {
    const Pose camera_floor = camera_from_floor(0.75, 0.0);
    const Pose floor_body = floor_from_body(0.10, -0.05, 0.6, false);
    const Pose camera_body = camera_floor * floor_body;
    const Pose camera_tag = camera_body * body_from_tag(top_tag());
    const cv::Mat frame = render_tag(76, camera_tag);

    AprilTagKeypointModel model(model_config(), nullptr, nullptr);
    const ModelResultStamped result = model.update(make_image(frame, 1788011445.25), camera_info());

    // The raw measurement.
    const std::string &json = model.last_tags_json();
    EXPECT_NE(json.find("\"id\":76"), std::string::npos) << json;
    EXPECT_NE(json.find("\"roi\":null"), std::string::npos);
    EXPECT_NE(json.find("\"decision_margin\":null"), std::string::npos);
    EXPECT_NE(json.find("\"frame_id\":\"camera\""), std::string::npos);
    EXPECT_NE(json.find("\"tag_size_m\":0.064"), std::string::npos);

    // Detection corners against the rendered ones, through a fresh solve.
    const auto truth_corners = tag_corners(camera_tag);
    const auto solutions =
        solve_tag_pose(truth_corners, kTagSize, intrinsics(), cv::Mat::zeros(1, 5, CV_64F));
    ASSERT_EQ(solutions.size(), 2u);
    EXPECT_LE(solutions[0].reprojection_error_px, solutions[1].reprojection_error_px);

    // Keypoints: the front and back body points projected through the true pose.
    ASSERT_EQ(result.keypoints.size(), 2u);
    const cv::Point2d front_truth = project(camera_body.apply({0.05, 0.0, 0.0}));
    const cv::Point2d back_truth = project(camera_body.apply({-0.05, 0.0, 0.0}));
    EXPECT_EQ(result.keypoints[0].label, Label::MR_STABS_MK2);
    EXPECT_EQ(result.keypoints[0].keypoint_label, KeypointLabel::MR_STABS_MK2_FRONT);
    EXPECT_EQ(result.keypoints[1].keypoint_label, KeypointLabel::MR_STABS_MK2_BACK);
    EXPECT_NEAR(result.keypoints[0].x, front_truth.x, 1.5);
    EXPECT_NEAR(result.keypoints[0].y, front_truth.y, 1.5);
    EXPECT_NEAR(result.keypoints[1].x, back_truth.x, 1.5);
    EXPECT_NEAR(result.keypoints[1].y, back_truth.y, 1.5);
    // Nose down: the front point sits lower than the back one.
    EXPECT_NEAR(result.keypoints[0].height_above_plane, kAxleHeight - 0.05 * std::sin(kRestPitch),
                1e-9);
    EXPECT_NEAR(result.keypoints[1].height_above_plane, kAxleHeight + 0.05 * std::sin(kRestPitch),
                1e-9);
    ASSERT_EQ(result.boxes.size(), 1u);
    EXPECT_EQ(result.keypoints[0].detection_index, 0);
    ASSERT_EQ(model.last_detections().detections.size(), 1u);
    EXPECT_EQ(model.last_detections().detections[0].label, Label::MR_STABS_MK2);

    // The next frame searches around this detection.
    ASSERT_TRUE(model.search_roi().has_value());
    const ModelResultStamped second = model.update(make_image(frame, 1788011445.27), camera_info());
    EXPECT_EQ(second.keypoints.size(), 2u);
    EXPECT_EQ(model.last_tags_json().find("\"roi\":null"), std::string::npos);
}

TEST(AprilTagKeypointModelTest, EmptyFrameIsRecordedAndClearsTheRoi) {
    AprilTagKeypointModel model(model_config(), nullptr, nullptr);
    const cv::Mat blank(kHeight, kWidth, CV_8UC3, cv::Scalar(150, 150, 150));
    const ModelResultStamped result = model.update(make_image(blank, 10.0), camera_info());
    EXPECT_TRUE(result.keypoints.empty());
    EXPECT_FALSE(model.search_roi().has_value());
    EXPECT_NE(model.last_tags_json().find("\"detections\":[]"), std::string::npos);
    EXPECT_NE(model.last_tags_json().find("\"image_stamp_ns\":10000000000"), std::string::npos);
}

TEST(AprilTagKeypointModelTest, BottomTagMeansUpsideDown) {
    const Pose camera_floor = camera_from_floor(0.75, 0.0);
    const Pose floor_body = floor_from_body(-0.05, 0.08, -1.1, true);
    const Pose camera_body = camera_floor * floor_body;
    const Pose camera_tag = camera_body * body_from_tag(bottom_tag());

    AprilTagKeypointModel model(model_config(), nullptr, nullptr);
    const ModelResultStamped result =
        model.update(make_image(render_tag(41, camera_tag), 1.0), camera_info());
    ASSERT_EQ(result.keypoints.size(), 2u);
    const cv::Point2d front_truth = project(camera_body.apply({0.05, 0.0, 0.0}));
    EXPECT_NEAR(result.keypoints[0].x, front_truth.x, 1.5);
    EXPECT_NEAR(result.keypoints[0].y, front_truth.y, 1.5);
    EXPECT_NEAR(result.keypoints[0].height_above_plane, kAxleHeight, 1e-9);
}

TEST(AprilTagKeypointModelTest, TagMountFacingComesFromRotation) {
    EXPECT_TRUE(make_tag_mount(top_tag(), kRestPitch).faces_up);
    EXPECT_FALSE(make_tag_mount(bottom_tag(), kRestPitch).faces_up);
}

TEST(AprilTagKeypointModelTest, KnownFloorPicksTheTrueIppeSolutionOnATiltedView) {
    // A tripod tilted 35 degrees off straight down, robots all over the box floor, corners with
    // 0.3 px of noise. The flip is sometimes the better fit; the floor must still pick the truth.
    const Pose camera_floor = camera_from_floor(0.75, 35.0 * M_PI / 180.0);
    const cv::Vec3d floor_up_camera = camera_floor.rotation * cv::Vec3d(0, 0, 1);
    const TagMount mount = make_tag_mount(top_tag(), kRestPitch);
    std::mt19937 rng(7);
    std::normal_distribution<float> noise(0.0f, 0.3f);

    int cases = 0;
    int flip_fits_better = 0;
    int reprojection_would_fail = 0;
    for (double x = -0.6; x <= 0.61; x += 0.3) {
        for (double y = -0.3; y <= 0.61; y += 0.3) {
            for (double yaw = 0.0; yaw < 2.0 * M_PI; yaw += M_PI / 6.0) {
                const Pose camera_tag =
                    camera_floor * floor_from_body(x, y, yaw, false) * body_from_tag(top_tag());
                auto corners = tag_corners(camera_tag);
                for (auto &corner : corners) corner += cv::Point2f(noise(rng), noise(rng));
                const auto solutions =
                    solve_tag_pose(corners, kTagSize, intrinsics(), cv::Mat::zeros(1, 5, CV_64F));
                ASSERT_EQ(solutions.size(), 2u);
                const size_t truth_index =
                    rotation_angle_between(rotation_of(solutions[0]), camera_tag.rotation) <=
                            rotation_angle_between(rotation_of(solutions[1]), camera_tag.rotation)
                        ? 0
                        : 1;
                const size_t chosen = select_ippe_solution(solutions, mount, floor_up_camera);
                const double chosen_error =
                    rotation_angle_between(rotation_of(solutions[chosen]), camera_tag.rotation);
                const double truth_error = rotation_angle_between(
                    rotation_of(solutions[truth_index]), camera_tag.rotation);
                // Either the true solution, or one indistinguishable from it.
                EXPECT_LT(chosen_error, truth_error + 2.0 * M_PI / 180.0)
                    << "x " << x << " y " << y << " yaw " << yaw;
                ++cases;
                if (truth_index != 0) ++flip_fits_better;
                if (truth_index != 0 && chosen == truth_index) ++reprojection_would_fail;
            }
        }
    }
    std::printf("[ IPPE     ] %d poses, flip had lower reprojection error in %d, floor fixed %d\n",
                cases, flip_fits_better, reprojection_would_fail);
}

TEST(AprilTagKeypointModelTest, FloorEstimatorLocksOnlyOnceTheRobotMoves) {
    const Pose camera_floor = camera_from_floor(0.75, 30.0 * M_PI / 180.0);
    const cv::Vec3d floor_up_camera = camera_floor.rotation * cv::Vec3d(0, 0, 1);
    const TagMount mount = make_tag_mount(top_tag(), kRestPitch);

    const auto candidates_for = [&](double x, double y, double yaw) {
        const Pose camera_tag =
            camera_floor * floor_from_body(x, y, yaw, false) * body_from_tag(top_tag());
        const auto solutions = solve_tag_pose(tag_corners(camera_tag), kTagSize, intrinsics(),
                                              cv::Mat::zeros(1, 5, CV_64F));
        std::vector<cv::Vec3d> candidates;
        for (const auto &solution : solutions) {
            if (camera_above_floor(solution, mount)) {
                candidates.push_back(floor_up_from_tag_pose(solution, mount));
            }
        }
        return candidates;
    };

    FloorNormalEstimator parked;
    for (int i = 0; i < 100; ++i) parked.add_frame(candidates_for(0.2, 0.1, 0.4));
    EXPECT_FALSE(parked.estimate().has_value());

    FloorNormalEstimator driving;
    for (int i = 0; i < 200 && !driving.estimate(); ++i) {
        const double t = i * 0.05;
        driving.add_frame(
            candidates_for(0.4 * std::cos(t), 0.2 + 0.3 * std::sin(1.3 * t), 2.1 * t));
    }
    ASSERT_TRUE(driving.estimate().has_value());
    const double error = std::acos(std::clamp(driving.estimate()->dot(floor_up_camera), -1.0, 1.0));
    EXPECT_LT(error, 1.0 * M_PI / 180.0);
}

TEST(AprilTagKeypointModelTest, DetectionCostOnAFullFrame) {
    const Pose camera_tag = camera_from_floor(0.75, 0.0) * floor_from_body(0.3, 0.2, 1.0, false) *
                            body_from_tag(top_tag());
    const cv::Mat frame = render_tag(76, camera_tag);
    constexpr int kRuns = 5;

    double full_ms = 0.0;
    double roi_ms = 0.0;
    for (int run = 0; run < kRuns; ++run) {
        AprilTagKeypointModel model(model_config(), nullptr, nullptr);
        auto start = std::chrono::steady_clock::now();
        model.update(make_image(frame, 1.0), camera_info());
        full_ms +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        ASSERT_TRUE(model.search_roi().has_value());
        start = std::chrono::steady_clock::now();
        const auto result = model.update(make_image(frame, 1.02), camera_info());
        roi_ms +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        EXPECT_EQ(result.keypoints.size(), 2u);
    }
    std::printf("[ TIMING   ] AprilTag update on 1920x1200: full frame %.2f ms, ROI %.2f ms\n",
                full_ms / kRuns, roi_ms / kRuns);
}

}  // namespace
}  // namespace auto_battlebot
