#include "keypoint_model/apriltag_keypoint_model.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "field_filter/fiducial_field_filter.hpp"
#include "foxglove_adapters/common.hpp"
#include "foxglove_adapters/json_schemas.hpp"

namespace auto_battlebot {
namespace {
constexpr const char *kTopic = "/apriltag/robot_tags";
constexpr double kDegrees = M_PI / 180.0;
/** Two floor candidates closer than this are the same direction. */
constexpr double kSameDirectionRad = 5.0 * kDegrees;
/** Candidates farther than this from the winner are a rival cluster. */
constexpr double kRivalRad = 10.0 * kDegrees;
/** Below this difference in fit to the floor the two IPPE solutions are indistinguishable. */
constexpr double kAmbiguousRad = 2.0 * kDegrees;
/** How decisively the overhead-camera prior must separate the two solutions before it decides. */
constexpr double kOverheadPriorMarginRad = 15.0 * kDegrees;
constexpr size_t kMinFramesToLock = 30;
constexpr size_t kMaxFramesHeld = 300;
constexpr size_t kLockAttemptEvery = 10;
constexpr double kLockSupport = 0.8;
constexpr double kMaxRivalSupport = 0.5;
constexpr double kRefineGain = 0.01;
constexpr int kMrStabsMk2ClassId = 0;

double angle_between(const cv::Vec3d &a, const cv::Vec3d &b) {
    const double denominator = cv::norm(a) * cv::norm(b);
    if (denominator <= 0.0) return M_PI;
    return std::acos(std::clamp(a.dot(b) / denominator, -1.0, 1.0));
}

cv::Matx33d rotation_from_rvec(const cv::Vec3d &rvec) {
    cv::Matx33d rotation;
    cv::Rodrigues(rvec, rotation);
    return rotation;
}

cv::Matx33d to_matx(const std::array<double, 9> &values) {
    return cv::Matx33d(values[0], values[1], values[2], values[3], values[4], values[5], values[6],
                       values[7], values[8]);
}

/** Robot body pose in the camera frame, from a tag pose and the tag's mounting. */
void body_in_camera(const TagPoseSolution &solution, const TagMount &mount,
                    cv::Matx33d &rotation_camera_body, cv::Vec3d &translation_camera_body) {
    const cv::Matx33d rotation_camera_tag = rotation_from_rvec(solution.rvec);
    rotation_camera_body = rotation_camera_tag * mount.rotation_body_tag.t();
    translation_camera_body = solution.tvec - rotation_camera_body * mount.translation_body_tag;
}

void append_number(std::string &json, double value) {
    if (!std::isfinite(value)) {
        json += "null";
        return;
    }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    json += buffer;
}

void append_vec3(std::string &json, const cv::Vec3d &value) {
    json += '[';
    for (int i = 0; i < 3; ++i) {
        if (i > 0) json += ',';
        append_number(json, value[i]);
    }
    json += ']';
}
}  // namespace

std::vector<TagPoseSolution> solve_tag_pose(const std::array<cv::Point2f, 4> &corners,
                                            double tag_size_m, const cv::Mat &intrinsics,
                                            const cv::Mat &distortion) {
    const float half = static_cast<float>(tag_size_m / 2.0);
    // The order SOLVEPNP_IPPE_SQUARE requires, which is also aruco's corner order.
    const std::vector<cv::Point3f> object_points = {
        {-half, half, 0.0f}, {half, half, 0.0f}, {half, -half, 0.0f}, {-half, -half, 0.0f}};
    const std::vector<cv::Point2f> image_points(corners.begin(), corners.end());

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::Mat errors;
    std::vector<TagPoseSolution> solutions;
    try {
        cv::solvePnPGeneric(object_points, image_points, intrinsics, distortion, rvecs, tvecs,
                            false, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(), errors);
    } catch (const cv::Exception &error) {
        spdlog::warn("AprilTagKeypointModel: solvePnPGeneric failed: {}", error.what());
        return solutions;
    }
    for (size_t i = 0; i < rvecs.size(); ++i) {
        TagPoseSolution solution;
        rvecs[i].convertTo(rvecs[i], CV_64F);
        tvecs[i].convertTo(tvecs[i], CV_64F);
        solution.rvec = cv::Vec3d(rvecs[i].ptr<double>());
        solution.tvec = cv::Vec3d(tvecs[i].ptr<double>());
        if (static_cast<int>(i) < errors.rows * errors.cols) {
            cv::Mat errors64;
            errors.convertTo(errors64, CV_64F);
            solution.reprojection_error_px = errors64.at<double>(static_cast<int>(i));
        }
        solutions.push_back(solution);
    }
    std::sort(solutions.begin(), solutions.end(), [](const auto &a, const auto &b) {
        return a.reprojection_error_px < b.reprojection_error_px;
    });
    return solutions;
}

TagMount make_tag_mount(const AprilTagMountConfiguration &config, double rest_pitch_rad) {
    TagMount mount;
    mount.id = config.id;
    mount.rotation_body_tag = to_matx(config.rotation);
    mount.translation_body_tag =
        cv::Vec3d(config.translation_m[0], config.translation_m[1], config.translation_m[2]);
    // Body z component of the tag's z axis, the printed face normal.
    mount.faces_up = mount.rotation_body_tag(2, 2) > 0.0;
    // Positive pitch about body y is nose down in FLU, which tips floor up toward body -x.
    mount.floor_up_body = mount.faces_up
                              ? cv::Vec3d(-std::sin(rest_pitch_rad), 0.0, std::cos(rest_pitch_rad))
                              : cv::Vec3d(0.0, 0.0, -1.0);
    return mount;
}

cv::Vec3d floor_up_from_tag_pose(const TagPoseSolution &solution, const TagMount &mount) {
    const cv::Matx33d rotation_camera_body =
        rotation_from_rvec(solution.rvec) * mount.rotation_body_tag.t();
    return rotation_camera_body * mount.floor_up_body;
}

bool camera_above_floor(const TagPoseSolution &solution, const TagMount &mount) {
    cv::Matx33d rotation_camera_body;
    cv::Vec3d translation_camera_body;
    body_in_camera(solution, mount, rotation_camera_body, translation_camera_body);
    // The camera sits at the origin; its offset from the axle along floor up is its height.
    return floor_up_from_tag_pose(solution, mount).dot(-translation_camera_body) > 0.0;
}

size_t select_ippe_solution(const std::vector<TagPoseSolution> &solutions, const TagMount &mount,
                            const std::optional<cv::Vec3d> &floor_up_camera) {
    if (solutions.size() < 2) return 0;
    const bool above_0 = camera_above_floor(solutions[0], mount);
    const bool above_1 = camera_above_floor(solutions[1], mount);
    if (above_0 != above_1) return above_0 ? 0 : 1;
    const cv::Vec3d reference = floor_up_camera ? *floor_up_camera : cv::Vec3d(0.0, 0.0, -1.0);
    const double margin = floor_up_camera ? kAmbiguousRad : kOverheadPriorMarginRad;
    const double angle_0 = angle_between(floor_up_from_tag_pose(solutions[0], mount), reference);
    const double angle_1 = angle_between(floor_up_from_tag_pose(solutions[1], mount), reference);
    if (std::abs(angle_0 - angle_1) < margin) return 0;
    return angle_1 < angle_0 ? 1 : 0;
}

void FloorNormalEstimator::add_frame(const std::vector<cv::Vec3d> &candidates) {
    if (estimate_ || candidates.empty()) return;
    frames_.push_back(candidates);
    if (frames_.size() > kMaxFramesHeld) frames_.erase(frames_.begin());
    if (frames_.size() >= kMinFramesToLock && frames_.size() % kLockAttemptEvery == 0) try_lock();
}

void FloorNormalEstimator::try_lock() {
    const auto support = [this](const cv::Vec3d &direction, double radius) {
        size_t count = 0;
        for (const auto &frame : frames_) {
            for (const auto &candidate : frame) {
                if (angle_between(candidate, direction) < radius) {
                    ++count;
                    break;
                }
            }
        }
        return count;
    };

    size_t best_support = 0;
    cv::Vec3d best;
    for (const auto &frame : frames_) {
        for (const auto &candidate : frame) {
            const size_t count = support(candidate, kSameDirectionRad);
            if (count > best_support) {
                best_support = count;
                best = candidate;
            }
        }
    }
    const double frame_count = static_cast<double>(frames_.size());
    if (static_cast<double>(best_support) < kLockSupport * frame_count) return;

    size_t rival_support = 0;
    for (const auto &frame : frames_) {
        for (const auto &candidate : frame) {
            if (angle_between(candidate, best) <= kRivalRad) continue;
            rival_support = std::max(rival_support, support(candidate, kSameDirectionRad));
        }
    }
    // A robot holding still repeats both solutions every frame, so both clusters are full and
    // nothing says which is the floor. Wait for it to move.
    if (static_cast<double>(rival_support) > kMaxRivalSupport * frame_count) return;

    cv::Vec3d sum(0.0, 0.0, 0.0);
    for (const auto &frame : frames_) {
        for (const auto &candidate : frame) {
            if (angle_between(candidate, best) < kSameDirectionRad) sum += cv::normalize(candidate);
        }
    }
    estimate_ = cv::normalize(sum);
    frames_.clear();
    spdlog::info("AprilTagKeypointModel: floor normal locked at ({:.3f}, {:.3f}, {:.3f}) in camera",
                 (*estimate_)[0], (*estimate_)[1], (*estimate_)[2]);
}

void FloorNormalEstimator::refine(const cv::Vec3d &chosen) {
    if (!estimate_ || angle_between(chosen, *estimate_) >= kSameDirectionRad) return;
    estimate_ =
        cv::normalize((1.0 - kRefineGain) * *estimate_ + kRefineGain * cv::normalize(chosen));
}

AprilTagKeypointModel::AprilTagKeypointModel(const AprilTagKeypointModelConfiguration &config,
                                             std::shared_ptr<VizSink> sink,
                                             std::shared_ptr<McapRecorder> mcap_recorder)
    : config_(config),
      detector_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),
                small_apriltag_detector_parameters()),
      channel_(kTopic, "json",
               VizSchema::jsonschema(foxglove_adapters::kAprilTagRobotTagsSchemaName,
                                     foxglove_adapters::kAprilTagRobotTagsSchema),
               false, std::move(sink), std::move(mcap_recorder)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("apriltag_keypoint_model")) {
    for (const auto &tag : config_.tags) {
        mounts_.push_back(make_tag_mount(tag, config_.rest_pitch_rad));
    }
}

const TagMount *AprilTagKeypointModel::mount_for(int id) const {
    if (std::find(config_.robot_tag_ids.begin(), config_.robot_tag_ids.end(), id) ==
        config_.robot_tag_ids.end()) {
        return nullptr;
    }
    for (const auto &mount : mounts_) {
        if (mount.id == id) return &mount;
    }
    return nullptr;
}

std::vector<AprilTagKeypointModel::Detection> AprilTagKeypointModel::detect(
    const cv::Mat &gray, const cv::Rect &region) const {
    const cv::Mat searched = auto_gamma(gray(region));
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detector_.detectMarkers(searched, corners, ids);

    std::vector<Detection> detections;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (!mount_for(ids[i]) || corners[i].size() != 4) continue;
        Detection detection;
        detection.id = ids[i];
        for (size_t k = 0; k < 4; ++k) {
            detection.corners[k] = corners[i][k] + cv::Point2f(static_cast<float>(region.x),
                                                               static_cast<float>(region.y));
        }
        detections.push_back(detection);
    }
    return detections;
}

ModelResultStamped AprilTagKeypointModel::update(RgbImage image, const CameraInfo &camera_info) {
    FunctionTimer timer(diagnostics_logger_, "update");
    ModelResultStamped result;
    result.header = image.header;
    last_detections_ = DetectionsStamped{};
    last_detections_.header = image.header;
    last_detections_.image_width = image.image.cols;
    last_detections_.image_height = image.image.rows;
    if (image.image.empty()) return result;

    cv::Mat intrinsics;
    if (!camera_info.intrinsics.empty()) camera_info.intrinsics.convertTo(intrinsics, CV_64F);
    if (intrinsics.rows != 3 || intrinsics.cols != 3 || intrinsics.at<double>(0, 0) <= 0.0) {
        diagnostics_logger_->error("camera_info", "no usable intrinsics");
        return result;
    }
    cv::Mat distortion =
        camera_info.distortion.empty() ? cv::Mat::zeros(1, 5, CV_64F) : camera_info.distortion;

    cv::Mat gray;
    if (image.image.channels() == 3) {
        cv::cvtColor(image.image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.image;
    }
    const cv::Rect full(0, 0, gray.cols, gray.rows);

    std::optional<cv::Rect> searched_roi;
    std::vector<Detection> detections;
    if (roi_) {
        const cv::Rect roi = *roi_ & full;
        if (roi.area() > 0) {
            detections = detect(gray, roi);
            searched_roi = roi;
        }
    }
    if (detections.empty()) {
        detections = detect(gray, full);
        searched_roi.reset();
    }

    for (auto &detection : detections) {
        detection.solutions =
            solve_tag_pose(detection.corners, config_.tag_size_m, intrinsics, distortion);
    }

    // Next frame's window: every robot tag found, grown by the margin.
    if (detections.empty()) {
        roi_.reset();
    } else {
        cv::Rect bounds;
        for (const auto &detection : detections) {
            const std::vector<cv::Point2f> points(detection.corners.begin(),
                                                  detection.corners.end());
            bounds |= cv::boundingRect(points);
        }
        const int margin = config_.roi_margin_px;
        roi_ = cv::Rect(bounds.x - margin, bounds.y - margin, bounds.width + 2 * margin,
                        bounds.height + 2 * margin) &
               full;
    }

    // Recorded before anything is filtered: the corners are the measurement.
    const auto image_stamp_ns = static_cast<uint64_t>(std::llround(image.header.stamp * 1e9));
    last_tags_json_ = to_json(image_stamp_ns, image, camera_info, searched_roi, detections);
    channel_.log(last_tags_json_, image_stamp_ns);

    // Keypoints from the best-fitting tag. Both tags in one frame means the robot is on its side;
    // the better fit is the better guess.
    const Detection *best = nullptr;
    for (const auto &detection : detections) {
        if (detection.solutions.empty()) continue;
        if (!best || detection.solutions.front().reprojection_error_px <
                         best->solutions.front().reprojection_error_px) {
            best = &detection;
        }
    }
    if (best) {
        emit_keypoints(*best, *mount_for(best->id), intrinsics, distortion, result);
    }

    DiagnosticsData data;
    data["detections"] = static_cast<int>(detections.size());
    data["roi_search"] = static_cast<int>(searched_roi.has_value());
    data["floor_locked"] = static_cast<int>(floor_estimator_.estimate().has_value());
    diagnostics_logger_->debug("detect", data);
    return result;
}

void AprilTagKeypointModel::emit_keypoints(const Detection &detection, const TagMount &mount,
                                           const cv::Mat &intrinsics, const cv::Mat &distortion,
                                           ModelResultStamped &result) {
    std::vector<cv::Vec3d> candidates;
    for (const auto &solution : detection.solutions) {
        if (camera_above_floor(solution, mount)) {
            candidates.push_back(floor_up_from_tag_pose(solution, mount));
        }
    }
    floor_estimator_.add_frame(candidates);

    const size_t chosen_index =
        select_ippe_solution(detection.solutions, mount, floor_estimator_.estimate());
    const TagPoseSolution &chosen = detection.solutions[chosen_index];
    if (chosen.reprojection_error_px > config_.max_reprojection_error_px) {
        diagnostics_logger_->debug(
            "rejected",
            {{"id", detection.id}, {"reprojection_error_px", chosen.reprojection_error_px}});
        return;
    }
    floor_estimator_.refine(floor_up_from_tag_pose(chosen, mount));

    cv::Matx33d rotation_camera_body;
    cv::Vec3d translation_camera_body;
    body_in_camera(chosen, mount, rotation_camera_body, translation_camera_body);

    const std::vector<cv::Point3d> body_points = {
        {config_.front_keypoint_m[0], config_.front_keypoint_m[1], config_.front_keypoint_m[2]},
        {config_.back_keypoint_m[0], config_.back_keypoint_m[1], config_.back_keypoint_m[2]}};
    cv::Vec3d rvec_camera_body;
    cv::Rodrigues(rotation_camera_body, rvec_camera_body);
    std::vector<cv::Point2d> projected;
    cv::projectPoints(body_points, rvec_camera_body, translation_camera_body, intrinsics,
                      distortion, projected);

    // The wheels hold the axle one radius off the floor either way up.
    const int detection_index = static_cast<int>(result.boxes.size());
    const std::array<KeypointLabel, 2> labels = {KeypointLabel::MR_STABS_MK2_FRONT,
                                                 KeypointLabel::MR_STABS_MK2_BACK};

    Detection2D detection_2d;
    detection_2d.label = Label::MR_STABS_MK2;
    detection_2d.class_id = kMrStabsMk2ClassId;
    detection_2d.confidence = 1.0;
    BoundingBox box{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};
    for (const auto &corner : detection.corners) {
        box.x1 = std::min(box.x1, static_cast<double>(corner.x));
        box.y1 = std::min(box.y1, static_cast<double>(corner.y));
        box.x2 = std::max(box.x2, static_cast<double>(corner.x));
        box.y2 = std::max(box.y2, static_cast<double>(corner.y));
    }
    for (size_t i = 0; i < projected.size(); ++i) {
        Keypoint keypoint;
        keypoint.label = Label::MR_STABS_MK2;
        keypoint.keypoint_label = labels[i];
        keypoint.x = projected[i].x;
        keypoint.y = projected[i].y;
        keypoint.confidence = 1.0;
        keypoint.detection_index = detection_index;
        const cv::Vec3d body_point(body_points[i].x, body_points[i].y, body_points[i].z);
        keypoint.height_above_plane = config_.axle_height_m + mount.floor_up_body.dot(body_point);
        result.keypoints.push_back(keypoint);
        detection_2d.keypoints.push_back({keypoint.x, keypoint.y, 1.0});
        box.x1 = std::min(box.x1, keypoint.x);
        box.y1 = std::min(box.y1, keypoint.y);
        box.x2 = std::max(box.x2, keypoint.x);
        box.y2 = std::max(box.y2, keypoint.y);
    }
    result.boxes.push_back(box);
    detection_2d.x1 = box.x1;
    detection_2d.y1 = box.y1;
    detection_2d.x2 = box.x2;
    detection_2d.y2 = box.y2;
    last_detections_.detections.push_back(detection_2d);
}

std::string AprilTagKeypointModel::to_json(uint64_t image_stamp_ns, const RgbImage &image,
                                           const CameraInfo &camera_info,
                                           const std::optional<cv::Rect> &roi,
                                           const std::vector<Detection> &detections) const {
    cv::Mat k;
    camera_info.intrinsics.convertTo(k, CV_64F);
    std::string json;
    json.reserve(512 + detections.size() * 512);
    json += "{\"image_stamp_ns\":";
    json += std::to_string(image_stamp_ns);
    json += ",\"frame_id\":\"";
    json += foxglove_adapters::frame_id_string(image.header.frame_id);
    json += "\",\"camera\":{\"fx\":";
    append_number(json, k.at<double>(0, 0));
    json += ",\"fy\":";
    append_number(json, k.at<double>(1, 1));
    json += ",\"cx\":";
    append_number(json, k.at<double>(0, 2));
    json += ",\"cy\":";
    append_number(json, k.at<double>(1, 2));
    json += ",\"width\":";
    json += std::to_string(camera_info.width > 0 ? camera_info.width : image.image.cols);
    json += ",\"height\":";
    json += std::to_string(camera_info.height > 0 ? camera_info.height : image.image.rows);
    json += "},\"tag_size_m\":";
    append_number(json, config_.tag_size_m);
    json += ",\"roi\":";
    if (roi) {
        json += '[' + std::to_string(roi->x) + ',' + std::to_string(roi->y) + ',' +
                std::to_string(roi->width) + ',' + std::to_string(roi->height) + ']';
    } else {
        json += "null";
    }
    json += ",\"detections\":[";
    for (size_t d = 0; d < detections.size(); ++d) {
        const Detection &detection = detections[d];
        if (d > 0) json += ',';
        json += "{\"id\":";
        json += std::to_string(detection.id);
        json += ",\"corners\":[";
        for (size_t c = 0; c < 4; ++c) {
            if (c > 0) json += ',';
            json += '[';
            append_number(json, detection.corners[c].x);
            json += ',';
            append_number(json, detection.corners[c].y);
            json += ']';
        }
        // OpenCV's aruco detector exposes no decision margin.
        json += "],\"decision_margin\":null,\"solutions\":[";
        for (size_t s = 0; s < detection.solutions.size(); ++s) {
            const TagPoseSolution &solution = detection.solutions[s];
            if (s > 0) json += ',';
            json += "{\"rvec\":";
            append_vec3(json, solution.rvec);
            json += ",\"tvec\":";
            append_vec3(json, solution.tvec);
            json += ",\"reprojection_error_px\":";
            append_number(json, solution.reprojection_error_px);
            json += '}';
        }
        json += "]}";
    }
    json += "]}";
    return json;
}

}  // namespace auto_battlebot
