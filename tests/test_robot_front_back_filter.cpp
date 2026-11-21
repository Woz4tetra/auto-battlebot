#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core.hpp>

#include "diagnostics_logger/test_diagnostics_logger.hpp"
#include "plant/mrs_buff_mk3_params.hpp"
#include "robot_filter/robot_front_back_filter.hpp"

namespace auto_battlebot {
namespace {
constexpr double kOurFrontX = 300.0;
constexpr double kOurBackX = 340.0;
constexpr double kOurRowY = 220.0;

RobotDescriptionsStamped run_filter(RobotFrontBackFilter &filter, ModelResultStamped keypoints,
                                    const FieldDescription &field, const CameraInfo &camera_info,
                                    const CommandFeedback &command_feedback) {
    filter.predict(keypoints.header.stamp, command_feedback);
    filter.correct(std::move(keypoints), field, camera_info);
    return filter.state();
}

CameraInfo make_camera_info() {
    CameraInfo camera_info;
    camera_info.width = 640;
    camera_info.height = 480;
    camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    camera_info.intrinsics.at<double>(0, 0) = 500.0;
    camera_info.intrinsics.at<double>(1, 1) = 500.0;
    camera_info.intrinsics.at<double>(0, 2) = 320.0;
    camera_info.intrinsics.at<double>(1, 2) = 240.0;
    return camera_info;
}

FieldDescription make_field() {
    FieldDescription field;
    field.child_frame_id = FrameId::FIELD;
    field.size.size.x = 2.0;
    field.size.size.y = 2.0;
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 2) = -1.0;
    tf(2, 3) = 3.0;
    field.tf_camera_from_fieldcenter.tf = tf;
    return field;
}

ModelResultStamped make_opponent_keypoints(double stamp, double front_x, double back_x) {
    ModelResultStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;
    keypoints.header.stamp = stamp;

    Keypoint front;
    front.label = Label::OPPONENT;
    front.keypoint_label = KeypointLabel::OPPONENT_FRONT;
    front.x = front_x;
    front.y = 220.0;
    front.confidence = 0.9;
    front.detection_index = 1;
    keypoints.keypoints.push_back(front);

    Keypoint back = front;
    back.keypoint_label = KeypointLabel::OPPONENT_BACK;
    back.x = back_x;
    back.confidence = 0.85;
    keypoints.keypoints.push_back(back);
    return keypoints;
}

void append_keypoint_pair(ModelResultStamped &model_results, Label label,
                          KeypointLabel front_kp_label, KeypointLabel back_kp_label,
                          int detection_index, double front_x, double back_x, double y,
                          double confidence) {
    Keypoint front;
    front.label = label;
    front.keypoint_label = front_kp_label;
    front.x = front_x;
    front.y = y;
    front.confidence = confidence;
    front.detection_index = detection_index;
    model_results.keypoints.push_back(front);

    Keypoint back = front;
    back.keypoint_label = back_kp_label;
    back.x = back_x;
    back.confidence = confidence;
    model_results.keypoints.push_back(back);
}

ModelResultStamped make_empty_keypoints(double stamp) {
    ModelResultStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;
    keypoints.header.stamp = stamp;
    return keypoints;
}

ModelResultStamped make_our_keypoints(double stamp, double front_x, double back_x, double y) {
    ModelResultStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;
    keypoints.header.stamp = stamp;
    append_keypoint_pair(keypoints, Label::MRS_BUFF_MK3, KeypointLabel::MRS_BUFF_MK3_FRONT,
                         KeypointLabel::MRS_BUFF_MK3_BACK, 1, front_x, back_x, y, 0.9);
    return keypoints;
}

std::shared_ptr<KalmanMotionEstimatorConfiguration> plant_backed_estimator() {
    auto estimator = std::make_shared<KalmanMotionEstimatorConfiguration>();
    estimator->plant = mrs_buff_mk3_plant();
    return estimator;
}

RobotFrontBackFilterConfiguration make_our_robot_config() {
    RobotFrontBackFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT, KeypointLabel::MRS_BUFF_MK3_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK, KeypointLabel::MRS_BUFF_MK3_BACK};
    config.label_to_frame_ids = {{Label::MRS_BUFF_MK3, {FrameId::OUR_ROBOT_1}}};
    config.default_frame_id = FrameId::OUR_ROBOT_1;
    config.max_jump_distance = 1.0;
    config.max_consecutive_jump_rejects = 1;
    config.motion_estimator = plant_backed_estimator();
    return config;
}
}  // namespace

TEST(RobotFrontBackFilterTest, RejectsLargeJumpThenAcceptsAfterThreshold) {
    RobotFrontBackFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK};
    config.label_to_frame_ids = {{Label::OPPONENT, {FrameId::THEIR_ROBOT_1}}};
    config.default_frame_id = FrameId::THEIR_ROBOT_1;
    config.max_jump_distance = 0.2;
    config.max_consecutive_jump_rejects = 1;
    config.motion_estimator = plant_backed_estimator();

    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const CommandFeedback command_feedback;

    const auto first = run_filter(filter, make_opponent_keypoints(1.0, 300.0, 340.0), field,
                                  camera_info, command_feedback);
    ASSERT_EQ(first.descriptions.size(), 1u);
    const double first_x = first.descriptions[0].pose.position.x;

    const auto second = run_filter(filter, make_opponent_keypoints(1.1, 460.0, 500.0), field,
                                   camera_info, command_feedback);
    ASSERT_EQ(second.descriptions.size(), 1u);
    EXPECT_NEAR(second.descriptions[0].pose.position.x, first_x, 1e-6);

    const auto third = run_filter(filter, make_opponent_keypoints(1.2, 460.0, 500.0), field,
                                  camera_info, command_feedback);
    ASSERT_EQ(third.descriptions.size(), 1u);
    EXPECT_GT(std::abs(third.descriptions[0].pose.position.x - first_x), 0.5);
}

TEST(RobotFrontBackFilterTest, DropoutFrameOutputCarriesTheKeypointStamp) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, kOurFrontX, kOurBackX, kOurRowY), field, camera_info,
               command_feedback);
    const auto dropout =
        run_filter(filter, make_empty_keypoints(1.1), field, camera_info, command_feedback);

    EXPECT_DOUBLE_EQ(dropout.header.stamp, 1.1);
}

namespace {
const RobotDescription *find_frame_id(const RobotDescriptionsStamped &result, FrameId frame_id) {
    for (const auto &description : result.descriptions) {
        if (description.frame_id == frame_id) return &description;
    }
    return nullptr;
}
}  // namespace

RobotFrontBackFilterConfiguration make_kalman_opponent_config() {
    RobotFrontBackFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK};
    config.label_to_frame_ids = {{Label::OPPONENT, {FrameId::THEIR_ROBOT_1}}};
    config.default_frame_id = FrameId::THEIR_ROBOT_1;
    config.max_jump_distance = 1.0;
    auto kalman = std::make_shared<KalmanMotionEstimatorConfiguration>();
    kalman->max_coast_s = 1.0;
    kalman->plant = mrs_buff_mk3_plant();
    config.motion_estimator = kalman;
    return config;
}

constexpr double kOpponentPixelsPerSecond = 80.0;
constexpr double kFrameDt = 1.0 / 30.0;
constexpr double kLateMeasurementShiftPixels = 12.0;

ModelResultStamped make_moving_opponent_keypoints(double stamp, double extra_pixels = 0.0) {
    const double offset = kOpponentPixelsPerSecond * (stamp - 1.0) + extra_pixels;
    return make_opponent_keypoints(stamp, 280.0 + offset, 320.0 + offset);
}

double drive_opponent_to(RobotFrontBackFilter &filter, const CameraInfo &camera_info,
                         const FieldDescription &field, int frames) {
    double stamp = 1.0;
    for (int i = 0; i < frames; ++i) {
        stamp = 1.0 + i * kFrameDt;
        run_filter(filter, make_moving_opponent_keypoints(stamp), field, camera_info,
                   CommandFeedback{});
    }
    return stamp;
}

const RobotDescription *find_opponent(const RobotDescriptionsStamped &state) {
    for (const auto &description : state.descriptions) {
        if (description.frame_id == FrameId::THEIR_ROBOT_1) return &description;
    }
    return nullptr;
}

TEST(RobotFrontBackFilterTest, CorrectWithStampOlderThanPredict) {
    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    constexpr double kReadAfterCorrectS = 0.3;

    RobotFrontBackFilterConfiguration lockstep_config = make_kalman_opponent_config();
    RobotFrontBackFilter lockstep(lockstep_config);
    ASSERT_TRUE(lockstep.initialize(1));
    const double last = drive_opponent_to(lockstep, camera_info, field, 30);
    const double late_stamp = last + kFrameDt;
    lockstep.predict(late_stamp, CommandFeedback{});
    lockstep.correct(make_moving_opponent_keypoints(late_stamp, kLateMeasurementShiftPixels), field,
                     camera_info);
    lockstep.predict(late_stamp + kReadAfterCorrectS, CommandFeedback{});
    const RobotDescriptionsStamped expected = lockstep.state();

    RobotFrontBackFilterConfiguration outrun_config = make_kalman_opponent_config();
    RobotFrontBackFilter outrun(outrun_config);
    ASSERT_TRUE(outrun.initialize(1));
    drive_opponent_to(outrun, camera_info, field, 30);
    outrun.predict(late_stamp + 0.1, CommandFeedback{});
    outrun.predict(late_stamp + 0.2, CommandFeedback{});
    outrun.correct(make_moving_opponent_keypoints(late_stamp, kLateMeasurementShiftPixels), field,
                   camera_info);
    outrun.predict(late_stamp + kReadAfterCorrectS, CommandFeedback{});
    const RobotDescriptionsStamped actual = outrun.state();

    const RobotDescription *expected_opponent = find_opponent(expected);
    const RobotDescription *actual_opponent = find_opponent(actual);
    ASSERT_NE(expected_opponent, nullptr);
    ASSERT_NE(actual_opponent, nullptr);
    EXPECT_NEAR(actual_opponent->pose.position.x, expected_opponent->pose.position.x, 0.005);
    EXPECT_NEAR(actual_opponent->pose.position.y, expected_opponent->pose.position.y, 0.005);
}

TEST(RobotFrontBackFilterTest, CoastedStateStampIncludesRenderLead) {
    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    RobotFrontBackFilterConfiguration config = make_kalman_opponent_config();
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));
    const double last = drive_opponent_to(filter, camera_info, field, 30);

    filter.predict(last + kFrameDt, CommandFeedback{});
    EXPECT_NEAR(filter.state().header.stamp, last + kFrameDt + plant_golden::kParam_delay_s, 1e-9);
}

TEST(RobotFrontBackFilterTest, SizeOverrideReplacesDetectedSize) {
    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    RobotFrontBackFilterConfiguration config = make_kalman_opponent_config();
    const Size known{0.61, 0.52, 0.17};
    config.robot_size_meters_per_label[Label::OPPONENT] = known;

    TestDiagnosticsLogger::reset();
    auto backend = std::make_shared<RecordingBackend>();
    DiagnosticsLogger::initialize({backend});
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));
    drive_opponent_to(filter, camera_info, field, 5);
    DiagnosticsLogger::publish();

    const RobotDescriptionsStamped state = filter.state();
    const RobotDescription *opponent = find_opponent(state);
    ASSERT_NE(opponent, nullptr);
    EXPECT_DOUBLE_EQ(opponent->size.x, known.x);
    EXPECT_DOUBLE_EQ(opponent->size.y, known.y);
    EXPECT_DOUBLE_EQ(opponent->size.z, known.z);
    bool logged_detected = false;
    for (const DiagnosticStatusSnapshot &snapshot : backend->last_snapshots()) {
        logged_detected = logged_detected || snapshot.subsection == "detected_size";
    }
    TestDiagnosticsLogger::reset();
    EXPECT_TRUE(logged_detected);
}

}  // namespace auto_battlebot
