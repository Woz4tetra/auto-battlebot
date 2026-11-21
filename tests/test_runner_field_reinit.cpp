#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <opencv2/core.hpp>

#include "control_loop/config.hpp"
#include "control_loop/stepped_control_loop.hpp"
#include "field_filter/noop_field_filter.hpp"
#include "keypoint_model/noop_keypoint_model.hpp"
#include "mask_model/noop_mask_model.hpp"
#include "navigation/noop_navigation.hpp"
#include "publisher/noop_publisher.hpp"
#include "rgbd_camera/noop_rgbd_camera.hpp"
#include "robot_filter/noop_robot_filter.hpp"
#include "runner.hpp"
#include "target_selector/noop_target.hpp"
#include "time/manual_clock.hpp"
#include "transmitter/noop_transmitter.hpp"

namespace auto_battlebot {
namespace {

class PlaneFieldFilter : public NoopFieldFilter {
   public:
    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(const CameraData &,
                                                                    const MaskStamped &) override {
        auto field = std::make_shared<FieldDescriptionWithInlierPoints>();
        field->header.frame_id = FrameId::FIELD;
        return field;
    }
};

class SolidMaskModel : public NoopMaskModel {
   public:
    MaskStamped update(RgbImage) override {
        MaskStamped mask;
        mask.mask.mask = cv::Mat::ones(4, 4, CV_8UC1);
        return mask;
    }
};

CameraInfo overhead_camera() {
    CameraInfo info;
    info.width = 640;
    info.height = 480;
    info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    info.intrinsics.at<double>(0, 0) = 500.0;
    info.intrinsics.at<double>(1, 1) = 500.0;
    info.intrinsics.at<double>(0, 2) = 320.0;
    info.intrinsics.at<double>(1, 2) = 240.0;
    return info;
}

FieldDescription overhead_field() {
    FieldDescription field;
    field.child_frame_id = FrameId::FIELD;
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 2) = -1.0;
    tf(2, 3) = 3.0;
    field.tf_camera_from_fieldcenter.tf = tf;
    return field;
}

ModelResultStamped detection_at(double x, double y, double stamp) {
    ModelResultStamped result;
    result.header.frame_id = FrameId::CAMERA;
    result.header.stamp = stamp;
    Keypoint front;
    front.label = Label::OPPONENT;
    front.keypoint_label = KeypointLabel::OPPONENT_FRONT;
    front.x = x - 10.0;
    front.y = y;
    front.confidence = 0.9;
    front.detection_index = 1;
    Keypoint back = front;
    back.keypoint_label = KeypointLabel::OPPONENT_BACK;
    back.x = x + 10.0;
    result.keypoints = {front, back};
    result.boxes.resize(2);
    result.boxes[1] = BoundingBox{x - 30.0, y - 30.0, x + 30.0, y + 30.0};
    return result;
}

TEST(RunnerFieldReinit, FieldReinitClearsTheStaticGateClusters) {
    StaticDetectionGateConfiguration gate_config;
    gate_config.enable = true;
    auto gate = std::make_shared<StaticDetectionGate>(gate_config);
    const CameraInfo camera_info = overhead_camera();
    const FieldDescription field = overhead_field();
    gate->filter(detection_at(100.0, 100.0, 0.0), camera_info, field, 0.0);
    gate->filter(detection_at(500.0, 400.0, 0.1), camera_info, field, 0.1);
    ASSERT_EQ(gate->last_stats().tracked_clusters, 2);

    auto camera = std::make_shared<NoopRgbdCamera>();
    auto keypoint_model = std::make_shared<NoopKeypointModel>();
    auto loop = std::make_shared<ControlLoop>(
        std::make_shared<NoopRobotFilter>(), std::make_shared<NoopTarget>(),
        std::make_shared<NoopNavigation>(), std::make_shared<NoopTransmitter>(),
        std::make_shared<ManualClock>(), nullptr);
    RunnerRemote remote;
    auto commands = remote.commands;
    Runner runner(
        RunnerConfiguration{}, camera, std::make_shared<HealthLogger>(HealthConfiguration{}),
        std::make_shared<SolidMaskModel>(), std::make_shared<PlaneFieldFilter>(), keypoint_model,
        std::make_shared<KeypointHeightGate>(KeypointHeightGateConfiguration{}), gate,
        std::make_shared<SteppedControlLoop>(loop, 0.0), std::make_shared<NoopPublisher>(),
        [](SystemAction) {}, [](const std::string &) {}, nullptr, nullptr, std::move(remote));

    commands->post(remote::ReinitFieldCommand{});
    ASSERT_TRUE(runner.tick());

    gate->filter(detection_at(300.0, 200.0, 0.2), camera_info, field, 0.2);
    EXPECT_EQ(gate->last_stats().tracked_clusters, 1);
}

}  // namespace
}  // namespace auto_battlebot
