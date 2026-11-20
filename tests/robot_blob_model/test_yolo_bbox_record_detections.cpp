#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "robot_blob_model/yolo_bbox_robot_blob_model.hpp"

namespace auto_battlebot {

struct YoloBboxRobotBlobModelTestAccess {
    using Detection = YoloBboxRobotBlobModel::Detection;

    static ModelResultStamped record(YoloBboxRobotBlobModel &model,
                                     const std::vector<Detection> &detections) {
        ModelResultStamped result;
        model.record_detections(detections, cv::Mat(), cv::Size(640, 640), cv::Size(640, 640),
                                result);
        return result;
    }

    static size_t last_detection_count(const YoloBboxRobotBlobModel &model) {
        return model.last_detections_original_px().detections.size();
    }
};

namespace {

TEST(YoloBboxRecordDetectionsTest, ABoxIsRecordedForEveryDetectionEvenWithAnUnmappedClass) {
    YoloBboxRobotBlobModelConfiguration config;
    config.label_indices = {Label::OPPONENT};
    YoloBboxRobotBlobModel model(config, nullptr);

    using Detection = YoloBboxRobotBlobModelTestAccess::Detection;
    Detection mapped;
    mapped.x1 = 10.0f;
    mapped.y1 = 10.0f;
    mapped.x2 = 100.0f;
    mapped.y2 = 100.0f;
    mapped.confidence = 0.9f;
    mapped.class_id = 0;
    Detection unmapped = mapped;
    unmapped.class_id = 7;

    const ModelResultStamped result =
        YoloBboxRobotBlobModelTestAccess::record(model, {mapped, unmapped});

    EXPECT_EQ(result.boxes.size(), 2u);
    EXPECT_EQ(YoloBboxRobotBlobModelTestAccess::last_detection_count(model), 1u);
}

}  // namespace
}  // namespace auto_battlebot
