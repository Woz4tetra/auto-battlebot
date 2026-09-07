#include <gtest/gtest.h>

#include "foxglove_adapters/detections.hpp"

namespace auto_battlebot {
class DetectionsAdapterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        detections.header.stamp = 1.5;
        detections.header.frame_id = FrameId::CAMERA;
        detections.image_width = 1280;
        detections.image_height = 720;
        Detection2D det;
        det.x1 = 10.0;
        det.y1 = 20.0;
        det.x2 = 110.0;
        det.y2 = 120.0;
        det.confidence = 0.9123;
        det.class_id = 0;
        det.label = Label::MR_STABS_MK1;
        det.keypoints.push_back({55.0, 70.0, 0.98});
        detections.detections.push_back(det);
    }
    DetectionsStamped detections;
};

TEST_F(DetectionsAdapterTest, JsonPayloadMatchesContract) {
    EXPECT_EQ(foxglove_adapters::to_detections_json(detections),
              "{\"stamp\":1.500000000,\"w\":1280,\"h\":720,\"dets\":[{\"x1\":10.0,\"y1\":20.0,"
              "\"x2\":110.0,\"y2\":120.0,\"conf\":0.9123,\"class_id\":0,\"label\":\"mr_stabs_mk1\","
              "\"kps\":[[55.0,70.0,0.9800]]}]}");
}

TEST_F(DetectionsAdapterTest, JsonOmitsKeypointsWhenAbsent) {
    detections.detections[0].keypoints.clear();
    const auto json = foxglove_adapters::to_detections_json(detections);
    EXPECT_EQ(json.find("kps"), std::string::npos);
}

TEST_F(DetectionsAdapterTest, AnnotationsDrawBoxKeypointAndLabel) {
    auto annotations = foxglove_adapters::to_image_annotations(detections);

    ASSERT_EQ(annotations.points.size(), 1u);
    EXPECT_EQ(annotations.points[0].type,
              foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LOOP);
    ASSERT_EQ(annotations.points[0].points.size(), 4u);
    EXPECT_DOUBLE_EQ(annotations.points[0].points[2].x, 110.0);
    EXPECT_DOUBLE_EQ(annotations.points[0].points[2].y, 120.0);

    ASSERT_EQ(annotations.circles.size(), 1u);
    EXPECT_DOUBLE_EQ(annotations.circles[0].position->x, 55.0);

    ASSERT_EQ(annotations.texts.size(), 1u);
    EXPECT_EQ(annotations.texts[0].text, "mr_stabs_mk1 0.91");
    ASSERT_TRUE(annotations.timestamp.has_value());
    EXPECT_EQ(annotations.timestamp->sec, 1u);
    EXPECT_EQ(annotations.timestamp->nsec, 500000000u);
}

}  // namespace auto_battlebot
