#include <gtest/gtest.h>

#include "keypoint_model/yolo_keypoint_model.hpp"

namespace auto_battlebot {

TEST(YoloKeypointModelTest, BoxesAlignWithDetectionIndex) {
    const std::vector<DetectionRow> rows = {
        {10.f, 20.f, 30.f, 40.f, 0.9f, 0.f},
        {1.f, 2.f, 3.f},
        {50.f, 60.f, 70.f, 80.f, 0.8f, 1.f},
    };
    const std::vector<BoundingBox> boxes = boxes_per_row(rows);
    ASSERT_EQ(boxes.size(), rows.size());
    EXPECT_DOUBLE_EQ(boxes[0].x1, 10.0);
    EXPECT_DOUBLE_EQ(boxes[1].x2, 0.0) << "short row keeps its slot with an empty box";
    EXPECT_DOUBLE_EQ(boxes[2].x1, 50.0) << "later box stays on its own detection index";
    EXPECT_DOUBLE_EQ(boxes[2].y2, 80.0);
}

TEST(PredictionMatrixTest, AtIndexesRowMajorByPrediction) {
    const std::vector<float> values = {0.f, 1.f, 2.f, 10.f, 11.f, 12.f};
    const PredictionMatrix matrix{values, 2, 3};
    EXPECT_FLOAT_EQ(matrix.at(0, 2), 2.f);
    EXPECT_FLOAT_EQ(matrix.at(1, 0), 10.f);
}

}  // namespace auto_battlebot
