#include <gtest/gtest.h>

#include "field_filter/fixed_field_filter.hpp"

namespace auto_battlebot {

TEST(FixedFieldFilterTest, TrackFieldPassesSimCameraPose) {
    FixedFieldFilter filter(2.4, 2.4);
    TransformStamped tf_visodom_from_camera;
    tf_visodom_from_camera.transform.tf = Eigen::Matrix4d::Identity();
    tf_visodom_from_camera.transform.tf(0, 3) = 1.5;
    tf_visodom_from_camera.transform.tf(2, 3) = 3.0;

    const FieldDescription description = filter.track_field(tf_visodom_from_camera, nullptr);

    EXPECT_EQ(description.tf_camera_from_fieldcenter.tf, tf_visodom_from_camera.transform.tf);
    EXPECT_NEAR(description.size.size.x, 2.4, 1e-9);
}

TEST(FixedFieldFilterTest, ComputeFieldMarksThePlaneFound) {
    FixedFieldFilter filter(2.4, 1.2);
    const auto description = filter.compute_field(CameraData{}, MaskStamped{});
    EXPECT_NE(description->header.frame_id, FrameId::EMPTY);
    EXPECT_NEAR(description->size.size.y, 1.2, 1e-9);
}
}  // namespace auto_battlebot
