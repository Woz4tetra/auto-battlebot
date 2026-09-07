#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "publisher/foxglove_publisher.hpp"

namespace auto_battlebot {

// FoxglovePublisher with neither sink nor recorder must still accept every publish: the app
// runs this way when the relay is down and recording is off.
TEST(FoxglovePublisherTest, PublishesWithoutSinks) {
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    FoxglovePublisher publisher(nullptr, nullptr);

    CameraData camera;
    camera.rgb.header.frame_id = FrameId::CAMERA;
    camera.rgb.image = cv::Mat(8, 8, CV_8UC3, cv::Scalar(1, 2, 3));
    camera.camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    EXPECT_NO_THROW(publisher.publish_camera_data(camera));

    FieldDescriptionWithInlierPoints field;
    field.header.frame_id = FrameId::CAMERA;
    field.size.size.x = 1.0;
    field.size.size.y = 1.0;
    EXPECT_NO_THROW(publisher.publish_initial_field_description(field));
    EXPECT_NO_THROW(publisher.publish_field_description(field, field));
    EXPECT_NO_THROW(publisher.publish_hazards(field));

    RobotDescriptionsStamped robots;
    robots.header.frame_id = FrameId::FIELD;
    EXPECT_NO_THROW(publisher.publish_robots(robots));

    DetectionsStamped detections;  // EMPTY frame id: skipped, never throws
    EXPECT_NO_THROW(publisher.publish_blob_detections(detections));
    detections.header.frame_id = FrameId::CAMERA;
    EXPECT_NO_THROW(publisher.publish_keypoint_detections(detections));

    NavigationVisualization nav;
    nav.header.frame_id = FrameId::FIELD;
    EXPECT_NO_THROW(publisher.publish_navigation(nav));
}

// With a recorder attached, the annotation topics never land in the file but the JSON
// detections do.
TEST(FoxglovePublisherTest, AnnotationsAreLiveOnly) {
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    const auto cwd = std::filesystem::current_path();
    const auto scratch = std::filesystem::temp_directory_path() / "auto_battlebot_pub_test";
    std::filesystem::create_directories(scratch);
    std::filesystem::current_path(scratch);
    auto recorder = std::make_shared<McapRecorder>("test_profile");
    const auto file = recorder->file_path();
    {
        FoxglovePublisher publisher(nullptr, recorder);
        DetectionsStamped detections;
        detections.header.frame_id = FrameId::CAMERA;
        detections.header.stamp = 2.0;
        Detection2D det;
        det.label = Label::MR_STABS_MK1;
        detections.detections.push_back(det);
        publisher.publish_blob_detections(detections);
    }
    recorder->close();
    std::filesystem::current_path(cwd);

    ASSERT_TRUE(std::filesystem::exists(file));
    std::ifstream in(file, std::ios::binary);
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("/blob_detections"), std::string::npos);
    EXPECT_EQ(contents.find("/blob_detections/annotations"), std::string::npos);
    EXPECT_NE(contents.find("auto_battlebot.Detections"), std::string::npos);
    std::filesystem::remove_all(scratch);
}

}  // namespace auto_battlebot
