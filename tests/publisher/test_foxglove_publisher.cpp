#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <mcap/reader.hpp>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "publisher/foxglove_publisher.hpp"

namespace auto_battlebot {

struct FoxglovePublisherTestAccess {
    static cv::Mat resize_for_preview(const FoxglovePublisher &publisher, const cv::Mat &image) {
        return publisher.resize_for_preview(image);
    }
};

namespace {

struct RecordedMessage {
    uint64_t log_time;
    std::string data;
};

using Recording = std::map<std::string, std::vector<RecordedMessage>>;

Recording record(const std::function<void(FoxglovePublisher &)> &publish) {
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    const auto cwd = std::filesystem::current_path();
    const auto scratch = std::filesystem::temp_directory_path() / "auto_battlebot_pub_record_test";
    std::filesystem::create_directories(scratch);
    std::filesystem::current_path(scratch);
    auto recorder = std::make_shared<McapRecorder>("test_profile");
    const auto file = recorder->file_path();
    {
        FoxglovePublisher publisher(nullptr, recorder);
        publish(publisher);
    }
    recorder->close();
    std::filesystem::current_path(cwd);

    Recording out;
    mcap::McapReader reader;
    if (reader.open(file.string()).ok()) {
        for (const auto &view : reader.readMessages()) {
            const auto *data = reinterpret_cast<const char *>(view.message.data);
            out[view.channel->topic].push_back(
                RecordedMessage{view.message.logTime, std::string(data, view.message.dataSize)});
        }
        reader.close();
    }
    std::filesystem::remove_all(scratch);
    return out;
}

FieldDescriptionWithInlierPoints square_field() {
    FieldDescriptionWithInlierPoints field;
    field.header.frame_id = FrameId::CAMERA;
    field.size.size.x = 1.0;
    field.size.size.y = 1.0;
    field.tf_camera_from_fieldcenter.tf = Eigen::MatrixXd::Identity(4, 4);
    return field;
}

}  // namespace

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

    const DetectionsStamped unframed_detections;
    EXPECT_NO_THROW(publisher.publish_blob_detections(unframed_detections));
    DetectionsStamped framed_detections;
    framed_detections.header.frame_id = FrameId::CAMERA;
    EXPECT_NO_THROW(publisher.publish_keypoint_detections(framed_detections));

    NavigationVisualization nav;
    nav.header.frame_id = FrameId::FIELD;
    EXPECT_NO_THROW(publisher.publish_navigation(nav));
}

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

TEST(FoxglovePublisherTest, EmptyHeaderDetectionsAreNotRecorded) {
    const Recording recording = record([](FoxglovePublisher &publisher) {
        publisher.publish_blob_detections(DetectionsStamped{});
        publisher.publish_keypoint_detections(DetectionsStamped{});
    });
    EXPECT_EQ(recording.count("/blob_detections"), 0u);
    EXPECT_EQ(recording.count("/keypoint_detections"), 0u);
}

TEST(FoxglovePublisherTest, DetectionsAreRecordedAtTheFrameStamp) {
    const Recording recording = record([](FoxglovePublisher &publisher) {
        DetectionsStamped detections;
        detections.header.frame_id = FrameId::CAMERA;
        detections.header.stamp = 2.5;
        publisher.publish_blob_detections(detections);
    });
    ASSERT_EQ(recording.at("/blob_detections").size(), 1u);
    EXPECT_EQ(recording.at("/blob_detections").front().log_time, 2500000000u);
}

TEST(FoxglovePublisherTest, HazardsArePublishedEvenWhenThereAreNone) {
    const Recording recording =
        record([](FoxglovePublisher &publisher) { publisher.publish_hazards(FieldDescription{}); });
    EXPECT_EQ(recording.at("/hazard_markers").size(), 1u);
}

TEST(FoxglovePublisherTest, FieldTransformIsRepublishedOnEveryCall) {
    const Recording recording = record([](FoxglovePublisher &publisher) {
        const FieldDescriptionWithInlierPoints field = square_field();
        publisher.publish_field_description(field, field);
        publisher.publish_field_description(field, field);
        publisher.publish_field_description(field, field);
    });
    EXPECT_EQ(recording.at("/tf").size(), 3u);
}

TEST(FoxglovePublisherTest, FieldMaskIsStampedInTheFrozenCameraWorldFrame) {
    const Recording recording = record([](FoxglovePublisher &publisher) {
        MaskStamped mask;
        mask.header.frame_id = FrameId::CAMERA;
        mask.mask.mask = cv::Mat(8, 8, CV_8UC1, cv::Scalar(1));
        RgbImage image;
        CameraInfo info;
        info.header.frame_id = FrameId::CAMERA;
        info.width = 8;
        info.height = 8;
        info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        info.distortion = cv::Mat::zeros(1, 5, CV_64F);
        publisher.publish_field_mask(mask, image, info);
    });
    EXPECT_NE(recording.at("/field_mask").front().data.find("camera_world"), std::string::npos);
    EXPECT_NE(recording.at("/field_mask/camera_info").front().data.find("camera_world"),
              std::string::npos);
}

TEST(FoxglovePublisherTest, PreviewDimensionsAreAlwaysEven) {
    FoxglovePublisherConfiguration config;
    config.preview_width = 321;
    FoxglovePublisher publisher(nullptr, nullptr, config);
    const cv::Mat small =
        FoxglovePublisherTestAccess::resize_for_preview(publisher, cv::Mat(481, 641, CV_8UC3));
    EXPECT_EQ(small.cols % 2, 0);
    EXPECT_EQ(small.rows % 2, 0);
    const cv::Mat odd_source =
        FoxglovePublisherTestAccess::resize_for_preview(publisher, cv::Mat(101, 201, CV_8UC3));
    EXPECT_EQ(odd_source.cols % 2, 0);
    EXPECT_EQ(odd_source.rows % 2, 0);
}

}  // namespace auto_battlebot
