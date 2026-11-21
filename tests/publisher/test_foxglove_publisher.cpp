#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <mcap/reader.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "publisher/foxglove_publisher.hpp"
#include "publisher/output_channel.hpp"
#include "viz/frame.hpp"
#include "viz/viz_sink.hpp"

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

Recording record_to(const std::function<void(const std::shared_ptr<McapRecorder> &)> &publish) {
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    const auto cwd = std::filesystem::current_path();
    const auto scratch = std::filesystem::temp_directory_path() / "auto_battlebot_pub_record_test";
    std::filesystem::create_directories(scratch);
    std::filesystem::current_path(scratch);
    auto recorder = std::make_shared<McapRecorder>("test_profile", scratch);
    const auto file = recorder->file_path();
    publish(recorder);
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

Recording record(const std::function<void(FoxglovePublisher &)> &publish) {
    return record_to([&](const std::shared_ptr<McapRecorder> &recorder) {
        FoxglovePublisher publisher(nullptr, recorder);
        publish(publisher);
    });
}

FieldDescriptionWithInlierPoints square_field() {
    FieldDescriptionWithInlierPoints field;
    field.header.frame_id = FrameId::CAMERA;
    field.size.size.x = 1.0;
    field.size.size.y = 1.0;
    field.tf_camera_from_fieldcenter.tf = Eigen::MatrixXd::Identity(4, 4);
    return field;
}

class SubscribingRelay {
   public:
    explicit SubscribingRelay(const std::string &path) : path_(path) {
        ::unlink(path_.c_str());
        listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);
        EXPECT_EQ(::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
        EXPECT_EQ(::listen(listen_fd_, 1), 0);
    }
    ~SubscribingRelay() {
        if (client_fd_ >= 0) ::close(client_fd_);
        ::close(listen_fd_);
        ::unlink(path_.c_str());
    }

    bool accept_client() {
        timeval tv{3, 0};
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        client_fd_ = ::accept(listen_fd_, nullptr, nullptr);
        if (client_fd_ < 0) return false;
        ::setsockopt(client_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        return true;
    }

    std::optional<viz::MessageFrame> next_message() {
        for (int i = 0; i < 50; ++i) {
            std::byte header[4];
            if (!read_exact(header, 4)) return std::nullopt;
            size_t off = 0;
            uint32_t len = 0;
            viz::detail::get_u32(header, 4, off, len);
            std::vector<std::byte> body(len);
            if (!read_exact(body.data(), len)) return std::nullopt;
            if (viz::frame_kind(body.data(), body.size()) != viz::FrameKind::MESSAGE) continue;
            viz::MessageFrame message;
            if (viz::decode_message(body.data(), body.size(), message)) return message;
        }
        return std::nullopt;
    }

    bool subscribe_to(const std::string &topic) {
        for (int i = 0; i < 200; ++i) {
            std::byte header[4];
            if (!read_exact(header, 4)) return false;
            size_t off = 0;
            uint32_t len = 0;
            viz::detail::get_u32(header, 4, off, len);
            std::vector<std::byte> body(len);
            if (!read_exact(body.data(), len)) return false;
            viz::AdvertiseFrame advertise;
            if (viz::frame_kind(body.data(), body.size()) != viz::FrameKind::ADVERTISE) continue;
            if (!viz::decode_advertise(body.data(), body.size(), advertise)) continue;
            if (advertise.topic != topic) continue;
            const auto frame = viz::encode_subscriber_count(advertise.channel_id, 1);
            return ::send(client_fd_, frame.data(), frame.size(), MSG_NOSIGNAL) ==
                   static_cast<ssize_t>(frame.size());
        }
        return false;
    }

   private:
    bool read_exact(std::byte *out, size_t len) {
        size_t got = 0;
        while (got < len) {
            const ssize_t n = ::recv(client_fd_, out + got, len - got, 0);
            if (n <= 0) return false;
            got += static_cast<size_t>(n);
        }
        return true;
    }

    std::string path_;
    int listen_fd_ = -1;
    int client_fd_ = -1;
};

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
    EXPECT_NO_THROW(publisher.publish_keypoint_detections(unframed_detections));
    DetectionsStamped framed_detections;
    framed_detections.header.frame_id = FrameId::CAMERA;
    EXPECT_NO_THROW(publisher.publish_keypoint_detections(framed_detections));

    NavigationVisualization nav;
    nav.header.frame_id = FrameId::FIELD;
    EXPECT_NO_THROW(publisher.publish_navigation(nav));
}

TEST(FoxglovePublisherTest, DestroyWhilePublishingIsClean) {
    CameraData camera;
    camera.rgb.header.frame_id = FrameId::CAMERA;
    camera.rgb.image = cv::Mat(1080, 1920, CV_8UC3, cv::Scalar(1, 2, 3));
    camera.camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    record_to([&](const std::shared_ptr<McapRecorder> &recorder) {
        for (int i = 0; i < 20; ++i) {
            FoxglovePublisher publisher(nullptr, recorder);
            publisher.publish_camera_data(camera);
            publisher.publish_camera_data(camera);
        }
    });
}

TEST(FoxglovePublisherTest, FrameMetaFromAnEarlierRecordingGenerationIsDropped) {
    CameraData camera;
    camera.rgb.header.frame_id = FrameId::CAMERA;
    camera.rgb.image = cv::Mat(8, 8, CV_8UC3, cv::Scalar(1, 2, 3));
    camera.camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    const Recording recording = record_to([&](const std::shared_ptr<McapRecorder> &recorder) {
        FoxglovePublisher publisher(nullptr, recorder);
        camera.frame_identity.video_frame_index = 7;
        camera.frame_identity.recording_generation = recorder->generation() - 1;
        publisher.publish_camera_data(camera);
        camera.frame_identity.video_frame_index = 8;
        camera.frame_identity.recording_generation = recorder->generation();
        publisher.publish_camera_data(camera);
    });
    const auto found = recording.find("/camera/frame_meta");
    ASSERT_NE(found, recording.end());
    ASSERT_EQ(found->second.size(), 1u);
    EXPECT_NE(found->second.front().data.find("8"), std::string::npos);
    EXPECT_EQ(found->second.front().data.find("7"), std::string::npos);
}

TEST(FoxglovePublisherTest, AnnotationsAreLiveOnly) {
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    const auto cwd = std::filesystem::current_path();
    const auto scratch = std::filesystem::temp_directory_path() / "auto_battlebot_pub_test";
    std::filesystem::create_directories(scratch);
    std::filesystem::current_path(scratch);
    auto recorder = std::make_shared<McapRecorder>("test_profile", scratch);
    const auto file = recorder->file_path();
    {
        FoxglovePublisher publisher(nullptr, recorder);
        DetectionsStamped detections;
        detections.header.frame_id = FrameId::CAMERA;
        detections.header.stamp = 2.0;
        Detection2D det;
        det.label = Label::MR_STABS_MK1;
        detections.detections.push_back(det);
        publisher.publish_keypoint_detections(detections);
    }
    recorder->close();
    std::filesystem::current_path(cwd);

    ASSERT_TRUE(std::filesystem::exists(file));
    std::ifstream in(file, std::ios::binary);
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("/keypoint_detections"), std::string::npos);
    EXPECT_EQ(contents.find("/keypoint_detections/annotations"), std::string::npos);
    EXPECT_NE(contents.find("auto_battlebot.Detections"), std::string::npos);
    std::filesystem::remove_all(scratch);
}

TEST(FoxglovePublisherTest, SubscribedAnnotationsStayOutOfTheRecording) {
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    const auto cwd = std::filesystem::current_path();
    const auto scratch = std::filesystem::temp_directory_path() / "auto_battlebot_pub_sub_test";
    std::filesystem::create_directories(scratch);
    std::filesystem::current_path(scratch);
    const std::string socket_path =
        "/tmp/auto_battlebot_pub_test_" + std::to_string(::getpid()) + ".sock";
    SubscribingRelay relay(socket_path);
    auto sink = std::make_shared<VizSink>(socket_path);
    auto recorder = std::make_shared<McapRecorder>("test_profile", scratch);
    const auto file = recorder->file_path();
    {
        FoxglovePublisher publisher(sink, recorder);
        ASSERT_TRUE(relay.accept_client());
        ASSERT_TRUE(relay.subscribe_to("/keypoint_detections/annotations"));
        for (int i = 0; i < 300; ++i) {
            DetectionsStamped detections;
            detections.header.frame_id = FrameId::CAMERA;
            detections.header.stamp = 2.0;
            Detection2D det;
            det.label = Label::MR_STABS_MK1;
            detections.detections.push_back(det);
            publisher.publish_keypoint_detections(detections);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    recorder->close();
    std::filesystem::current_path(cwd);

    ASSERT_TRUE(std::filesystem::exists(file));
    std::ifstream in(file, std::ios::binary);
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("/keypoint_detections"), std::string::npos);
    EXPECT_EQ(contents.find("/keypoint_detections/annotations"), std::string::npos);
    std::filesystem::remove_all(scratch);
}

TEST(OutputChannelTest, ZeroLengthPayloadsNeverReachTheRelay) {
    const std::string socket_path =
        "/tmp/auto_battlebot_chan_test_" + std::to_string(::getpid()) + ".sock";
    SubscribingRelay relay(socket_path);
    auto sink = std::make_shared<VizSink>(socket_path);
    OutputChannel channel("/empty_probe", "json", VizSchema{"n", "jsonschema", "{}"}, false, sink,
                          nullptr);
    ASSERT_TRUE(relay.accept_client());
    ASSERT_TRUE(relay.subscribe_to("/empty_probe"));
    for (int i = 0; i < 200 && sink->num_subscribers(1) == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    channel.log(std::string(), 1);
    channel.log(std::string("{}"), 2);
    const auto first = relay.next_message();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->log_time_ns, 2u);
    EXPECT_EQ(first->payload.size(), 2u);
}

TEST(FoxglovePublisherTest, EmptyHeaderDetectionsAreNotRecorded) {
    const Recording recording = record([](FoxglovePublisher &publisher) {
        publisher.publish_keypoint_detections(DetectionsStamped{});
    });
    EXPECT_EQ(recording.count("/keypoint_detections"), 0u);
}

TEST(FoxglovePublisherTest, DetectionsAreRecordedAtTheFrameStamp) {
    const Recording recording = record([](FoxglovePublisher &publisher) {
        DetectionsStamped detections;
        detections.header.frame_id = FrameId::CAMERA;
        detections.header.stamp = 2.5;
        publisher.publish_keypoint_detections(detections);
    });
    ASSERT_EQ(recording.at("/keypoint_detections").size(), 1u);
    EXPECT_EQ(recording.at("/keypoint_detections").front().log_time, 2500000000u);
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
