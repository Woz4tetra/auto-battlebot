#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "config/config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "mask_model/fixed_mask_model.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {
namespace {

template <typename Parse>
auto parse_section(const std::string &section, const std::string &type, Parse parse) {
    const toml::table table = toml::parse("type = \"" + type + "\"\n");
    ConfigParser parser(table, section);
    return parse(parser);
}

std::shared_ptr<ClockInterface> manual_clock() {
    return make_clock(*parse_section("clock", "ManualClock", parse_clock_config));
}

RgbImage image_at(double stamp) {
    RgbImage image;
    image.header = {stamp, FrameId::CAMERA};
    image.image = cv::Mat::zeros(4, 4, CV_8UC3);
    return image;
}

TEST(FactoryVariantTest, NoopMaskModelMasksNothing) {
    auto config = parse_section("field_model", "NoopMaskModel", parse_mask_model_config);
    auto model = make_mask_model(*config);
    ASSERT_TRUE(model->initialize());
    const MaskStamped mask = model->update(image_at(3.0));
    EXPECT_TRUE(mask.mask.mask.empty());
}

TEST(FactoryVariantTest, FixedMaskModelMarksEveryPixelFieldAtTheImageStamp) {
    auto config = parse_section("field_model", "FixedMaskModel", parse_mask_model_config);
    auto model = make_mask_model(*config);
    ASSERT_NE(dynamic_cast<FixedMaskModel *>(model.get()), nullptr);
    ASSERT_TRUE(model->initialize());
    const MaskStamped mask = model->update(image_at(3.0));
    EXPECT_EQ(mask.header.stamp, 3.0);
    EXPECT_EQ(mask.mask.label, Label::FIELD);
    EXPECT_EQ(cv::countNonZero(mask.mask.mask), mask.mask.mask.rows * mask.mask.mask.cols);
}

TEST(FactoryVariantTest, NoopFieldFilterReturnsAnEmptyField) {
    auto config = parse_section("field_filter", "NoopFieldFilter", parse_field_filter_config);
    auto filter = make_field_filter(*config);
    const auto initial = filter->compute_field(CameraData{}, MaskStamped{});
    ASSERT_NE(initial, nullptr);
    const FieldDescription field = filter->track_field(TransformStamped{}, initial);
    EXPECT_EQ(field.child_frame_id, FrameId::EMPTY);
    EXPECT_TRUE(field.hazards.empty());
}

TEST(FactoryVariantTest, NoopKeypointModelDetectsNothing) {
    auto config = parse_section("keypoint_model", "NoopKeypointModel", parse_keypoint_model_config);
    auto model = make_keypoint_model(*config, nullptr, nullptr);
    ASSERT_TRUE(model->initialize());
    const ModelResultStamped result = model->update(image_at(1.0), CameraInfo{});
    EXPECT_TRUE(result.keypoints.empty());
    EXPECT_TRUE(result.boxes.empty());
}

TEST(FactoryVariantTest, NoopRobotFilterTracksNobody) {
    auto config = parse_section("robot_filter", "NoopRobotFilter", parse_robot_filter_config);
    auto filter = make_robot_filter(*config, manual_clock());
    ASSERT_TRUE(filter->initialize(1));
    filter->correct(ModelResultStamped{}, FieldDescription{}, CameraInfo{});
    EXPECT_TRUE(filter->state().descriptions.empty());
}

TEST(FactoryVariantTest, NoopTargetAlwaysPicksTheFieldCenter) {
    auto config = parse_section("target_selector", "NoopTarget", parse_target_selector_config);
    auto selector = make_target_selector(*config);
    const auto target =
        selector->get_target(RobotDescriptionsStamped{}, FieldDescription{}, BehaviorMode::ATTACK);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(target->pose.x, 0.0);
    EXPECT_EQ(target->pose.y, 0.0);
}

TEST(FactoryVariantTest, NoopNavigationCommandsZeroAndEchoesTheRobots) {
    auto config = parse_section("navigation", "NoopNavigation", parse_navigation_config);
    auto navigation = make_navigation(*config, manual_clock());
    ASSERT_TRUE(navigation->initialize());
    RobotDescriptionsStamped robots;
    robots.header.stamp = 7.0;
    const VelocityCommand command = navigation->update(robots, FieldDescription{}, {});
    EXPECT_EQ(command.forward_mps, 0.0);
    EXPECT_EQ(command.yaw_rate_radps, 0.0);
    EXPECT_EQ(navigation->get_last_visualization().header.stamp, 7.0);
}

TEST(FactoryVariantTest, NoopTransmitterAcceptsCommandsAndReportsNothing) {
    auto config = parse_section("transmitter", "NoopTransmitter", parse_transmitter_config);
    auto transmitter = make_transmitter(*config, manual_clock(), nullptr, nullptr);
    ASSERT_TRUE(transmitter->initialize());
    transmitter->send(StickCommand{});
    EXPECT_TRUE(transmitter->update().stick_commands.empty());
    EXPECT_FALSE(transmitter->did_init_button_press());
}

TEST(FactoryVariantTest, NoopPublisherTakesEveryMessageWithoutASink) {
    auto config = parse_section("publisher", "NoopPublisher", parse_publisher_config);
    auto publisher = make_publisher(*config, nullptr);
    ASSERT_NE(publisher, nullptr);
    publisher->publish_camera_data(CameraData{});
    publisher->publish_robots(RobotDescriptionsStamped{});
    publisher->publish_navigation(NavigationVisualization{});
}

TEST(FactoryVariantTest, SystemClockReadsWallTime) {
    auto config = parse_section("clock", "SystemClock", parse_clock_config);
    auto clock = make_clock(*config);
    const double before = now();
    const double read = clock->now();
    EXPECT_GE(read, before);
    EXPECT_LE(read, now());
}

TEST(FactoryVariantTest, CameraFollowingClockStartsAtZeroThenRunsFromTheCameraStamp) {
    auto config = parse_section("clock", "CameraFollowingClock", parse_clock_config);
    auto clock = make_clock(*config);
    EXPECT_EQ(clock->now(), 0.0);
    clock->set(1000.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const double read = clock->now();
    EXPECT_GE(read, 1000.015);
    EXPECT_LT(read, 1001.0);
}

class FakeSimServer {
   public:
    FakeSimServer() {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        ::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
        ::listen(listen_fd_, 1);
        socklen_t len = sizeof(addr);
        ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
        port_ = ntohs(addr.sin_port);
        thread_ = std::thread([this] { serve_one_frame(); });
    }

    ~FakeSimServer() {
        if (thread_.joinable()) thread_.join();
        ::close(listen_fd_);
    }

    int port() const { return port_; }
    std::array<double, 3> request{};

   private:
    template <typename T>
    void put(std::vector<uint8_t> &out, const T &value) {
        const auto *bytes = reinterpret_cast<const uint8_t *>(&value);
        out.insert(out.end(), bytes, bytes + sizeof(T));
    }

    void serve_one_frame() {
        const int fd = ::accept(listen_fd_, nullptr, nullptr);
        if (fd < 0) return;
        ::recv(fd, request.data(), sizeof(request), MSG_WAITALL);
        std::vector<uint8_t> out;
        put(out, uint32_t{2});
        put(out, uint32_t{1});
        for (int i = 0; i < 16; ++i) put(out, i % 5 == 0 ? 1.0 : 0.0);
        for (double v : {500.0, 501.0, 1.0, 0.5, 12.5}) put(out, v);
        out.insert(out.end(), 2 * 1 * 3, uint8_t{200});
        put(out, 1.5f);
        put(out, 2.5f);
        put(out, uint32_t{2});
        for (double v : {0.1, 0.2, 0.3, -0.4, -0.5, 0.6}) put(out, v);
        put(out, 1.0);
        put(out, 12.4);
        for (double v : {1.0, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0}) put(out, v);
        ::send(fd, out.data(), out.size(), MSG_NOSIGNAL);
        ::close(fd);
    }

    int listen_fd_ = -1;
    int port_ = 0;
    std::thread thread_;
};

TEST(FactoryVariantTest, SimRgbdCameraSendsTheCommandAndDecodesAFrame) {
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    FakeSimServer server;
    const toml::table table =
        toml::parse("type = \"SimRgbdCamera\"\nsim_port = " + std::to_string(server.port()) + "\n");
    ConfigParser parser(table, "rgbd_camera");
    auto config = parse_rgbd_camera_config(parser);
    auto camera = make_rgbd_camera(*config, nullptr);
    ASSERT_TRUE(camera->initialize());
    CameraData data;
    ASSERT_TRUE(camera->get(data));
    EXPECT_EQ(data.rgb.header.stamp, 12.5);
    EXPECT_EQ(data.rgb.image.cols, 2);
    EXPECT_EQ(data.rgb.image.rows, 1);
    EXPECT_EQ(data.depth.image.at<float>(0, 1), 2.5f);
    EXPECT_EQ(data.camera_info.intrinsics.at<double>(0, 0), 500.0);
    ASSERT_TRUE(data.ground_truth.has_value());
    EXPECT_DOUBLE_EQ(data.ground_truth->our_robot.x, 0.1);
    ASSERT_EQ(data.ground_truth->opponents.size(), 1u);
    EXPECT_DOUBLE_EQ(data.ground_truth->opponents[0].y, -0.5);
    EXPECT_TRUE(data.sim_measurements.delivered);
    EXPECT_DOUBLE_EQ(data.sim_measurements.capture_stamp, 12.4);
    EXPECT_TRUE(data.sim_measurements.robots[0].measured);
    EXPECT_TRUE(data.pose_tracking_ready);
}

}  // namespace
}  // namespace auto_battlebot
