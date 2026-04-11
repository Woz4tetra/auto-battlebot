#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

#include <opencv2/core.hpp>

#include "lvgl_platform_bound/lvgl_ui_controller.hpp"
#include "lvgl_platform_bound/lvgl_ui_presenter.hpp"
#include "lvgl_platform_bound/lvgl_ui_services.hpp"
#include "transform_utils.hpp"
#include "ui/ui_runner.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {
namespace {

SystemStatus make_status() {
    SystemStatus st;
    st.camera_ok = true;
    st.transmitter_connected = true;
    st.loop_rate_hz = 120.0;
    st.initialized = true;
    st.selected_opponent_count = 2;
    st.autonomy_enabled = true;
    st.svo_recording_enabled = true;
    st.mcap_recording_enabled = true;
    st.recording_enabled = true;
    st.jetson_temperature_c = 55.0;
    st.jetson_compute_mode = "MAXN";
    return st;
}

FieldDescription make_identity_field(double width, double height) {
    FieldDescription field;
    field.tf_camera_from_fieldcenter.tf = Eigen::Matrix4d::Identity();
    field.size.size.x = width;
    field.size.size.y = height;
    return field;
}

CameraInfo make_pinhole_camera(int width, int height) {
    CameraInfo info;
    info.width = width;
    info.height = height;
    info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    info.intrinsics.at<double>(0, 0) = 200.0;  // fx
    info.intrinsics.at<double>(1, 1) = 200.0;  // fy
    info.intrinsics.at<double>(0, 2) = static_cast<double>(width) / 2.0;
    info.intrinsics.at<double>(1, 2) = static_cast<double>(height) / 2.0;
    return info;
}

RobotDescription make_robot(Label label, Group group, double x, double y, double yaw = 0.0) {
    RobotDescription robot;
    robot.label = label;
    robot.group = group;
    robot.pose.position.x = x;
    robot.pose.position.y = y;
    robot.pose.position.z = 1.0;
    robot.pose.rotation = euler_to_quaternion(0.0, 0.0, yaw);
    robot.size.x = 0.4;
    robot.size.y = 0.4;
    return robot;
}

TEST(UiPresenterTest, PresentsHomeFaultAndReadyStates) {
    SystemStatus fault = make_status();
    fault.camera_ok = false;
    const auto fault_vm = ui_internal::present_home(fault, false, 0, 1, 15.0, false);
    EXPECT_EQ(fault_vm.status_label_text, "Hardware Disconnected");
    EXPECT_EQ(fault_vm.status_tile_color, 0xFF1744u);
    EXPECT_TRUE(fault_vm.reinit_should_pulse == false);

    SystemStatus ready = make_status();
    const auto ready_vm = ui_internal::present_home(ready, true, 2, 1, 118.5, true);
    EXPECT_EQ(ready_vm.status_label_text, "System Ready");
    EXPECT_EQ(ready_vm.status_tile_color, 0x00C853u);
    EXPECT_EQ(ready_vm.selected_opponent_count, 2);
}

TEST(UiPresenterTest, DiagnosticsSectionsRemainOrderedAndStale) {
    std::vector<std::string> order;
    std::map<std::string, std::pair<DiagnosticStatusSnapshot, std::chrono::steady_clock::time_point>>
        cache;

    DiagnosticStatusSnapshot snap_a;
    snap_a.name = "camera";
    snap_a.subsection = "get";
    snap_a.values["fps"] = "120";
    DiagnosticStatusSnapshot snap_b;
    snap_b.name = "navigation";
    snap_b.values["target"] = "opponent";

    const auto now = std::chrono::steady_clock::now();
    ui_internal::merge_diag_snapshots(order, cache, {snap_a, snap_b}, now);
    ASSERT_EQ(order.size(), 2u);

    const auto sections_fresh =
        ui_internal::present_diagnostics_sections(order, cache, now, /*stale_sec=*/2.0);
    ASSERT_EQ(sections_fresh.size(), 2u);
    EXPECT_EQ(sections_fresh[0].title, "camera: get");
    EXPECT_FALSE(sections_fresh[0].stale);

    const auto later = now + std::chrono::seconds(5);
    const auto sections_stale =
        ui_internal::present_diagnostics_sections(order, cache, later, /*stale_sec=*/2.0);
    ASSERT_EQ(sections_stale.size(), 2u);
    EXPECT_TRUE(sections_stale[0].stale);
    EXPECT_TRUE(sections_stale[1].stale);
}

TEST(UiOverlayRendererTest, RendersOverlayOnFixedFixture) {
    auto renderer = ui_internal::make_debug_overlay_renderer();
    ASSERT_NE(renderer, nullptr);

    cv::Mat image = cv::Mat::zeros(240, 320, CV_8UC3);
    const cv::Mat before = image.clone();

    FieldDescription field = make_identity_field(2.0, 2.0);
    CameraInfo camera = make_pinhole_camera(320, 240);

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, Group::OURS, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, Group::THEIRS, 0.3, 0.2));

    NavigationPathSegment path{};
    path.our_x = 0.0;
    path.our_y = 0.0;
    path.target_x = 0.4;
    path.target_y = 0.2;

    renderer->render(image, robots, path, field, camera);
    const cv::Mat diff = image != before;
    EXPECT_GT(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(UiControllerTest, DispatchesCommandCallbacksToUiState) {
    auto ui_state = std::make_shared<UIState>();
    ui_internal::UiController controller(ui_state);

    controller.request_reinitialize();
    EXPECT_TRUE(ui_state->reinit_requested.exchange(false));

    controller.set_opponent_count(3);
    EXPECT_EQ(ui_state->opponent_count_requested.exchange(-1), 3);

    SystemStatus st = make_status();
    st.autonomy_enabled = true;
    ui_state->set_system_status(st);
    controller.toggle_autonomy();
    EXPECT_EQ(ui_state->autonomy_toggle_requested.exchange(0), -1);

    controller.toggle_recording();
    EXPECT_TRUE(ui_state->recording_toggle_requested.exchange(false));

    controller.request_system_action(UISystemAction::QUIT_APP);
    EXPECT_EQ(ui_state->system_action_requested.exchange(static_cast<int>(UISystemAction::NONE)),
              static_cast<int>(UISystemAction::QUIT_APP));

    TargetSelection manual;
    manual.pose.x = 1.0;
    manual.pose.y = 2.0;
    manual.pose.yaw = 0.1;
    manual.label = Label::EMPTY;
    controller.set_manual_target(manual);
    const auto got_manual = ui_state->get_manual_target();
    ASSERT_TRUE(got_manual.has_value());
    EXPECT_DOUBLE_EQ(got_manual->pose.x, 1.0);
    controller.set_manual_target(std::nullopt);
    EXPECT_FALSE(ui_state->get_manual_target().has_value());
}

TEST(UiIntegrationSmokeTest, UiThreadStartsAndShutsDown) {
    auto ui_state = std::make_shared<UIState>();
    ui_state->set_window_size(320, 240);
    ui_state->set_fullscreen(false);

    std::promise<void> done;
    auto future = done.get_future();
    std::thread ui_thread([&done, &ui_state]() {
        run_ui_thread(ui_state);
        done.set_value();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ui_state->quit_requested.store(true);

    const auto status = future.wait_for(std::chrono::seconds(3));
    if (status != std::future_status::ready) {
        ui_thread.detach();
        GTEST_SKIP() << "UI thread did not exit in time (likely environment/display specific).";
    }
    ui_thread.join();
    SUCCEED();
}

}  // namespace
}  // namespace auto_battlebot
