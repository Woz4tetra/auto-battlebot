#include "runner.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <opencv2/core.hpp>

#include "time_utils.hpp"

namespace auto_battlebot {
namespace {
constexpr double kTickWarnMs = 250.0;
constexpr double kHealthLogWarnMs = 150.0;
constexpr double kDiagnosticsPublishWarnMs = 100.0;
constexpr double kRecoverLoopWarnMs = 5000.0;
constexpr double kUiDebugCopyWarnMs = 40.0;
constexpr auto kWatchdogCheckInterval = std::chrono::seconds(3);
// Stall threshold must stay larger than the camera layer's own hard timeouts
// (kOpenHardTimeout = 30 s in zed_rgbd_camera.cpp) so a legitimately slow ZED reopen
// during recover_camera_after_failure() does not look like a wedged main loop.
constexpr auto kWatchdogStallThreshold = std::chrono::seconds(45);
constexpr auto kCameraRecoverInitialBackoff = std::chrono::milliseconds(1000);
constexpr auto kCameraRecoverMaxBackoff = std::chrono::milliseconds(10000);
constexpr auto kCameraRecoverPetInterval = std::chrono::milliseconds(500);
}  // namespace

Runner::Runner(const RunnerConfiguration &runner_config,
               std::shared_ptr<RgbdCameraInterface> camera,
               const HealthConfiguration &health_config,
               std::shared_ptr<MaskModelInterface> field_model,
               std::shared_ptr<RobotBlobModelInterface> robot_mask_model,
               std::shared_ptr<FieldFilterInterface> field_filter,
               std::shared_ptr<KeypointModelInterface> keypoint_model,
               std::shared_ptr<RobotFilterInterface> robot_filter,
               std::shared_ptr<TargetSelectorInterface> target_selector,
               std::shared_ptr<NavigationInterface> navigation,
               std::shared_ptr<TransmitterInterface> transmitter,
               std::shared_ptr<PublisherInterface> publisher,
               SystemActionCallback system_action_callback, std::shared_ptr<UIState> ui_state,
               std::shared_ptr<McapRecorder> mcap_recorder)
    : runner_config_(runner_config),
      health_config_(health_config),
      camera_(camera),
      field_model_(field_model),
      robot_mask_model_(robot_mask_model),
      field_filter_(field_filter),
      keypoint_model_(keypoint_model),
      robot_filter_(robot_filter),
      target_selector_(target_selector),
      navigation_(navigation),
      transmitter_(transmitter),
      publisher_(publisher),
      ui_state_(std::move(ui_state)),
      mcap_recorder_(std::move(mcap_recorder)),
      system_action_callback_(std::move(system_action_callback)),
      runtime_opponent_count_(runner_config_.default_opponent_count),
      robot_filter_reinit_pending_(false),
      previous_selected_target_(TargetSelection{}),
      previous_navigation_robots_(),
      has_previous_navigation_robots_(false),
      initialized_(false),
      autonomy_enabled_(runner_config_.autonomy_enabled_by_default),
      initial_field_description_(),
      diagnostics_logger_(DiagnosticsLogger::get_logger("runner")),
      health_logger_(std::make_unique<HealthLogger>(health_config_)),
      start_time_(std::chrono::steady_clock::now()) {}

void Runner::publish_system_status(bool camera_ok, double loop_rate_hz) const {
    if (!ui_state_) return;
    const bool svo_recording_enabled = camera_->is_recording_enabled();
    const bool mcap_recording_enabled = mcap_recorder_ ? mcap_recorder_->is_enabled() : true;
    SystemStatus status;
    status.camera_ok = camera_ok;
    status.transmitter_connected = transmitter_->is_connected();
    status.loop_rate_hz = loop_rate_hz;
    status.initialized = initialized_;
    status.selected_opponent_count = runtime_opponent_count_;
    status.autonomy_enabled = autonomy_enabled_;
    status.svo_recording_enabled = svo_recording_enabled;
    status.mcap_recording_enabled = mcap_recording_enabled;
    status.recording_enabled = svo_recording_enabled && mcap_recording_enabled;
    ui_state_->set_system_status(status);
}

void Runner::stop_recordings_for_shutdown() const {
    if (!camera_->set_recording_enabled(false)) {
        spdlog::warn("Failed to disable SVO recording during shutdown.");
    }
    if (mcap_recorder_) {
        mcap_recorder_->set_enabled(false);
        mcap_recorder_->close();
    }
}

void Runner::handle_opponent_count_request() {
    int req = ui_state_->opponent_count_requested.exchange(-1);
    if (req < 1 || req > 3) return;

    runtime_opponent_count_ = req;
    robot_filter_reinit_pending_ = true;
}

void Runner::handle_autonomy_toggle_request() {
    int autonomy_req = ui_state_->autonomy_toggle_requested.exchange(0);
    if (autonomy_req == 1 && !autonomy_enabled_) {
        transmitter_->enable();
        autonomy_enabled_ = true;
    } else if (autonomy_req == -1 && autonomy_enabled_) {
        transmitter_->disable();
        autonomy_enabled_ = false;
    }
}

void Runner::handle_recording_toggle_request() const {
    if (!ui_state_->recording_toggle_requested.exchange(false)) return;

    const bool svo_enabled = camera_->is_recording_enabled();
    const bool mcap_enabled = mcap_recorder_ ? mcap_recorder_->is_enabled() : true;
    const bool target_enabled = !(svo_enabled && mcap_enabled);

    if (!camera_->set_recording_enabled(target_enabled)) {
        spdlog::warn("Failed to set SVO recording to {}", target_enabled ? "enabled" : "disabled");
    }
    if (mcap_recorder_ && !mcap_recorder_->set_enabled(target_enabled)) {
        spdlog::warn("Failed to set MCAP recording to {}", target_enabled ? "enabled" : "disabled");
    }
}

bool Runner::handle_system_action_request() {
    int raw_action =
        ui_state_->system_action_requested.exchange(static_cast<int>(UISystemAction::NONE));
    auto requested_action = static_cast<UISystemAction>(raw_action);
    if (requested_action == UISystemAction::NONE) return true;

    spdlog::warn("Runner received system action request: {}", raw_action);
    if (system_action_callback_) {
        system_action_callback_(requested_action);
    }
    return true;
}

bool Runner::handle_ui_requests(bool &should_reinit_field) {
    if (!ui_state_) return true;

    if (ui_state_->quit_requested.load()) {
        spdlog::warn("UI requested quit via UIState::quit_requested.");
        stop_recordings_for_shutdown();
        return false;
    }

    should_reinit_field = ui_state_->reinit_requested.exchange(false);
    handle_opponent_count_request();
    handle_autonomy_toggle_request();
    handle_recording_toggle_request();
    return handle_system_action_request();
}

bool Runner::recover_camera_after_failure() {
    if (camera_->should_close()) {
        spdlog::error("Camera signalled to close the application");
        return false;
    }

    spdlog::error("Failed to get camera data. Reinitializing.");
    auto is_running = [this]() {
        if (!miniros::ok()) return false;
        if (ui_state_ && ui_state_->quit_requested.load()) return false;
        return true;
    };

    // Sleep with periodic watchdog pets so a long back-off does not get mistaken
    // for a stalled main loop. Returns false if the run state goes away during the sleep.
    auto pet_sleep = [&](std::chrono::milliseconds total) {
        const auto deadline = std::chrono::steady_clock::now() + total;
        while (std::chrono::steady_clock::now() < deadline) {
            if (!is_running()) return false;
            const auto remaining = deadline - std::chrono::steady_clock::now();
            const auto step =
                std::min<std::chrono::steady_clock::duration>(kCameraRecoverPetInterval, remaining);
            if (step.count() <= 0) break;
            std::this_thread::sleep_for(step);
            pet_watchdog();
        }
        return true;
    };

    auto backoff = kCameraRecoverInitialBackoff;
    while (is_running()) {
        const auto iteration_start = std::chrono::steady_clock::now();
        pet_watchdog();
        if (ui_state_ && !handle_system_action_request()) {
            camera_->cancel_initialize();
            return false;
        }
        if (!pet_sleep(std::chrono::milliseconds(100))) {
            camera_->cancel_initialize();
            break;
        }
        if (!is_running()) {
            camera_->cancel_initialize();
            break;
        }
        pet_watchdog();
        if (camera_->initialize()) {
            pet_watchdog();
            break;
        }
        pet_watchdog();

        const double iteration_ms = std::chrono::duration<double, std::milli>(
                                        std::chrono::steady_clock::now() - iteration_start)
                                        .count();
        if (iteration_ms > kRecoverLoopWarnMs) {
            spdlog::warn("validation: camera_recover_iteration slow elapsed_ms={:.2f}",
                         iteration_ms);
        }

        spdlog::warn("Camera reinitialize attempt failed; backing off {} ms before retry",
                     backoff.count());
        if (!pet_sleep(backoff)) {
            camera_->cancel_initialize();
            break;
        }
        backoff = std::min(backoff * 2, kCameraRecoverMaxBackoff);
    }

    if (!is_running()) {
        if (!miniros::ok()) miniros::shutdown();
        return false;
    }
    return true;
}

void Runner::set_ui_debug_image_from_camera(const CameraData &camera_data) const {
    if (!ui_state_) return;
    if (!camera_data.rgb.image.data || camera_data.rgb.image.empty()) return;

    const auto copy_start = std::chrono::steady_clock::now();
    const cv::Mat &img = camera_data.rgb.image;
    std::vector<uint8_t> data(img.ptr<uint8_t>(),
                              img.ptr<uint8_t>() + img.total() * img.elemSize());
    ui_state_->set_debug_image(img.cols, img.rows, img.channels(), data);
    const double copy_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - copy_start)
            .count();
    if (copy_ms > kUiDebugCopyWarnMs) {
        spdlog::warn("validation: set_ui_debug_image slow elapsed_ms={:.2f} bytes={}", copy_ms,
                     data.size());
    }
}

bool Runner::handle_uninitialized_tick(const CameraData &camera_data, double loop_rate_hz) {
    publisher_->publish_camera_data(camera_data);
    if (ui_state_) {
        ui_state_->set_camera_info(camera_data.camera_info);
        ui_state_->set_field_description(std::nullopt);
        set_ui_debug_image_from_camera(camera_data);
    }
    publish_system_status(true, loop_rate_hz);
    return true;
}

void Runner::initialize() {
    // Initialize all interfaces
    if (!camera_->initialize()) {
        spdlog::error("Failed to initialize camera");
    }
    if (!field_model_->initialize()) {
        spdlog::error("Failed to initialize field model");
    }
    if (!robot_mask_model_->initialize()) {
        spdlog::error("Failed to initialize robot blob model");
    }
    if (!keypoint_model_->initialize()) {
        spdlog::error("Failed to initialize keypoint model.");
    }
    if (!transmitter_->initialize()) {
        spdlog::error("Failed to initialize transmitter");
    }
    if (autonomy_enabled_) {
        transmitter_->enable();
    } else {
        transmitter_->disable();
    }
    diagnostics_logger_->debug({}, "Initialization complete");
    DiagnosticsLogger::publish();
}

void Runner::initialize_field(const CameraData &camera_data) {
    spdlog::info("Initializing field");
    field_filter_->reset(camera_data.tf_visodom_from_camera);
    MaskStamped field_mask = field_model_->update(camera_data.rgb);
    publisher_->publish_field_mask(field_mask, camera_data.rgb);

    if (field_mask.mask.mask.empty()) {
        spdlog::error("Field model returned an empty mask; skipping field initialization.");
        return;
    }

    initial_field_description_ = field_filter_->compute_field(camera_data, field_mask);
    if (initial_field_description_->header.frame_id == FrameId::EMPTY) {
        spdlog::error("Failed to find a plane.");
        return;
    }
    publisher_->publish_initial_field_description(*initial_field_description_);

    robot_filter_->initialize(runtime_opponent_count_);
    navigation_->initialize();
    previous_selected_target_ = TargetSelection{};
    has_previous_navigation_robots_ = false;
    previous_navigation_robots_.descriptions.clear();
    initialized_ = true;
    spdlog::info("Field initialized");
}

bool Runner::has_our_robot(const RobotDescriptionsStamped &robots) {
    for (const auto &robot : robots.descriptions) {
        if (robot.frame_id == FrameId::OUR_ROBOT_1) return true;
    }
    return false;
}

bool Runner::has_their_robot(const RobotDescriptionsStamped &robots) {
    for (const auto &robot : robots.descriptions) {
        if (robot.group == Group::THEIRS) return true;
    }
    return false;
}

bool Runner::has_navigation_critical_robots(const RobotDescriptionsStamped &robots) {
    return has_our_robot(robots) && has_their_robot(robots);
}

int Runner::run() {
    auto loop_duration =
        std::chrono::microseconds(static_cast<int64_t>(1000000.0 / runner_config_.max_loop_rate));
    auto prev_time = std::chrono::steady_clock::now();
    auto heartbeat_window_start = std::chrono::steady_clock::now();
    uint64_t heartbeat_ticks = 0;
    double max_tick_ms = 0.0;
    double max_health_ms = 0.0;
    double max_publish_ms = 0.0;

    watchdog_stop_ = false;
    pet_watchdog();
    watchdog_thread_ = std::thread([this]() {
        while (!watchdog_stop_) {
            std::this_thread::sleep_for(kWatchdogCheckInterval);
            if (watchdog_stop_) break;
            const int64_t now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
            const auto stall = std::chrono::nanoseconds(now_ns - last_tick_ns_.load());
            if (stall > kWatchdogStallThreshold) {
                spdlog::critical("Watchdog: main loop stalled for {:.1f}s, aborting",
                                 std::chrono::duration<double>(stall).count());
                spdlog::default_logger()->flush();
                std::abort();
            }
        }
    });

    while (true) {
        auto current_time = std::chrono::steady_clock::now();

        // Sleep until next tick to maintain loop rate
        auto remaining_time = loop_duration - (current_time - prev_time);
        prev_time = current_time;
        if (remaining_time.count() < 0) {
            double exceeded_time =
                -1 * std::chrono::duration_cast<std::chrono::microseconds>(remaining_time).count() /
                1000.0;
            diagnostics_logger_->debug("", {{"loop_duration_exceeded_ms", exceeded_time}});
        }
        std::this_thread::sleep_for(remaining_time);

        pet_watchdog();

        const auto tick_start = std::chrono::steady_clock::now();
        if (!tick()) {
            spdlog::warn("Runner::tick requested shutdown; runner loop exiting.");
            watchdog_stop_ = true;
            watchdog_thread_.join();
            return 0;
        }
        const double tick_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tick_start)
                .count();
        max_tick_ms = std::max(max_tick_ms, tick_ms);
        if (tick_ms > kTickWarnMs) {
            spdlog::warn("validation: tick slow elapsed_ms={:.2f}", tick_ms);
        }

        if (health_logger_) {
            const auto health_start = std::chrono::steady_clock::now();
            health_logger_->maybe_log();
            const double health_ms = std::chrono::duration<double, std::milli>(
                                         std::chrono::steady_clock::now() - health_start)
                                         .count();
            max_health_ms = std::max(max_health_ms, health_ms);
            if (health_ms > kHealthLogWarnMs) {
                spdlog::warn("validation: health_logger.maybe_log slow elapsed_ms={:.2f}",
                             health_ms);
            }
        }
        const auto publish_start = std::chrono::steady_clock::now();
        DiagnosticsLogger::publish();
        const double publish_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - publish_start)
                                      .count();
        max_publish_ms = std::max(max_publish_ms, publish_ms);
        if (publish_ms > kDiagnosticsPublishWarnMs) {
            spdlog::warn("validation: DiagnosticsLogger::publish slow elapsed_ms={:.2f}",
                         publish_ms);
        }

        heartbeat_ticks++;
        const auto now = std::chrono::steady_clock::now();
        const auto heartbeat_elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - heartbeat_window_start)
                .count();
        if (heartbeat_elapsed >= 1000) {
            spdlog::debug(
                "runner heartbeat: ticks={} tick_ms_max={:.2f} health_ms_max={:.2f} "
                "publish_ms_max={:.2f}",
                heartbeat_ticks, max_tick_ms, max_health_ms, max_publish_ms);
            heartbeat_window_start = now;
            heartbeat_ticks = 0;
            max_tick_ms = 0.0;
            max_health_ms = 0.0;
            max_publish_ms = 0.0;
        }
    }
}

bool Runner::tick() {
    FunctionTimer timer(diagnostics_logger_, "tick");
    diagnostics_logger_->debug({}, "Tick");

    double period_ms = elapsed_ms();
    double loop_rate_hz = (period_ms > 0.0) ? (1000.0 / period_ms) : 0.0;

    DiagnosticsData rate_data;
    rate_data["rate"] = loop_rate_hz;
    diagnostics_logger_->debug(rate_data);

    if (!miniros::ok()) {
        spdlog::warn("miniros reported not ok; shutting down runner.");
        miniros::shutdown();
        return false;
    }

    bool should_reinit_field = false;
    if (!handle_ui_requests(should_reinit_field)) {
        return false;
    }

    CommandFeedback command_feedback = transmitter_->update();
    should_reinit_field = should_reinit_field || transmitter_->did_init_button_press();

    CameraData camera_data;
    bool is_camera_ok;
    {
        FunctionTimer timer(diagnostics_logger_, "camera.get");
        is_camera_ok = camera_->get(camera_data, should_reinit_field);
    }

    if (!is_camera_ok) {
        publish_system_status(false, loop_rate_hz);
        if (ui_state_) {
            ui_state_->set_field_description(std::nullopt);
        }
        return recover_camera_after_failure();
    }

    if (should_reinit_field) {
        if (camera_data.tracking_ok) {
            robot_filter_reinit_pending_ = false;
            initialize_field(camera_data);
        } else {
            spdlog::warn("Skipping field initialization because camera tracking is not ready.");
        }
    } else if (robot_filter_reinit_pending_ && initialized_) {
        robot_filter_reinit_pending_ = false;
        robot_filter_->initialize(runtime_opponent_count_);
        navigation_->initialize();
        has_previous_navigation_robots_ = false;
        previous_navigation_robots_.descriptions.clear();
    }

    if (!initialized_) return handle_uninitialized_tick(camera_data, loop_rate_hz);

    FieldDescription field_description;
    {
        FunctionTimer timer(diagnostics_logger_, "field_filter.track_field");
        field_description = field_filter_->track_field(camera_data.tf_visodom_from_camera,
                                                       initial_field_description_);
    }

    KeypointsStamped keypoints;
    {
        FunctionTimer timer(diagnostics_logger_, "keypoint_model.update");
        keypoints = keypoint_model_->update(camera_data.rgb);
    }

    KeypointsStamped robot_blob_keypoints;
    {
        FunctionTimer timer(diagnostics_logger_, "robot_mask_model.update");
        robot_blob_keypoints = robot_mask_model_->update(camera_data.rgb);
    }

    RobotDescriptionsStamped robots;
    {
        FunctionTimer timer(diagnostics_logger_, "robot_filter.update");
        robots = robot_filter_->update(keypoints, field_description, camera_data.camera_info,
                                       robot_blob_keypoints, command_feedback);
    }

    {
        FunctionTimer timer(diagnostics_logger_, "publishers");
        publisher_->publish_camera_data(camera_data);
        publisher_->publish_field_description(field_description, *initial_field_description_);
        publisher_->publish_robots(robots);
    }

    RobotDescriptionsStamped robots_for_navigation = robots;
    bool using_previous_navigation_robots = false;
    if (has_navigation_critical_robots(robots)) {
        previous_navigation_robots_ = robots;
        has_previous_navigation_robots_ = true;
    } else if (has_previous_navigation_robots_ &&
               has_navigation_critical_robots(previous_navigation_robots_)) {
        robots_for_navigation = previous_navigation_robots_;
        using_previous_navigation_robots = true;
    }

    TargetSelection resolved_target = previous_selected_target_;
    std::optional<TargetSelection> manual_target;
    if (ui_state_) manual_target = ui_state_->get_manual_target();
    if (manual_target.has_value()) {
        resolved_target = *manual_target;
    } else if (target_selector_) {
        std::optional<TargetSelection> selected =
            target_selector_->get_target(robots_for_navigation, field_description);
        if (selected.has_value()) {
            resolved_target = *selected;
            previous_selected_target_ = resolved_target;
        }
    }

    diagnostics_logger_->debug("navigation",
                               {{"using_previous_robots", (int)using_previous_navigation_robots}});

    VelocityCommand command =
        navigation_->update(robots_for_navigation, field_description, resolved_target);
    transmitter_->send(command);

    {
        double now_s = auto_battlebot::now();
        double pipeline_latency_ms = (now_s - robots.header.stamp) * 1000.0;
        diagnostics_logger_->debug("pipeline", {{"latency_ms", pipeline_latency_ms}});
    }

    {
        NavigationVisualization nav_viz;
        nav_viz.header = robots.header;
        nav_viz.path = navigation_->get_last_path();
        nav_viz.command = command;
        nav_viz.robots = robots;
        publisher_->publish_navigation(nav_viz);
    }

    publish_system_status(true, loop_rate_hz);
    if (ui_state_) {
        ui_state_->set_camera_info(camera_data.camera_info);
        ui_state_->set_field_description(field_description);
        ui_state_->set_robots(robots);
        ui_state_->set_keypoints(keypoints);
        ui_state_->set_navigation_path(navigation_->get_last_path());
        set_ui_debug_image_from_camera(camera_data);
    }

    return true;
}

double Runner::elapsed_ms() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
    start_time_ = now;
    return duration.count() / 1000.0;
}

void Runner::pet_watchdog() {
    last_tick_ns_ = std::chrono::steady_clock::now().time_since_epoch().count();
}

}  // namespace auto_battlebot
