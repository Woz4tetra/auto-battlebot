#include "runner.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <opencv2/core.hpp>

#include "time_utils.hpp"

namespace auto_battlebot {
Runner::Runner(const RunnerConfiguration &runner_config,
               std::shared_ptr<RgbdCameraInterface> camera,
               std::shared_ptr<HealthLogger> health_logger,
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
      initialized_(false),
      autonomy_enabled_(runner_config_.autonomy_enabled_by_default),
      initial_field_description_(),
      diagnostics_logger_(DiagnosticsLogger::get_logger("runner")),
      health_logger_(std::move(health_logger)),
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
    if (health_logger_) {
        status.jetson_temperature_c = health_logger_->get_last_temp_c();
        status.jetson_compute_mode = health_logger_->get_last_compute_mode();
    }
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
    if (req == -1) return;
    if (req < 1 || req > 3) {
        spdlog::warn("Requested number of opponents is not between 1 and 3 ({}). Ignoring.", req);
        return;
    }

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

void Runner::set_ui_debug_image_from_camera(const CameraData &camera_data) const {
    if (!ui_state_) return;
    if (!camera_data.rgb.image.data || camera_data.rgb.image.empty()) return;

    const cv::Mat &img = camera_data.rgb.image;
    std::vector<uint8_t> data(img.ptr<uint8_t>(),
                              img.ptr<uint8_t>() + img.total() * img.elemSize());
    ui_state_->set_debug_image(img.cols, img.rows, img.channels(), data);
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
    while (is_running()) {
        if (ui_state_ && !handle_system_action_request()) {
            camera_->cancel_initialize();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!is_running()) {
            camera_->cancel_initialize();
            break;
        }
        if (camera_->initialize()) break;

        spdlog::warn("Camera reinitialize attempt failed. Exiting.");
        return false;
    }

    if (!is_running()) {
        if (!miniros::ok()) miniros::shutdown();
        return false;
    }
    return true;
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
    initialized_ = true;
    spdlog::info("Field initialized");
}

TargetSelection Runner::resolve_target(const RobotDescriptionsStamped &robots,
                                       const FieldDescription &field_description) {
    if (ui_state_) {
        if (auto manual_target = ui_state_->get_manual_target()) {
            return *manual_target;
        }
    }
    if (target_selector_) {
        if (auto selected = target_selector_->get_target(robots, field_description)) {
            previous_selected_target_ = *selected;
        }
    }
    return previous_selected_target_;
}

int Runner::run() {
    auto loop_duration =
        std::chrono::microseconds(static_cast<int64_t>(1000000.0 / runner_config_.max_loop_rate));
    auto prev_time = std::chrono::steady_clock::now();

    while (true) {
        auto current_time = std::chrono::steady_clock::now();

        // Sleep until next tick to maintain loop rate
        auto remaining_time = loop_duration - (current_time - prev_time);
        prev_time = current_time;
        if (remaining_time.count() < 0) {
            diagnostics_logger_->debug("", {{"loop_duration_exceeded_ms", -to_ms(remaining_time)}});
        }
        std::this_thread::sleep_for(remaining_time);

        const auto tick_start = std::chrono::steady_clock::now();
        if (!tick()) {
            spdlog::warn("Runner::tick requested shutdown; runner loop exiting.");
            return 0;
        }
        health_logger_->record_tick(ms_since(tick_start));
        health_logger_->maybe_log();

        DiagnosticsLogger::publish();
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

    TargetSelection resolved_target = resolve_target(robots, field_description);

    VelocityCommand command = navigation_->update(robots, field_description, resolved_target);
    transmitter_->send(command);

    {
        double now_s = auto_battlebot::now();
        double pipeline_latency_ms = (now_s - robots.header.stamp) * 1000.0;
        diagnostics_logger_->debug("pipeline", {{"latency_ms", pipeline_latency_ms}});
    }

    publisher_->publish_navigation(navigation_->get_last_visualization());

    publish_system_status(true, loop_rate_hz);
    if (ui_state_) {
        ui_state_->set_camera_info(camera_data.camera_info);
        ui_state_->set_field_description(field_description);
        ui_state_->set_robots(robots);
        ui_state_->set_keypoints(keypoints);
        ui_state_->set_navigation_path(navigation_->get_last_visualization().path);
        set_ui_debug_image_from_camera(camera_data);
    }

    return true;
}

double Runner::elapsed_ms() {
    auto now = std::chrono::steady_clock::now();
    double elapsed = to_ms(now - start_time_);
    start_time_ = now;
    return elapsed;
}

}  // namespace auto_battlebot
