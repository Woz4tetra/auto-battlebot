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
               std::shared_ptr<ParallelModelBatch> perception_batch,
               std::shared_ptr<TransmitterInterface> transmitter,
               std::shared_ptr<ControlLoopInterface> control_loop,
               std::shared_ptr<PublisherInterface> publisher,
               SystemActionCallback system_action_callback,
               ProfileSelectCallback profile_select_callback, std::shared_ptr<UIState> ui_state,
               std::shared_ptr<McapRecorder> mcap_recorder, std::shared_ptr<ClockInterface> clock)
    : runner_config_(runner_config),
      camera_(camera),
      field_model_(field_model),
      robot_mask_model_(robot_mask_model),
      field_filter_(field_filter),
      keypoint_model_(keypoint_model),
      perception_batch_(std::move(perception_batch)),
      transmitter_(transmitter),
      control_loop_(std::move(control_loop)),
      publisher_(publisher),
      ui_state_(std::move(ui_state)),
      mcap_recorder_(std::move(mcap_recorder)),
      clock_(std::move(clock)),
      system_action_callback_(std::move(system_action_callback)),
      profile_select_callback_(std::move(profile_select_callback)),
      runtime_opponent_count_(runner_config_.default_opponent_count),
      robot_filter_reinit_pending_(false),
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
    status.transmitter_connected = control_loop_->loop().is_transmitter_connected();
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
        autonomy_enabled_ = true;
        control_loop_->loop().set_autonomy_enabled(true);
    } else if (autonomy_req == -1 && autonomy_enabled_) {
        autonomy_enabled_ = false;
        control_loop_->loop().set_autonomy_enabled(false);
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
    // Finalize recordings before the host reboots/powers off, otherwise the MCAP
    // writer is never closed and the file is left corrupted.
    stop_recordings_for_shutdown();
    if (system_action_callback_) {
        system_action_callback_(requested_action);
    }
    return true;
}

void Runner::handle_profile_switch_request() {
    auto requested_profile = ui_state_->take_requested_profile();
    if (!requested_profile) return;

    spdlog::info("Runner received profile switch request: {}", *requested_profile);
    if (profile_select_callback_) {
        profile_select_callback_(*requested_profile);
    }
    // The new profile only takes effect on the next launch; tell the user to reboot.
    ui_state_->set_profile_notice("Selected " + *requested_profile + ". Reboot to apply.");
}

void Runner::set_ui_debug_image_from_camera(const CameraData &camera_data) const {
    if (!ui_state_) return;
    if (!camera_data.rgb.image.data || camera_data.rgb.image.empty()) return;

    // UIState clones internally to detach from the camera SDK's reusable buffer.
    ui_state_->set_debug_image(camera_data.rgb.image);
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
    handle_profile_switch_request();
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
        if (quit_requested_.load()) return false;
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
    control_loop_->loop().set_autonomy_enabled(autonomy_enabled_);
    // Starts the thread for threaded drivers; a no-op for stepped ones. Everything the control
    // loop touches must be constructed by now, since it may begin cycling immediately.
    control_loop_->start();
    diagnostics_logger_->debug({}, "Initialization complete");
    DiagnosticsLogger::publish();
}

void Runner::initialize_field(const CameraData &camera_data) {
    spdlog::info("Initializing field");
    field_filter_->reset(camera_data.tf_visodom_from_camera);
    MaskStamped field_mask = field_model_->update(camera_data.rgb);

    // Check before publishing. Publishing first put an empty /field_mask in the recording on
    // every failed init, so a run whose field model never loaded looked like one that ran and
    // found nothing, and the missing /tf_static read as a publisher problem instead.
    if (field_mask.mask.mask.empty()) {
        spdlog::error(
            "Field model returned an empty mask; skipping field initialization. The model "
            "usually failed to load at startup, so check for earlier engine errors.");
        return;
    }

    publisher_->publish_field_mask(field_mask, camera_data.rgb, camera_data.camera_info);

    initial_field_description_ = field_filter_->compute_field(camera_data, field_mask);
    if (initial_field_description_->header.frame_id == FrameId::EMPTY) {
        spdlog::error("Failed to find a plane.");
        return;
    }
    publisher_->publish_initial_field_description(*initial_field_description_);

    control_loop_->loop().request_filter_reinit(runtime_opponent_count_);
    initialized_ = true;
    spdlog::info("Field initialized");
}

int Runner::run() {
    // max_loop_rate <= 0 means free-run: no wall-clock pacing, so the loop runs as fast as it can.
    // In the headless sim that is lockstep with the sim server's responses.
    const bool paced = runner_config_.max_loop_rate > 0.0;
    const auto loop_duration = paced ? std::chrono::microseconds(static_cast<int64_t>(
                                           1000000.0 / runner_config_.max_loop_rate))
                                     : std::chrono::microseconds(0);
    auto prev_time = std::chrono::steady_clock::now();

    while (true) {
        auto current_time = std::chrono::steady_clock::now();

        // Sleep until next tick to maintain loop rate (skipped when free-running).
        if (paced) {
            auto remaining_time = loop_duration - (current_time - prev_time);
            if (remaining_time.count() < 0) {
                diagnostics_logger_->debug("",
                                           {{"loop_duration_exceeded_ms", -to_ms(remaining_time)}});
            }
            std::this_thread::sleep_for(remaining_time);
        }
        prev_time = current_time;

        const auto tick_start = std::chrono::steady_clock::now();
        if (!tick()) {
            spdlog::warn("Runner::tick requested shutdown; runner loop exiting.");
            control_loop_->stop();
            return 0;
        }
        if (!control_loop_->is_healthy()) {
            // A stalled control loop leaves the robot executing its last command. Cut autonomy
            // rather than trusting a loop that has missed its deadline.
            spdlog::error("Control loop missed its watchdog deadline; disabling autonomy.");
            autonomy_enabled_ = false;
            control_loop_->loop().set_autonomy_enabled(false);
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

    if (quit_requested_.load()) {
        spdlog::warn("Quit requested via signal; shutting down runner.");
        stop_recordings_for_shutdown();
        return false;
    }

    if (!miniros::ok()) {
        spdlog::warn("miniros reported not ok; shutting down runner.");
        miniros::shutdown();
        return false;
    }

    bool should_reinit_field = false;
    if (!handle_ui_requests(should_reinit_field)) {
        return false;
    }

    // Stepped drivers read the transmitter here, on this thread, so the ordering matches the
    // pre-Phase-2 tick exactly. Threaded drivers own it and make this a no-op, latching the
    // init-button edge for take_init_button_press() to hand back.
    control_loop_->pump_input();
    should_reinit_field = should_reinit_field || control_loop_->loop().take_init_button_press();

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

    // Drive logical time from the frame stamp: control dt and message stamps come from this single
    // source, so sim/playback runs are deterministic and correct regardless of wall-clock speed.
    // (Pipeline latency below intentionally stays on wall-clock.)
    if (clock_) {
        clock_->set(camera_data.rgb.header.stamp);
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
        control_loop_->loop().request_filter_reinit(runtime_opponent_count_);
    }

    if (!initialized_) return handle_uninitialized_tick(camera_data, loop_rate_hz);

    FieldDescription field_description;
    {
        FunctionTimer timer(diagnostics_logger_, "field_filter.track_field");
        field_description = field_filter_->track_field(camera_data.tf_visodom_from_camera,
                                                       initial_field_description_);
    }

    KeypointsStamped keypoints;
    KeypointsStamped robot_blob_keypoints;
    if (runner_config_.parallel_models) {
        FunctionTimer timer(diagnostics_logger_, "perception_batch.update");
        BatchResult batch = perception_batch_->update(camera_data.rgb);
        keypoints = std::move(batch.keypoints);
        robot_blob_keypoints = std::move(batch.robot_blob_keypoints);
        // Per-model wall times are measured inside the workers and re-emitted here under
        // the sequential-era labels so latency reports stay comparable across versions.
        DiagnosticsData keypoint_timing;
        keypoint_timing["elapsed_ms"] = batch.keypoint_model_elapsed_ms;
        diagnostics_logger_->info("keypoint_model.update", keypoint_timing, "");
        DiagnosticsData robot_blob_timing;
        robot_blob_timing["elapsed_ms"] = batch.robot_blob_model_elapsed_ms;
        diagnostics_logger_->info("robot_mask_model.update", robot_blob_timing, "");
    } else {
        {
            FunctionTimer timer(diagnostics_logger_, "keypoint_model.update");
            keypoints = keypoint_model_->update(camera_data.rgb);
        }
        {
            FunctionTimer timer(diagnostics_logger_, "robot_mask_model.update");
            robot_blob_keypoints = robot_mask_model_->update(camera_data.rgb);
        }
    }

    // Hand perception to the control loop and let the driver decide when cycles run. The stepped
    // driver runs them inline here; the threaded driver has been consuming measurements on its own
    // thread all along and ignores advance_to.
    control_loop_->loop().submit_measurement(ControlMeasurement{
        .keypoints = keypoints,
        .robot_blob_keypoints = robot_blob_keypoints,
        .field_description = field_description,
        .camera_info = camera_data.camera_info,
    });
    {
        FunctionTimer timer(diagnostics_logger_, "control_loop.advance");
        control_loop_->advance_to(camera_data.rgb.header.stamp);
    }

    const ControlOutput control_output = control_loop_->loop().latest_output();
    const RobotDescriptionsStamped &robots = control_output.robots;

    {
        // Measure end-to-end latency from when the image was sampled (camera frame timestamp)
        // rather than from `robots.header.stamp`, which gets reused across cache substitutions
        // and so under-reports latency on substituted ticks.
        const double pipeline_latency_ms =
            (auto_battlebot::now() - camera_data.rgb.header.stamp) * 1000.0;
        diagnostics_logger_->debug("pipeline", {{"latency_ms", pipeline_latency_ms}});
    }

    // All publishing runs after the command send so none of it (notably the ~10 ms image
    // compression on Jetson) sits on the control critical path.
    {
        FunctionTimer timer(diagnostics_logger_, "publishers");
        publisher_->publish_camera_data(camera_data);
        publisher_->publish_field_description(field_description, *initial_field_description_);
        publisher_->publish_robots(robots);
        publisher_->publish_blob_detections(robot_mask_model_->last_detections());
        publisher_->publish_keypoint_detections(keypoint_model_->last_detections());
        publisher_->publish_navigation(control_loop_->loop().last_visualization());
    }

    publish_system_status(true, loop_rate_hz);
    if (ui_state_) {
        ui_state_->set_camera_info(camera_data.camera_info);
        ui_state_->set_field_description(field_description);
        ui_state_->set_robots(robots);
        ui_state_->set_keypoints(keypoints);
        ui_state_->set_navigation_path(control_loop_->loop().last_visualization().path);
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
