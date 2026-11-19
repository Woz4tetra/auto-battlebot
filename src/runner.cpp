#include "runner.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <magic_enum.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>

#include "remote/field_projection.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {
Runner::Runner(
    const RunnerConfiguration &runner_config, std::shared_ptr<RgbdCameraInterface> camera,
    std::shared_ptr<HealthLogger> health_logger, std::shared_ptr<MaskModelInterface> field_model,
    std::shared_ptr<RobotBlobModelInterface> robot_mask_model,
    std::shared_ptr<FieldFilterInterface> field_filter,
    std::shared_ptr<KeypointModelInterface> keypoint_model,
    std::shared_ptr<KeypointHeightGate> height_gate,
    std::shared_ptr<StaticDetectionGate> static_gate,
    std::shared_ptr<ParallelModelBatch> perception_batch,
    std::shared_ptr<ControlLoopInterface> control_loop,
    std::shared_ptr<PublisherInterface> publisher, SystemActionCallback system_action_callback,
    ProfileSelectCallback profile_select_callback, std::shared_ptr<McapRecorder> mcap_recorder,
    std::shared_ptr<ClockInterface> clock, RunnerRemote remote)
    : runner_config_(runner_config),
      camera_(camera),
      field_model_(field_model),
      robot_mask_model_(robot_mask_model),
      field_filter_(field_filter),
      keypoint_model_(keypoint_model),
      height_gate_(std::move(height_gate)),
      static_gate_(std::move(static_gate)),
      perception_batch_(std::move(perception_batch)),
      control_loop_(std::move(control_loop)),
      publisher_(publisher),
      mcap_recorder_(std::move(mcap_recorder)),
      clock_(std::move(clock)),
      system_action_callback_(std::move(system_action_callback)),
      profile_select_callback_(std::move(profile_select_callback)),
      remote_(std::move(remote)),
      command_log_(mcap_recorder_),
      runtime_opponent_count_(runner_config_.default_opponent_count),
      initialized_(false),
      autonomy_enabled_(runner_config_.autonomy_enabled_by_default),
      behavior_mode_(runner_config_.default_behavior_mode),
      auto_tracking_(runner_config_.auto_tracking_by_default),
      initial_field_description_(),
      diagnostics_logger_(DiagnosticsLogger::get_logger("runner")),
      health_logger_(std::move(health_logger)),
      prev_tick_start_(std::chrono::steady_clock::now()),
      constructed_at_(prev_tick_start_) {
    remote_.commands->set_on_system_action([camera = std::weak_ptr<RgbdCameraInterface>(camera_)] {
        if (auto locked = camera.lock()) locked->cancel_initialize();
    });
}

Runner::~Runner() { remote_.commands->set_on_system_action(nullptr); }

static_assert(std::atomic<bool>::is_always_lock_free,
              "quit_from_signal_handler() runs in a signal handler and needs a lock-free store");

void Runner::quit_from_signal_handler() {
    quit_requested_.store(true);
    // A camera re-open can take 30 s. cancel_initialize() only stores an atomic, so this is
    // still safe from the signal handler.
    camera_->cancel_initialize();
}

void Runner::publish_system_status(bool camera_ok, double loop_rate_hz) {
    const TransmitterStatus transmitter = control_loop_->transmitter_status();
    remote::SystemStatusMessage message;
    message.camera_ok = camera_ok;
    message.transmitter_connected = transmitter.connected;
    message.transmitter_receiving = transmitter.receiving_channels;
    message.loop_rate_hz = loop_rate_hz;
    message.initialized = initialized_;
    message.selected_opponent_count = runtime_opponent_count_;
    message.autonomy_enabled = autonomy_enabled_;
    message.camera_recording = camera_->is_recording_enabled();
    message.mcap_recording = mcap_recorder_ ? mcap_recorder_->is_enabled() : true;
    if (health_logger_) {
        message.jetson_temperature_c = health_logger_->soc_temperature_c();
        message.compute_mode = health_logger_->get_last_compute_mode();
    }
    message.uptime_s = uptime_s();
    if (transmitter.has_trainer_autonomy_switch)
        message.autonomy_switch_on = transmitter.autonomy_switch_on;
    message.autonomy_on_s = autonomy_on_s();
    if (mcap_recorder_) message.mcap_file_s = mcap_recorder_->file_age_s();
    message.behavior_mode = behavior_mode_;
    message.auto_tracking = auto_tracking_;
    last_status_ = message;
    if (!remote_.status) return;
    remote_.status->publish(message);
    remote_.status->publish(remote_.app_info);
    remote_.status->publish(remote_.resolved_config);
    publish_camera_settings(false);
}

double Runner::uptime_s() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - constructed_at_)
        .count();
}

std::optional<double> Runner::autonomy_on_s() const {
    if (!autonomy_switch_on_since_) return std::nullopt;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         *autonomy_switch_on_since_)
        .count();
}

void Runner::update_timers() {
    const TransmitterStatus transmitter = control_loop_->transmitter_status();
    const bool on = transmitter.has_trainer_autonomy_switch && transmitter.autonomy_switch_on;
    if (on && !autonomy_switch_on_since_) {
        autonomy_switch_on_since_ = std::chrono::steady_clock::now();
    } else if (!on) {
        autonomy_switch_on_since_.reset();
    }
    diagnostics_logger_->debug(
        {{"uptime_s", uptime_s()}, {"autonomy_on_s", autonomy_on_s().value_or(0.0)}});
}

void Runner::publish_tracks(const RobotDescriptionsStamped &robots, const FieldDescription &field,
                            const CameraInfo &camera_info, const TargetSelection &target,
                            bool manual_target) const {
    if (!remote_.status) return;
    remote::TracksMessage message;
    message.field_x = field.size.size.x;
    message.field_y = field.size.size.y;
    message.field_outline = remote::project_field_outline(field, camera_info);
    message.camera_marker = remote::project_camera_marker(field, camera_info);
    for (const auto &robot : robots.descriptions) {
        if (!robot.is_stale) {
            if (robot.group == Group::OURS) message.our_robot_seen = true;
            if (robot.group == Group::THEIRS) ++message.opponents_seen;
        }
        const Rotation &q = robot.pose.rotation;
        remote::TrackedRobot tracked;
        tracked.id = remote::detail::lowercase(magic_enum::enum_name(robot.frame_id));
        tracked.label = remote::detail::lowercase(magic_enum::enum_name(robot.label));
        tracked.ours = robot.group == Group::OURS;
        tracked.stale = robot.is_stale;
        tracked.x = robot.pose.position.x;
        tracked.y = robot.pose.position.y;
        tracked.yaw =
            std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        tracked.image = remote::project_field_point(field, camera_info, tracked.x, tracked.y);
        message.robots.push_back(std::move(tracked));
    }
    const NavigationVisualization visualization = control_loop_->last_visualization();
    if (const auto &path = visualization.path) {
        remote::TargetMessage out;
        out.from_x = path->our_x;
        out.from_y = path->our_y;
        out.from_image = remote::project_field_point(field, camera_info, out.from_x, out.from_y);
        out.x = path->target_x;
        out.y = path->target_y;
        if (target.label != Label::EMPTY) {
            out.label = remote::detail::lowercase(magic_enum::enum_name(target.label));
        }
        out.mode = target.mode;
        out.image = remote::project_field_point(field, camera_info, out.x, out.y);
        out.manual = manual_target;
        message.target = std::move(out);
    }
    remote_.status->publish(message);
}

void Runner::stop_recordings_for_shutdown() const {
    recording_worker_->stop();
    std::lock_guard<std::mutex> lock(recording_mutex_);
    if (!camera_->set_recording_enabled(false)) {
        spdlog::warn("Failed to disable SVO recording during shutdown.");
    }
    if (mcap_recorder_) {
        publisher_->flush();
        spdlog::default_logger()->flush();
        mcap_recorder_->set_enabled(false);
        mcap_recorder_->close();
    }
}

void Runner::roll_over_recording_if_full() {
    constexpr auto kRecordingSizeCheckInterval = std::chrono::seconds(1);
    const auto now = std::chrono::steady_clock::now();
    if (now - last_size_check_ < kRecordingSizeCheckInterval) return;
    last_size_check_ = now;
    if (rollover_pending_.load()) return;
    if (mcap_recorder_ && mcap_recorder_->past_size_limit()) {
        spdlog::info("[Runner] {} reached the size cap; rolling over",
                     mcap_recorder_->file_path().string());
    } else if (camera_->recording_encoder_stalled()) {
        spdlog::error("[Runner] The camera's video encoder stopped; restarting the recording");
    } else {
        return;
    }
    rollover_pending_.store(true);
    recording_worker_->post([this] {
        apply_recording(false, true);
        apply_recording(true, true);
        rollover_pending_.store(false);
    });
}

void Runner::set_opponent_count(int count) {
    runtime_opponent_count_ = count;
    if (initialized_) control_loop_->request_filter_reinit(runtime_opponent_count_);
}

void Runner::set_autonomy(bool enabled) {
    if (autonomy_enabled_ == enabled) return;
    autonomy_enabled_ = enabled;
    control_loop_->set_autonomy_enabled(enabled);
}

void Runner::set_behavior_mode(BehaviorMode mode) {
    behavior_mode_ = mode;
    control_loop_->set_behavior_mode(mode);
}

void Runner::set_auto_tracking(bool enabled) {
    if (auto_tracking_ == enabled) return;
    auto_tracking_ = enabled;
    control_loop_->set_target_override(enabled ? std::nullopt : std::optional<Pose2D>(Pose2D{}));
}

std::string Runner::set_manual_target(const remote::SetManualTargetCommand &command,
                                      bool &accepted) {
    accepted = false;
    if (auto_tracking_) return "Turn auto tracking off to place the target";
    if (command.field.has_value() == command.image.has_value()) {
        return "Set exactly one of field or image";
    }
    remote::FieldPoint point;
    if (command.field) {
        point = *command.field;
    } else {
        if (!projection_field_) return "The field is not initialized";
        const auto hit = remote::unproject_image_point(*projection_field_, projection_camera_info_,
                                                       *command.image);
        if (!hit) return "That point is above the floor";
        point = *hit;
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return "Target is not finite";
    if (projection_field_) {
        const double hx = projection_field_->size.size.x / 2.0;
        const double hy = projection_field_->size.size.y / 2.0;
        if (hx > 0.0 && hy > 0.0) {
            point.x = std::clamp(point.x, -hx, hx);
            point.y = std::clamp(point.y, -hy, hy);
        }
    }
    accepted = true;
    control_loop_->set_target_override(Pose2D{.x = point.x, .y = point.y, .yaw = 0.0});
    return "";
}

void Runner::set_recording(bool enabled) const {
    recording_worker_->post([this, enabled] { apply_recording(enabled, true); });
}

void Runner::wait_for_frames_in_flight() const {
    // The tick keeps running while this thread toggles. A tick that took a frame before the
    // encoder stopped still holds that frame's video index, and would publish its frame_meta into
    // the next file, pointing at the wrong packet. Frames captured after the stop carry no index.
    // Three tick starts: the tick in flight finishes, and so does the one that may have taken
    // the last frame the camera buffered before the stop. Bounded, for a tick stuck in camera
    // recovery.
    constexpr uint64_t kTicks = 3;
    constexpr auto kTimeout = std::chrono::milliseconds(500);
    const uint64_t target = ticks_started_.load() + kTicks;
    const auto deadline = std::chrono::steady_clock::now() + kTimeout;
    while (ticks_started_.load() < target && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void Runner::apply_recording(bool enabled, bool from_worker) const {
    std::lock_guard<std::mutex> lock(recording_mutex_);
    const auto set_mcap = [this, enabled, from_worker] {
        if (!enabled && from_worker) wait_for_frames_in_flight();
        if (!enabled && mcap_recorder_) publisher_->flush();
        if (mcap_recorder_ && !mcap_recorder_->set_enabled(enabled)) {
            spdlog::warn("Failed to set MCAP recording to {}", enabled ? "enabled" : "disabled");
        }
    };
    if (enabled) set_mcap();
    if (!camera_->set_recording_enabled(enabled)) {
        spdlog::warn("Failed to set SVO recording to {}", enabled ? "enabled" : "disabled");
    }
    if (!enabled) set_mcap();
}

std::string Runner::select_profile(const std::string &name) {
    spdlog::info("Runner received profile switch request: {}", name);
    if (profile_select_callback_) profile_select_callback_(name);
    std::string notice = "Selected " + name + ". Restart the app or reboot to apply.";
    return notice;
}

void Runner::run_system_action(SystemAction action) {
    spdlog::warn("Runner received system action request: {}", magic_enum::enum_name(action));
    stop_recordings_for_shutdown();
    if (system_action_callback_) system_action_callback_(action);
}

std::string Runner::set_camera_settings(const CameraSettings &settings, bool &accepted) {
    if (!camera_->supports_camera_settings()) {
        accepted = false;
        return "This camera has no live controls";
    }
    camera_->request_camera_settings(settings);
    requested_camera_settings_ = settings;
    camera_settings_modified_ = true;
    publish_camera_settings(true);
    return "";
}

std::string Runner::run_camera_profile(CameraProfileAction action, const std::string &name,
                                       bool &accepted) {
    if (!camera_->supports_camera_settings()) {
        accepted = false;
        return "This camera has no live controls";
    }
    const bool is_default = name == CameraProfileStore::kDefaultName;
    std::string message;
    try {
        switch (action) {
            case CameraProfileAction::APPLY: {
                std::optional<CameraSettings> settings;
                if (!is_default) {
                    settings = remote_.camera_profiles ? remote_.camera_profiles->settings(name)
                                                       : std::nullopt;
                    if (!settings) throw std::runtime_error("No camera profile named " + name);
                }
                camera_->request_camera_settings(settings);
                requested_camera_settings_ = settings;
                active_camera_profile_ = name;
                camera_settings_modified_ = false;
                spdlog::info("Camera profile {} applied", name);
                break;
            }
            case CameraProfileAction::SAVE: {
                if (!remote_.camera_profiles) throw std::runtime_error("No profile store");
                CameraSettings settings;
                if (requested_camera_settings_) {
                    settings = *requested_camera_settings_;
                } else if (auto state = camera_->camera_settings()) {
                    settings = state->settings;
                } else {
                    throw std::runtime_error("Camera has not opened yet");
                }
                remote_.camera_profiles->save(name, settings);
                requested_camera_settings_ = settings;
                active_camera_profile_ = name;
                camera_settings_modified_ = false;
                message = "Saved " + name;
                spdlog::info("Camera profile {} saved to {}", name,
                             remote_.camera_profiles->path().string());
                break;
            }
            case CameraProfileAction::DELETE: {
                if (!remote_.camera_profiles) throw std::runtime_error("No profile store");
                remote_.camera_profiles->remove(name);
                message = "Deleted " + name;
                if (active_camera_profile_ == name) {
                    active_camera_profile_ = CameraProfileStore::kDefaultName;
                    camera_settings_modified_ = requested_camera_settings_.has_value();
                }
                break;
            }
        }
    } catch (const std::runtime_error &e) {
        accepted = false;
        message = e.what();
    }
    publish_camera_settings(true);
    return message;
}

void Runner::publish_camera_settings(bool now) {
    if (!remote_.status) return;
    const auto clock_now = std::chrono::steady_clock::now();
    if (!now && clock_now - last_camera_settings_publish_ < std::chrono::milliseconds(500)) return;
    last_camera_settings_publish_ = clock_now;
    remote_.status->publish(camera_settings_status());
}

remote::CameraSettingsMessage Runner::camera_settings_status() const {
    remote::CameraSettingsMessage message;
    message.supported = camera_->supports_camera_settings();
    message.profiles = remote_.camera_profiles
                           ? remote_.camera_profiles->names()
                           : std::vector<std::string>{CameraProfileStore::kDefaultName};
    message.active_profile = active_camera_profile_;
    message.modified = camera_settings_modified_;
    if (auto state = camera_->camera_settings()) {
        message.settings = state->settings;
        message.limits = state->limits;
    } else {
        message.supported = false;
    }
    return message;
}

void Runner::ack(std::string_view topic, bool accepted, std::string message) {
    if (!remote_.status) return;
    remote::CommandAckMessage out;
    out.seq = ++ack_seq_;
    out.topic = std::string(topic);
    out.accepted = accepted;
    out.message = std::move(message);
    remote_.status->publish(out);
}

bool Runner::handle_commands(bool &should_reinit_field) {
    bool keep_running = true;
    for (auto &command : remote_.commands->drain()) {
        const std::string_view topic = remote::command_topic_of(command);
        if (std::holds_alternative<remote::SetManualTargetCommand>(command)) {
            spdlog::debug("Command: {}", topic);
        } else {
            spdlog::info("Command: {}", topic);
        }
        command_log_.record(command);
        bool accepted = true;
        std::string message;
        std::visit(
            remote::overloaded{
                [&](const remote::ReinitFieldCommand &) { should_reinit_field = true; },
                [&](const remote::SetOpponentCountCommand &c) {
                    if (c.count < 1 || c.count > 3) {
                        accepted = false;
                        message = "Opponent count must be 1 to 3";
                        return;
                    }
                    set_opponent_count(c.count);
                },
                [&](const remote::SetAutonomyCommand &c) { set_autonomy(c.enabled); },
                [&](const remote::SetBehaviorModeCommand &c) { set_behavior_mode(c.mode); },
                [&](const remote::SetAutoTrackingCommand &c) { set_auto_tracking(c.enabled); },
                [&](const remote::SetManualTargetCommand &c) {
                    message = set_manual_target(c, accepted);
                },
                [&](const remote::SetRecordingCommand &c) { set_recording(c.enabled); },
                [&](const remote::SelectProfileCommand &c) { message = select_profile(c.name); },
                [&](const remote::SystemActionCommand &c) {
                    run_system_action(c.action);
                    keep_running = false;
                },
                [&](const remote::SetWifiAccessCommand &c) {
                    if (!remote_.host) {
                        accepted = false;
                        message = "No host services in this process";
                        return;
                    }
                    remote_.host->set_wifi_access(c.enabled);
                },
                [&](const remote::SetCameraSettingsCommand &c) {
                    message = set_camera_settings(c.settings, accepted);
                },
                [&](const remote::CameraProfileCommand &c) {
                    message = run_camera_profile(c.action, c.name, accepted);
                },
            },
            command);
        ack(topic, accepted, std::move(message));
        if (!keep_running) break;
    }
    return keep_running;
}

bool Runner::recover_camera_after_failure() {
    if (camera_->should_close()) {
        spdlog::error("Camera signalled to close the application");
        return false;
    }

    spdlog::error("Failed to get camera data. Reinitializing.");
    auto is_running = [this]() {
        if (quit_requested_.load()) return false;
        return true;
    };
    while (is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Commands and the quit check come after the sleep, right before the open: initialize()
        // clears the cancel flag, so a reboot posted during the sleep would otherwise be lost
        // and wait out the whole open.
        bool unused_reinit = false;
        if (!handle_commands(unused_reinit)) {
            camera_->cancel_initialize();
            return false;
        }
        if (!is_running()) {
            camera_->cancel_initialize();
            break;
        }
        bool reinitialized = false;
        {
            std::lock_guard<std::mutex> lock(recording_mutex_);
            reinitialized = camera_->initialize();
        }
        if (reinitialized) break;

        handle_commands(unused_reinit);
        spdlog::warn("Camera reinitialize attempt failed. Exiting.");
        return false;
    }

    if (!is_running()) {
        return false;
    }
    return true;
}

bool Runner::handle_uninitialized_tick(const CameraData &camera_data, double loop_rate_hz) {
    publisher_->publish_camera_data(camera_data);
    publish_system_status(true, loop_rate_hz);
    return true;
}

void Runner::initialize() {
    if (!camera_->initialize()) {
        spdlog::error("Failed to initialize camera");
    }
    if (!field_model_->initialize()) {
        throw std::runtime_error("Failed to initialize field model");
    }
    if (!robot_mask_model_->initialize()) {
        throw std::runtime_error("Failed to initialize robot blob model");
    }
    if (!keypoint_model_->initialize()) {
        throw std::runtime_error("Failed to initialize keypoint model");
    }
    control_loop_->set_autonomy_enabled(autonomy_enabled_);
    control_loop_->set_behavior_mode(behavior_mode_);
    if (!auto_tracking_) control_loop_->set_target_override(Pose2D{});
    if (!control_loop_->start()) {
        spdlog::error("Failed to initialize control loop");
    }
    diagnostics_logger_->debug({}, "Initialization complete");
    DiagnosticsLogger::publish();
}

void Runner::initialize_field(const CameraData &camera_data) {
    spdlog::info("Initializing field");
    field_filter_->reset(camera_data.tf_visodom_from_camera);
    MaskStamped field_mask = field_model_->update(camera_data.rgb);

    if (field_mask.mask.mask.empty()) {
        spdlog::error("Field model returned an empty mask; skipping field initialization.");
        return;
    }

    publisher_->publish_field_mask(field_mask, camera_data.rgb, camera_data.camera_info);

    initial_field_description_ = field_filter_->compute_field(camera_data, field_mask);
    if (initial_field_description_->header.frame_id == FrameId::EMPTY) {
        spdlog::error("Failed to find a plane.");
        return;
    }
    publisher_->publish_initial_field_description(*initial_field_description_);

    control_loop_->request_filter_reinit(runtime_opponent_count_);
    // Static-gate clusters are stored in field coordinates. A re-init moves the field origin, so
    // every stored position now refers to somewhere else. Start over.
    static_gate_->reset();
    initialized_ = true;
    field_init_attempts_remaining_ = 0;
    spdlog::info("Field initialized");
}

int Runner::run() {
    const bool paced = runner_config_.max_loop_rate > 0.0;
    const auto loop_duration = paced ? std::chrono::microseconds(static_cast<int64_t>(
                                           1000000.0 / runner_config_.max_loop_rate))
                                     : std::chrono::microseconds(0);
    auto prev_time = std::chrono::steady_clock::now();

    while (true) {
        auto current_time = std::chrono::steady_clock::now();

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
            stop_recordings_for_shutdown();
            return 0;
        }
        if (!control_loop_->is_healthy()) {
            if (!control_loop_stalled_) {
                spdlog::error("Control loop missed its watchdog deadline; disabling autonomy.");
                control_loop_stalled_ = true;
                control_loop_stalled_since_ = tick_start;
            }
            autonomy_enabled_ = false;
            control_loop_->set_autonomy_enabled(false);
        } else if (control_loop_stalled_) {
            spdlog::warn("Control loop is cycling again after {:.0f} ms; autonomy stays off.",
                         ms_since(control_loop_stalled_since_));
            control_loop_stalled_ = false;
        }
        health_logger_->record_tick(ms_since(tick_start));
        health_logger_->maybe_log();

        DiagnosticsLogger::publish();
    }
}

bool Runner::tick() {
    ticks_started_.fetch_add(1);
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

    bool should_reinit_field = false;
    if (!handle_commands(should_reinit_field)) return false;
    roll_over_recording_if_full();

    control_loop_->pump_input();
    update_timers();
    should_reinit_field = should_reinit_field || control_loop_->take_init_button_press();

    CameraData camera_data;
    bool is_camera_ok;
    {
        FunctionTimer timer(diagnostics_logger_, "camera.get");
        is_camera_ok = camera_->get(camera_data);
    }

    if (!is_camera_ok) {
        publish_system_status(false, loop_rate_hz);
        return recover_camera_after_failure();
    }

    if (clock_) {
        clock_->set(camera_data.rgb.header.stamp);
    }

    if (should_reinit_field) {
        field_init_attempts_remaining_ = kFieldInitAttempts;
    }
    if (field_init_attempts_remaining_ > 0) {
        if (camera_data.pose_tracking_ready) {
            --field_init_attempts_remaining_;
            initialize_field(camera_data);
            if (field_init_attempts_remaining_ == 0 && !initialized_) {
                spdlog::error("Field initialization gave up after {} frames.", kFieldInitAttempts);
            }
        } else {
            spdlog::warn("Skipping field initialization because camera tracking is not ready.");
        }
    }

    if (!initialized_) return handle_uninitialized_tick(camera_data, loop_rate_hz);

    FieldDescription field_description;
    {
        FunctionTimer timer(diagnostics_logger_, "field_filter.track_field");
        field_description = field_filter_->track_field(camera_data.tf_visodom_from_camera,
                                                       initial_field_description_);
    }

    ModelResultStamped keypoints;
    ModelResultStamped robot_blob_keypoints;
    if (runner_config_.parallel_models) {
        FunctionTimer timer(diagnostics_logger_, "perception_batch.update");
        BatchResult batch = perception_batch_->update(camera_data.rgb, camera_data.camera_info);
        keypoints = std::move(batch.keypoints);
        robot_blob_keypoints = std::move(batch.robot_blob_keypoints);
        DiagnosticsData keypoint_timing;
        keypoint_timing["elapsed_ms"] = batch.keypoint_model_elapsed_ms;
        diagnostics_logger_->info("keypoint_model.update", keypoint_timing, "");
        DiagnosticsData robot_blob_timing;
        robot_blob_timing["elapsed_ms"] = batch.robot_blob_model_elapsed_ms;
        diagnostics_logger_->info("robot_mask_model.update", robot_blob_timing, "");
    } else {
        {
            FunctionTimer timer(diagnostics_logger_, "keypoint_model.update");
            keypoints = keypoint_model_->update(camera_data.rgb, camera_data.camera_info);
        }
        {
            FunctionTimer timer(diagnostics_logger_, "robot_mask_model.update");
            robot_blob_keypoints = robot_mask_model_->update(camera_data.rgb);
        }
    }

    {
        FunctionTimer timer(diagnostics_logger_, "keypoint_filter");
        robot_blob_keypoints =
            static_gate_->filter(robot_blob_keypoints, camera_data.camera_info, field_description,
                                 camera_data.rgb.header.stamp);
        keypoints = height_gate_->filter(keypoints, camera_data.depth, camera_data.camera_info,
                                         field_description);
        robot_blob_keypoints = height_gate_->filter(robot_blob_keypoints, camera_data.depth,
                                                    camera_data.camera_info, field_description);
    }

    control_loop_->submit_measurement(ControlMeasurement{
        .keypoints = keypoints,
        .robot_blob_keypoints = robot_blob_keypoints,
        .field_description = field_description,
        .camera_info = camera_data.camera_info,
    });
    {
        FunctionTimer timer(diagnostics_logger_, "control_loop.advance");
        control_loop_->advance_to(camera_data.rgb.header.stamp);
    }

    const ControlOutput control_output = control_loop_->latest_output();
    const RobotDescriptionsStamped &robots = control_output.robots;

    FieldDescription hazard_view = field_description;
    hazard_view.hazards = control_output.hazards;

    {
        const double pipeline_latency_ms =
            ((clock_ ? clock_->now() : auto_battlebot::now()) - camera_data.rgb.header.stamp) *
            1000.0;
        diagnostics_logger_->debug("pipeline", {{"latency_ms", pipeline_latency_ms}});
    }

    {
        FunctionTimer timer(diagnostics_logger_, "publishers");
        publisher_->publish_camera_data(camera_data);
        publisher_->publish_field_description(field_description, *initial_field_description_);
        publisher_->publish_hazards(hazard_view);
        publisher_->publish_robots(robots);
        publisher_->publish_blob_detections(robot_mask_model_->last_detections());
        publisher_->publish_keypoint_detections(keypoint_model_->last_detections());
        publisher_->publish_navigation(control_loop_->last_visualization());
    }

    publish_system_status(true, loop_rate_hz);
    projection_field_ = field_description;
    projection_camera_info_ = camera_data.camera_info;
    publish_tracks(robots, field_description, camera_data.camera_info, control_output.target,
                   control_output.manual_target);
    if (remote_.status) {
        const auto &sticks = control_output.command_feedback.stick_commands;
        if (auto it = sticks.find(FrameId::OUR_ROBOT_1); it != sticks.end()) {
            remote_.status->publish(
                remote::SticksMessage{.linear = it->second.forward, .angular = it->second.yaw});
        }
    }

    return true;
}

double Runner::elapsed_ms() {
    auto now = std::chrono::steady_clock::now();
    double elapsed = to_ms(now - prev_tick_start_);
    prev_tick_start_ = now;
    return elapsed;
}

}  // namespace auto_battlebot
