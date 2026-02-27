#include "runner.hpp"

#include <opencv2/core.hpp>

#include "enums.hpp"

namespace auto_battlebot {
static int count_opponents_in_config(const std::vector<RobotConfig> &configs) {
    int n = 0;
    for (const auto &r : configs) {
        if (r.label == Label::OPPONENT) n++;
    }
    return n > 0 ? n : 1;
}

Runner::Runner(const RunnerConfiguration &runner_config,
               const std::vector<RobotConfig> &robot_configs,
               std::shared_ptr<RgbdCameraInterface> camera,
               std::shared_ptr<FieldModelInterface> field_model,
               std::shared_ptr<FieldFilterInterface> field_filter,
               std::shared_ptr<KeypointModelInterface> keypoint_model,
               std::shared_ptr<RobotFilterInterface> robot_filter,
               std::shared_ptr<NavigationInterface> navigation,
               std::shared_ptr<TransmitterInterface> transmitter,
               std::shared_ptr<PublisherInterface> publisher, std::shared_ptr<UIState> ui_state)
    : runner_config_(runner_config),
      camera_(camera),
      field_model_(field_model),
      field_filter_(field_filter),
      keypoint_model_(keypoint_model),
      robot_filter_(robot_filter),
      navigation_(navigation),
      transmitter_(transmitter),
      publisher_(publisher),
      ui_state_(std::move(ui_state)),
      initial_robot_configs_(robot_configs),
      runtime_opponent_count_(count_opponents_in_config(robot_configs)),
      robot_filter_reinit_pending_(false),
      initialized_(false),
      initial_field_description_(),
      start_time_(std::chrono::steady_clock::now()) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("runner");
}

std::vector<RobotConfig> Runner::build_effective_robot_configs() const {
    std::vector<RobotConfig> out;
    for (const auto &r : initial_robot_configs_) {
        if (r.label != Label::OPPONENT) out.push_back(r);
    }
    for (int i = 0; i < runtime_opponent_count_; ++i) {
        out.push_back(RobotConfig{Label::OPPONENT, Group::THEIRS});
    }
    return out;
}

void Runner::initialize() {
    // Initialize all interfaces
    if (!camera_->initialize()) {
        std::cerr << "Failed to initialize camera" << std::endl;
    }
    if (!field_model_->initialize()) {
        std::cerr << "Failed to initialize field model" << std::endl;
    }
    if (!keypoint_model_->initialize()) {
        std::cerr << "Failed to initialize keypoint model." << std::endl;
    }
    if (!transmitter_->initialize()) {
        std::cerr << "Failed to initialize transmitter" << std::endl;
    }
    diagnostics_logger_->debug({}, "Initialization complete");
    DiagnosticsLogger::publish();
}

void Runner::initialize_field(const CameraData &camera_data) {
    std::cout << "Initializing field" << std::endl;
    field_filter_->reset(camera_data.tf_visodom_from_camera);
    FieldMaskStamped field_mask = field_model_->update(camera_data.rgb);
    publisher_->publish_field_mask(field_mask, camera_data.rgb);

    if (field_mask.mask.mask.empty()) {
        std::cerr << "Field model returned an empty mask; skipping field initialization."
                  << std::endl;
        return;
    }

    initial_field_description_ = field_filter_->compute_field(camera_data, field_mask);
    if (initial_field_description_->header.frame_id == FrameId::EMPTY) {
        std::cerr << "Failed to find a plane." << std::endl;
        return;
    }
    publisher_->publish_initial_field_description(*initial_field_description_);

    robot_filter_->initialize(build_effective_robot_configs());
    navigation_->initialize();
    initialized_ = true;
    std::cout << "Field initialized" << std::endl;
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
            double exceeded_time =
                -1 * std::chrono::duration_cast<std::chrono::microseconds>(remaining_time).count() /
                1000.0;
            diagnostics_logger_->debug("", {{"loop_duration_exceeded_ms", exceeded_time}});
        }
        std::this_thread::sleep_for(remaining_time);

        if (!tick()) {
            return 0;
        }

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
        miniros::shutdown();
        return false;
    }

    bool should_reinit_field = false;
    if (ui_state_) {
        if (ui_state_->quit_requested.load()) return false;
        should_reinit_field = ui_state_->reinit_requested.exchange(false);
        int req = ui_state_->opponent_count_requested.exchange(-1);
        if (req >= 1 && req <= 3) {
            runtime_opponent_count_ = req;
            robot_filter_->set_opponent_count(runtime_opponent_count_);
            robot_filter_reinit_pending_ = true;
        }
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
        if (camera_->should_close()) {
            std::cerr << "Camera signalled to close the application" << std::endl;
            return false;
        }
        std::cerr << "Failed to get camera data. Reinitializing." << std::endl;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } while (!camera_->initialize() && miniros::ok());

        if (!miniros::ok()) {
            miniros::shutdown();
            return false;
        }

        return true;
    }

    if (should_reinit_field) {
        robot_filter_reinit_pending_ = false;
        initialize_field(camera_data);
    } else if (robot_filter_reinit_pending_ && initialized_) {
        robot_filter_reinit_pending_ = false;
        robot_filter_->initialize(build_effective_robot_configs());
        navigation_->initialize();
    }

    if (!initialized_) {
        publisher_->publish_camera_data(camera_data);
        return true;
    }

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

    RobotDescriptionsStamped robots;
    {
        FunctionTimer timer(diagnostics_logger_, "robot_filter.update");
        robots = robot_filter_->update(keypoints, field_description, camera_data.camera_info,
                                       command_feedback);
    }

    {
        FunctionTimer timer(diagnostics_logger_, "publishers");
        publisher_->publish_camera_data(camera_data);
        publisher_->publish_field_description(field_description, *initial_field_description_);
        publisher_->publish_robots(robots);
    }

    VelocityCommand command = navigation_->update(robots, field_description);
    transmitter_->send(command);

    if (ui_state_) {
        SystemStatus status;
        status.camera_ok = true;
        status.transmitter_connected = transmitter_->is_connected();
        status.loop_rate_hz = loop_rate_hz;
        status.initialized = initialized_;
        ui_state_->set_system_status(status);
        ui_state_->set_robots(robots);
        ui_state_->set_keypoints(keypoints);
        ui_state_->set_navigation_path(navigation_->get_last_path());
        if (camera_data.rgb.image.data && !camera_data.rgb.image.empty()) {
            const cv::Mat &img = camera_data.rgb.image;
            std::vector<uint8_t> data(img.ptr<uint8_t>(),
                                      img.ptr<uint8_t>() + img.total() * img.elemSize());
            ui_state_->set_debug_image(img.cols, img.rows, img.channels(), data);
        }
    }

    return true;
}

double Runner::elapsed_ms() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
    start_time_ = now;
    return duration.count() / 1000.0;
}

}  // namespace auto_battlebot
