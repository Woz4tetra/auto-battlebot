#include "runner.hpp"

#include <thread>

namespace auto_battlebot
{

    Runner::Runner(
        const RunnerConfiguration &runner_config,
        const std::vector<RobotConfig> &robot_configs,
        std::shared_ptr<RgbdCameraInterface> camera,
        std::shared_ptr<FieldModelInterface> field_model,
        std::shared_ptr<FieldFilterInterface> field_filter,
        std::shared_ptr<KeypointModelInterface> keypoint_model,
        std::shared_ptr<RobotFilterInterface> robot_filter,
        std::shared_ptr<NavigationInterface> navigation,
        std::shared_ptr<TransmitterInterface> transmitter,
        std::shared_ptr<PublisherInterface> publisher) : runner_config_(runner_config),
                                                         robot_configs_(robot_configs),
                                                         camera_(camera),
                                                         field_model_(field_model),
                                                         field_filter_(field_filter),
                                                         keypoint_model_(keypoint_model),
                                                         robot_filter_(robot_filter),
                                                         navigation_(navigation),
                                                         transmitter_(transmitter),
                                                         publisher_(publisher),
                                                         initialized_(false),
                                                         initial_field_description_(),
                                                         start_time_(std::chrono::steady_clock::now())
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("runner");
    }

    void Runner::initialize()
    {
        // Initialize all interfaces
        if (!camera_->initialize())
        {
            std::cerr << "Failed to initialize camera" << std::endl;
        }
        if (!field_model_->initialize())
        {
            std::cerr << "Failed to initialize field model" << std::endl;
        }
        if (!keypoint_model_->initialize())
        {
            std::cerr << "Failed to initialize keypoint model" << std::endl;
        }
        if (!transmitter_->initialize())
        {
            std::cerr << "Failed to initialize transmitter" << std::endl;
        }
        diagnostics_logger_->debug({}, "Initialization complete");
        DiagnosticsLogger::publish();
    }

    void Runner::initialize_field(const CameraData &camera_data)
    {
        std::cout << "Initializing field" << std::endl;
        field_filter_->reset(camera_data.tf_visodom_from_camera);
        FieldMaskStamped field_mask = field_model_->update(camera_data.rgb);
        publisher_->publish_field_mask(field_mask, camera_data.rgb);

        initial_field_description_ = field_filter_->compute_field(camera_data, field_mask);
        publisher_->publish_initial_field_description(*initial_field_description_);

        robot_filter_->initialize(robot_configs_);
        navigation_->initialize();
        initialized_ = true;
        std::cout << "Field initialized" << std::endl;
    }

    int Runner::run()
    {
        auto loop_duration = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / runner_config_.loop_rate));
        auto next_tick_time = std::chrono::steady_clock::now();

        while (true)
        {
            if (!tick())
            {
                return 0;
            }
            DiagnosticsData rate_data;
            rate_data["rate"] = 1000.0 / elapsed_ms();
            diagnostics_logger_->debug(rate_data);
            DiagnosticsLogger::publish();

            // Sleep until next tick to maintain loop rate
            next_tick_time += loop_duration;
            std::this_thread::sleep_until(next_tick_time);
        }
    }

    bool Runner::tick()
    {
        FunctionTimer timer(diagnostics_logger_, "tick");
        diagnostics_logger_->debug({}, "Tick");
        if (!miniros::ok())
        {
            miniros::shutdown();
            return false;
        }

        transmitter_->update();

        CameraData camera_data;
        bool is_camera_ok;
        {
            FunctionTimer timer(diagnostics_logger_, "camera.get");
            is_camera_ok = camera_->get(camera_data);
        }

        if (!is_camera_ok)
        {
            if (camera_->should_close())
            {
                std::cerr << "Camera signalled to close the application" << std::endl;
                return false;
            }
            // Push error to diagnostics and log
            std::cerr << "Failed to get camera data. Reinitializing." << std::endl;
            while (!camera_->initialize() && miniros::ok())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (!miniros::ok())
            {
                miniros::shutdown();
                return false;
            }

            return true;
        }

        publisher_->publish_camera_data(camera_data);

        if (transmitter_->did_init_button_press())
        {
            initialize_field(camera_data);
        }

        if (!initialized_)
        {
            return true;
        }

        FieldDescription field_description;
        {
            FunctionTimer timer(diagnostics_logger_, "field_filter.track_field");
            field_description = field_filter_->track_field(
                camera_data.tf_visodom_from_camera,
                initial_field_description_);
        }
        publisher_->publish_field_description(field_description, *initial_field_description_);

        KeypointsStamped keypoints;
        {
            FunctionTimer timer(diagnostics_logger_, "keypoint_model.update");
            keypoints = keypoint_model_->update(camera_data.rgb);
        }
        // publisher_->publish_keypoints(keypoints);

        RobotDescriptionsStamped robots;
        {
            FunctionTimer timer(diagnostics_logger_, "robot_filter.update");
            robots = robot_filter_->update(keypoints, field_description);
        }
        // publisher_->publish_robots(robots);

        VelocityCommand command = navigation_->update(robots);
        transmitter_->send(command);

        return true;
    }

    double Runner::elapsed_ms()
    {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
        start_time_ = now;
        return duration.count() / 1000.0;
    }

} // namespace auto_battlebot
