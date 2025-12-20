#include "runner.hpp"
#include <iostream>

namespace auto_battlebot
{

    Runner::Runner(
        const std::vector<RobotConfig> &robot_configs,
        std::shared_ptr<RgbdCameraInterface> camera,
        std::shared_ptr<FieldModelInterface> field_model,
        std::shared_ptr<FieldFilterInterface> field_filter,
        std::shared_ptr<KeypointModelInterface> keypoint_model,
        std::shared_ptr<RobotFilterInterface> robot_filter,
        std::shared_ptr<NavigationInterface> navigation,
        std::shared_ptr<TransmitterInterface> transmitter) : robot_configs_(robot_configs),
                                                             camera_(camera),
                                                             field_model_(field_model),
                                                             field_filter_(field_filter),
                                                             keypoint_model_(keypoint_model),
                                                             robot_filter_(robot_filter),
                                                             navigation_(navigation),
                                                             transmitter_(transmitter),
                                                             initialized_(false),
                                                             initial_field_description_()
    {
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
    }

    void Runner::initialize_field(const CameraData &camera_data)
    {
        field_filter_->reset(camera_data.tf_visodom_from_camera);
        FieldMaskStamped field_mask = field_model_->update(camera_data.rgb);
        initial_field_description_ = field_filter_->compute_field(camera_data, field_mask);

        robot_filter_->initialize(robot_configs_);
        navigation_->initialize();
        initialized_ = true;
    }

    int Runner::run()
    {
        while (true)
        {
            if (!tick())
            {
                return 0;
            }
        }
    }

    bool Runner::tick()
    {
        transmitter_->update();

        CameraData camera_data;
        if (!camera_->update() || !camera_->get(camera_data))
        {
            if (camera_->should_close())
            {
                std::cerr << "Camera signalled to close the application" << std::endl;
                return false;
            }
            // Push error to diagnostics and log
            std::cerr << "Failed to get camera data. Reinitializing." << std::endl;
            while (!camera_->initialize())
            {
            }

            return true;
        }

        if (transmitter_->did_init_button_press())
        {
            initialize_field(camera_data);
        }

        if (!initialized_)
        {
            return true;
        }

        FieldDescription field_description = field_filter_->track_field(
            camera_data.tf_visodom_from_camera,
            initial_field_description_);
        KeypointsStamped keypoints = keypoint_model_->update(camera_data.rgb);
        RobotDescriptionsStamped robots = robot_filter_->update(keypoints, field_description);
        VelocityCommand command = navigation_->update(robots);
        transmitter_->send(command);

        return true;
    }

} // namespace auto_battlebot
