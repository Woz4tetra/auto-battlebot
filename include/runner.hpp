#pragma once

#include "data_structures.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "field_model/field_model_interface.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "navigation/navigation_interface.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "publisher/publisher_interface.hpp"
#include <memory>
#include <vector>

namespace auto_battlebot
{
    class Runner
    {
    public:
        Runner(
            const std::vector<RobotConfig> &robot_configs,
            std::shared_ptr<RgbdCameraInterface> camera,
            std::shared_ptr<FieldModelInterface> field_model,
            std::shared_ptr<FieldFilterInterface> field_filter,
            std::shared_ptr<KeypointModelInterface> keypoint_model,
            std::shared_ptr<RobotFilterInterface> robot_filter,
            std::shared_ptr<NavigationInterface> navigation,
            std::shared_ptr<TransmitterInterface> transmitter,
            std::shared_ptr<PublisherInterface> publisher);

        void initialize();
        void initialize_field(const CameraData &camera_data);
        int run();
        bool tick();

    private:
        std::vector<RobotConfig> robot_configs_;
        std::shared_ptr<RgbdCameraInterface> camera_;
        std::shared_ptr<FieldModelInterface> field_model_;
        std::shared_ptr<FieldFilterInterface> field_filter_;
        std::shared_ptr<KeypointModelInterface> keypoint_model_;
        std::shared_ptr<RobotFilterInterface> robot_filter_;
        std::shared_ptr<NavigationInterface> navigation_;
        std::shared_ptr<TransmitterInterface> transmitter_;
        std::shared_ptr<PublisherInterface> publisher_;

        bool initialized_;
        FieldDescription initial_field_description_;
    };
} // namespace auto_battlebot
