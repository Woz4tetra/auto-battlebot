#pragma once

#include "data_structures.hpp"
#include "interfaces/rgbd_camera_interface.hpp"
#include "interfaces/field_model_interface.hpp"
#include "interfaces/field_filter_interface.hpp"
#include "interfaces/keypoint_model_interface.hpp"
#include "interfaces/robot_filter_interface.hpp"
#include "interfaces/navigation_interface.hpp"
#include "interfaces/transmitter_interface.hpp"
#include <memory>
#include <vector>

namespace auto_battlebot {

class Runner {
public:
    Runner(
        const std::vector<RobotConfig>& robot_configs,
        std::shared_ptr<RgbdCameraInterface> camera,
        std::shared_ptr<FieldModelInterface> field_model,
        std::shared_ptr<FieldFilterInterface> field_filter,
        std::shared_ptr<KeypointModelInterface> keypoint_model,
        std::shared_ptr<RobotFilterInterface> robot_filter,
        std::shared_ptr<NavigationInterface> navigation,
        std::shared_ptr<TransmitterInterface> transmitter
    );

    void initialize();
    void initialize_field(const CameraData& camera_data);
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
    
    bool initialized_;
    FieldDescription initial_field_description_;
};

}  // namespace auto_battlebot
