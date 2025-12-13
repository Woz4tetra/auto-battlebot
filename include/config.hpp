#pragma once

#include <string>
#include <vector>
#include <memory>
#include "data_structures.hpp"
#include "interfaces/rgbd_camera_interface.hpp"
#include "interfaces/field_model_interface.hpp"
#include "interfaces/field_filter_interface.hpp"
#include "interfaces/keypoint_model_interface.hpp"
#include "interfaces/robot_filter_interface.hpp"
#include "interfaces/navigation_interface.hpp"
#include "interfaces/transmitter_interface.hpp"

namespace auto_battlebot {

struct ClassConfiguration {
    std::string camera;
    std::string field_model;
    std::string field_filter;
    std::string keypoint_model;
    std::string robot_filter;
    std::string navigation;
    std::string transmitter;
};

// Configuration loading functions
ClassConfiguration load_classes_from_config(const std::string& config_path = "");
std::vector<RobotConfig> load_robots_from_config(const std::string& config_path = "");

// Factory functions for creating interface implementations
std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const std::string& camera_type);
std::shared_ptr<FieldModelInterface> make_field_model(const std::string& model_type);
std::shared_ptr<FieldFilterInterface> make_field_filter(const std::string& filter_type);
std::shared_ptr<KeypointModelInterface> make_keypoint_model(const std::string& model_type);
std::shared_ptr<RobotFilterInterface> make_robot_filter(const std::string& filter_type);
std::shared_ptr<NavigationInterface> make_navigation(const std::string& navigation_type);
std::shared_ptr<TransmitterInterface> make_transmitter(const std::string& transmitter_type);

}  // namespace auto_battlebot
