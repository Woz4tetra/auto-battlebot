#pragma once

#include <string>
#include <vector>
#include <memory>
#include "data_structures.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/config.hpp"
#include "field_model/field_model_interface.hpp"
#include "field_model/config.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "field_filter/config.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "keypoint_model/config.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "robot_filter/config.hpp"
#include "navigation/navigation_interface.hpp"
#include "navigation/config.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "transmitter/config.hpp"

namespace auto_battlebot
{
    struct ClassConfiguration
    {
        RgbdCameraConfiguration camera;
        FieldModelConfiguration field_model;
        FieldFilterConfiguration field_filter;
        KeypointModelConfiguration keypoint_model;
        RobotFilterConfiguration robot_filter;
        NavigationConfiguration navigation;
        TransmitterConfiguration transmitter;
    };

    // Configuration loading functions
    ClassConfiguration load_classes_from_config(const std::string &config_path = "");
    std::vector<RobotConfig> load_robots_from_config(const std::string &config_path = "");
} // namespace auto_battlebot
