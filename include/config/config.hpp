#pragma once

#include <toml++/toml.h>

#include <fstream>
#include <iostream>
#include <magic_enum.hpp>
#include <memory>
#include <string>
#include <vector>

#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "directories.hpp"
#include "field_filter/config.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "keypoint_model/config.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "mask_model/config.hpp"
#include "mask_model/mask_model_interface.hpp"
#include "mcap_recorder/config.hpp"
#include "navigation/config.hpp"
#include "navigation/navigation_interface.hpp"
#include "publisher/config.hpp"
#include "publisher/publisher_interface.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "robot_blob_model/config.hpp"
#include "robot_blob_model/robot_blob_model_interface.hpp"
#include "robot_filter/config.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "runner_config.hpp"
#include "target_selector/config.hpp"
#include "transmitter/config.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "ui/config.hpp"

namespace auto_battlebot {
struct ClassConfiguration {
    std::unique_ptr<RgbdCameraConfiguration> camera;
    std::unique_ptr<MaskModelConfiguration> field_model;
    std::unique_ptr<RobotBlobModelConfiguration> robot_mask_model;
    std::unique_ptr<FieldFilterConfiguration> field_filter;
    std::unique_ptr<KeypointModelConfiguration> keypoint_model;
    std::unique_ptr<RobotFilterConfiguration> robot_filter;
    std::unique_ptr<TargetSelectorConfiguration> target_selector;
    std::unique_ptr<NavigationConfiguration> navigation;
    std::unique_ptr<TransmitterConfiguration> transmitter;
    std::unique_ptr<PublisherConfiguration> publisher;
    RunnerConfiguration runner;
    std::unique_ptr<UiConfiguration> ui;
    McapRecorderConfig mcap_recorder;
};

// Configuration loading functions
ClassConfiguration load_classes_from_config(const std::string &config_path = "");
}  // namespace auto_battlebot
