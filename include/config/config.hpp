#pragma once

#include <toml++/toml.h>

#include <fstream>
#include <iostream>
#include <magic_enum.hpp>
#include <memory>
#include <string>
#include <vector>

#include "config/config_parser.hpp"
#include "control_loop/config.hpp"
#include "data_structures.hpp"
#include "directories.hpp"
#include "field_filter/config.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "health/config.hpp"
#include "keypoint_filter/config.hpp"
#include "keypoint_model/config.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "mask_model/config.hpp"
#include "mask_model/mask_model_interface.hpp"
#include "mcap_recorder/config.hpp"
#include "navigation/config.hpp"
#include "navigation/navigation_interface.hpp"
#include "plant/config.hpp"
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
#include "time/config.hpp"
#include "transmitter/config.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "ui/config.hpp"

namespace auto_battlebot {
struct ClassConfiguration {
    /** The shared jig fit from [plant]. Both the our-robot EKF and MotionProfileNavigation are
     * handed this after their own sections parse; see PlantConfiguration. */
    PlantConfiguration plant;
    std::unique_ptr<RgbdCameraConfiguration> camera;
    std::unique_ptr<MaskModelConfiguration> field_model;
    std::unique_ptr<RobotBlobModelConfiguration> robot_mask_model;
    std::unique_ptr<FieldFilterConfiguration> field_filter;
    std::unique_ptr<KeypointModelConfiguration> keypoint_model;
    KeypointFilterConfiguration keypoint_filter;
    std::unique_ptr<RobotFilterConfiguration> robot_filter;
    std::unique_ptr<TargetSelectorConfiguration> target_selector;
    std::unique_ptr<NavigationConfiguration> navigation;
    std::unique_ptr<TransmitterConfiguration> transmitter;
    std::unique_ptr<ControlLoopConfiguration> control_loop;
    std::unique_ptr<PublisherConfiguration> publisher;
    RunnerConfiguration runner;
    HealthConfiguration health;
    std::unique_ptr<UiConfiguration> ui;
    McapRecorderConfig mcap_recorder;
    std::unique_ptr<ClockConfiguration> clock;
};

// Configuration loading functions.
// `extends` values inside configs are resolved relative to `config_root`; when empty it defaults
// to get_config_dir(). Tests pass an explicit root so they can stay hermetic.
ClassConfiguration load_classes_from_config(const std::filesystem::path &path = {},
                                            const std::filesystem::path &config_root = {});

// Load `path` and resolve its `extends` chain into a single merged TOML table -- exactly the data
// fed to every section parser. Useful for dumping the effective config. Note this reflects only
// values explicitly set by the config (or its bases); defaults applied by parsers for omitted
// optional fields are not included.
toml::table load_merged_config(const std::filesystem::path &path,
                               const std::filesystem::path &config_root = {});
std::filesystem::path normalize_config_path(const std::string &config_path);
}  // namespace auto_battlebot
