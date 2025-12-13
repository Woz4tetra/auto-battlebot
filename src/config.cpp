#include "config.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace auto_battlebot {

ClassConfiguration load_classes_from_config(const std::string& config_path) {
    std::string path = config_path.empty() ? "config/classes.json" : config_path;
    
    ClassConfiguration config;
    
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Failed to open config file: " << path << std::endl;
            std::cerr << "Using default configuration" << std::endl;
            
            // Default configuration
            config.camera = "realsense";
            config.field_model = "default";
            config.field_filter = "default";
            config.keypoint_model = "default";
            config.robot_filter = "default";
            config.navigation = "default";
            config.transmitter = "default";
            
            return config;
        }
        
        json j;
        file >> j;
        
        config.camera = j.value("camera", "realsense");
        config.field_model = j.value("field_model", "default");
        config.field_filter = j.value("field_filter", "default");
        config.keypoint_model = j.value("keypoint_model", "default");
        config.robot_filter = j.value("robot_filter", "default");
        config.navigation = j.value("navigation", "default");
        config.transmitter = j.value("transmitter", "default");
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing config file: " << e.what() << std::endl;
        std::cerr << "Using default configuration" << std::endl;
        
        // Default configuration
        config.camera = "realsense";
        config.field_model = "default";
        config.field_filter = "default";
        config.keypoint_model = "default";
        config.robot_filter = "default";
        config.navigation = "default";
        config.transmitter = "default";
    }
    
    return config;
}

std::vector<RobotConfig> load_robots_from_config(const std::string& config_path) {
    std::string path = config_path.empty() ? "config/robots.json" : config_path;
    
    std::vector<RobotConfig> robots;
    
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Failed to open robot config file: " << path << std::endl;
            std::cerr << "Using empty robot configuration" << std::endl;
            return robots;
        }
        
        json j;
        file >> j;
        
        if (j.is_array()) {
            for (const auto& robot_json : j) {
                RobotConfig robot;
                robot.label = robot_json.value("label", "");
                robot.group = robot_json.value("group", "");
                robots.push_back(robot);
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing robot config file: " << e.what() << std::endl;
        return robots;
    }
    
    return robots;
}

// Factory function implementations
// These return nullptr as placeholders - implement concrete classes later
std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const std::string& camera_type) {
    std::cerr << "Factory for camera type '" << camera_type << "' not implemented" << std::endl;
    return nullptr;
}

std::shared_ptr<FieldModelInterface> make_field_model(const std::string& model_type) {
    std::cerr << "Factory for field model type '" << model_type << "' not implemented" << std::endl;
    return nullptr;
}

std::shared_ptr<FieldFilterInterface> make_field_filter(const std::string& filter_type) {
    std::cerr << "Factory for field filter type '" << filter_type << "' not implemented" << std::endl;
    return nullptr;
}

std::shared_ptr<KeypointModelInterface> make_keypoint_model(const std::string& model_type) {
    std::cerr << "Factory for keypoint model type '" << model_type << "' not implemented" << std::endl;
    return nullptr;
}

std::shared_ptr<RobotFilterInterface> make_robot_filter(const std::string& filter_type) {
    std::cerr << "Factory for robot filter type '" << filter_type << "' not implemented" << std::endl;
    return nullptr;
}

std::shared_ptr<NavigationInterface> make_navigation(const std::string& navigation_type) {
    std::cerr << "Factory for navigation type '" << navigation_type << "' not implemented" << std::endl;
    return nullptr;
}

std::shared_ptr<TransmitterInterface> make_transmitter(const std::string& transmitter_type) {
    std::cerr << "Factory for transmitter type '" << transmitter_type << "' not implemented" << std::endl;
    return nullptr;
}

}  // namespace auto_battlebot
