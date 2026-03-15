#include "publisher/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_parser.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(PublisherConfiguration, NoopPublisherConfiguration, "NoopPublisher")
REGISTER_CONFIG(PublisherConfiguration, RosPublisherConfiguration, "RosPublisher")

std::unique_ptr<PublisherConfiguration> parse_publisher_config(ConfigParser &parser) {
    return ConfigFactory<PublisherConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<PublisherConfiguration> load_publisher_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["publisher"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [publisher]");
    }
    ConfigParser parser(*section, "publisher");
    auto config = parse_publisher_config(parser);
    parsed_sections.push_back("publisher");
    return config;
}

std::shared_ptr<PublisherInterface> make_publisher_no_ros(const PublisherConfiguration &config) {
    if (config.type != "NoopPublisher") {
        throw std::invalid_argument("make_publisher_no_ros only supports NoopPublisher, got " +
                                    config.type);
    }
    spdlog::info("Selected NoopPublisher");
    return std::make_shared<NoopPublisher>();
}

std::shared_ptr<PublisherInterface> make_publisher(miniros::NodeHandle &nh,
                                                   const PublisherConfiguration &config,
                                                   std::shared_ptr<McapRecorder> mcap_recorder) {
    spdlog::info("Selected {} for Publisher", config.type);
    if (config.type == "NoopPublisher") {
        return std::make_shared<NoopPublisher>();
    } else if (config.type == "RosPublisher") {
        auto rgb_image_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<sensor_msgs::CompressedImage>("/camera/image", 10));
        auto camera_info_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 10));
        auto field_mask_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<sensor_msgs::CompressedImage>("/field_mask", 10, true));
        auto tf_publisher =
            std::make_shared<miniros::Publisher>(nh.advertise<tf2_msgs::TFMessage>("/tf", 10));
        auto static_tf_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<tf2_msgs::TFMessage>("/tf_static", 10, true));
        auto field_marker_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<visualization_msgs::MarkerArray>("/field_markers", 10, true));
        auto robot_marker_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<visualization_msgs::MarkerArray>("/robot_markers", 10, true));

        return std::make_shared<RosPublisher>(rgb_image_publisher, camera_info_publisher,
                                              field_mask_publisher, tf_publisher,
                                              static_tf_publisher, field_marker_publisher,
                                              robot_marker_publisher, std::move(mcap_recorder));
    }
    throw std::invalid_argument("Failed to load Publisher of type " + config.type);
}
}  // namespace auto_battlebot
