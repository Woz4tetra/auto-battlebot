#include "publisher/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "ros/typed_publisher.hpp"

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
std::shared_ptr<RosPublisher> make_ros_publisher(miniros::NodeHandle &nh,
                                                 std::shared_ptr<McapRecorder> mcap_recorder) {
    return std::make_shared<RosPublisher>(
        TypedPublisher<sensor_msgs::CompressedImage>::advertise(nh, "/camera/image", 10),
        TypedPublisher<sensor_msgs::CameraInfo>::advertise(nh, "/camera/camera_info", 10),
        TypedPublisher<sensor_msgs::CompressedImage>::advertise(nh, "/field_mask", 10, true),
        TypedPublisher<tf2_msgs::TFMessage>::advertise(nh, "/tf", 10),
        TypedPublisher<tf2_msgs::TFMessage>::advertise(nh, "/tf_static", 10, true),
        TypedPublisher<visualization_msgs::MarkerArray>::advertise(nh, "/field_markers", 10, true),
        TypedPublisher<visualization_msgs::MarkerArray>::advertise(nh, "/robot_markers", 10, true),
        TypedPublisher<visualization_msgs::MarkerArray>::advertise(nh, "/nav_markers", 10, true),
        std::move(mcap_recorder));
}

std::shared_ptr<PublisherInterface> make_publisher(miniros::NodeHandle &nh,
                                                   const PublisherConfiguration &config,
                                                   std::shared_ptr<McapRecorder> mcap_recorder) {
    spdlog::info("Selected {} for Publisher", config.type);
    if (config.type == "NoopPublisher") {
        return std::make_shared<NoopPublisher>();
    } else if (config.type == "RosPublisher") {
        return make_ros_publisher(nh, std::move(mcap_recorder));
    }
    throw std::invalid_argument("Failed to load Publisher of type " + config.type);
}
}  // namespace auto_battlebot
