#pragma once

#include <miniros/publisher.h>
#include <miniros/ros.h>

#include <sensor_msgs/CameraInfo.hxx>
#include <sensor_msgs/CompressedImage.hxx>
#include <sensor_msgs/PointCloud2.hxx>
#include <tf2_msgs/TFMessage.hxx>
#include <visualization_msgs/MarkerArray.hxx>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/noop_publisher.hpp"
#include "publisher/publisher_interface.hpp"
#include "publisher/ros_publisher.hpp"

namespace auto_battlebot {
struct PublisherConfiguration {
    std::string type;
    virtual ~PublisherConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopPublisherConfiguration : public PublisherConfiguration {
    NoopPublisherConfiguration() { type = "NoopPublisher"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct RosPublisherConfiguration : public PublisherConfiguration {
    RosPublisherConfiguration() { type = "RosPublisher"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

std::shared_ptr<PublisherInterface> make_publisher(
    miniros::NodeHandle &nh, const PublisherConfiguration &config,
    std::shared_ptr<McapRecorder> mcap_recorder = nullptr);
/** Create publisher without ROS (NoopPublisher only). Use when config.type is "NoopPublisher". */
std::shared_ptr<PublisherInterface> make_publisher_no_ros(const PublisherConfiguration &config);
std::unique_ptr<PublisherConfiguration> parse_publisher_config(ConfigParser &parser);
std::unique_ptr<PublisherConfiguration> load_publisher_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
