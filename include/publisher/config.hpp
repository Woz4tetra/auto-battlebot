#pragma once

#include <miniros/ros.h>
#include <miniros/publisher.h>
#include <sensor_msgs/Image.hxx>
#include <sensor_msgs/CameraInfo.hxx>
#include <sensor_msgs/PointCloud2.hxx>
#include <tf2_msgs/TFMessage.hxx>
#include <visualization_msgs/MarkerArray.hxx>

#include "data_structures.hpp"
#include "publisher/publisher_interface.hpp"
#include "config/config_factory.hpp"
#include "publisher/noop_publisher.hpp"
#include "publisher/ros_publisher.hpp"
#include "config/config_parser.hpp"

namespace auto_battlebot
{
    struct PublisherConfiguration
    {
        std::string type;
        virtual ~PublisherConfiguration() = default;
        virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    };

    struct NoopPublisherConfiguration : public PublisherConfiguration
    {
        NoopPublisherConfiguration()
        {
            type = "NoopPublisher";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    struct RosPublisherConfiguration : public PublisherConfiguration
    {
        RosPublisherConfiguration()
        {
            type = "RosPublisher";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    std::shared_ptr<PublisherInterface> make_publisher(miniros::NodeHandle &nh, const PublisherConfiguration &config);
    std::unique_ptr<PublisherConfiguration> parse_publisher_config(ConfigParser &parser);
} // namespace auto_battlebot
