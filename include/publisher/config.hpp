#pragma once

#include <miniros/ros.h>

#include "data_structures.hpp"
#include "publisher/publisher_interface.hpp"
#include "config_factory.hpp"

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

    std::shared_ptr<PublisherInterface> make_publisher(miniros::NodeHandle &nh, const PublisherConfiguration &config);
    std::unique_ptr<PublisherConfiguration> parse_publisher_config(ConfigParser &parser);
} // namespace auto_battlebot
