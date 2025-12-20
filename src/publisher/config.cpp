#include "publisher/config.hpp"
#include "publisher/noop_publisher.hpp"
#include "config_parser.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(PublisherConfiguration, NoopPublisherConfiguration, "NoopPublisher")

    std::unique_ptr<PublisherConfiguration> parse_publisher_config(ConfigParser &parser)
    {
        return ConfigFactory<PublisherConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<PublisherInterface> make_publisher(const PublisherConfiguration &config)
    {
        if (config.type == "NoopPublisher")
        {
            return std::make_shared<NoopPublisher>();
        }
        throw std::invalid_argument("Failed to load Publisher of type " + config.type);
    }
} // namespace auto_battlebot
