#include "transmitter/config.hpp"
#include "transmitter/noop_transmitter.hpp"
#include "config_parser.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(TransmitterConfiguration, NoopTransmitterConfiguration, "NoopTransmitter")

    std::unique_ptr<TransmitterConfiguration> parse_transmitter_config(ConfigParser &parser)
    {
        return ConfigFactory<TransmitterConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config)
    {
        if (config.type == "NoopTransmitter")
        {
            return std::make_shared<NoopTransmitter>();
        }
        throw std::invalid_argument("Failed to load Transmitter of type " + config.type);
    }
} // namespace auto_battlebot
