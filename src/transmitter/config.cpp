#include "transmitter/config.hpp"
#include "transmitter/noop_transmitter.hpp"

namespace auto_battlebot
{
    std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config)
    {
        if (config.type == "NoopTransmitter")
        {
            return std::make_shared<NoopTransmitter>();
        }
        return nullptr;
    }
} // namespace auto_battlebot
