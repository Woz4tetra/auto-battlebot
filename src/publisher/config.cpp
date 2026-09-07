#include "publisher/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "publisher/foxglove_publisher.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(PublisherConfiguration, NoopPublisherConfiguration, "NoopPublisher")
REGISTER_CONFIG(PublisherConfiguration, FoxglovePublisherConfiguration, "FoxglovePublisher")

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

std::shared_ptr<VizSink> make_viz_sink(const PublisherConfiguration &config) {
    if (!config.uses_viz()) return nullptr;
    const auto &foxglove_config = dynamic_cast<const FoxglovePublisherConfiguration &>(config);
    return std::make_shared<VizSink>(foxglove_config.socket_path);
}

std::shared_ptr<PublisherInterface> make_publisher(const PublisherConfiguration &config,
                                                   std::shared_ptr<VizSink> sink,
                                                   std::shared_ptr<McapRecorder> mcap_recorder) {
    spdlog::info("Selected {} for Publisher", config.type);
    if (config.type == "NoopPublisher") {
        return std::make_shared<NoopPublisher>();
    } else if (config.type == "FoxglovePublisher") {
        return std::make_shared<FoxglovePublisher>(std::move(sink), std::move(mcap_recorder));
    }
    throw std::invalid_argument("Failed to load Publisher of type " + config.type);
}
}  // namespace auto_battlebot
