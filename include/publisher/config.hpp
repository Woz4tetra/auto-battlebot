#pragma once

#include <memory>
#include <string>
#include <vector>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/noop_publisher.hpp"
#include "publisher/publisher_interface.hpp"
#include "viz/frame.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {
struct PublisherConfiguration {
    std::string type;
    virtual ~PublisherConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    /** True when the publisher streams to the viz relay and needs a VizSink constructed. */
    virtual bool uses_viz() const { return false; }
};

struct NoopPublisherConfiguration : public PublisherConfiguration {
    NoopPublisherConfiguration() { type = "NoopPublisher"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct FoxglovePublisherConfiguration : public PublisherConfiguration {
    FoxglovePublisherConfiguration() { type = "FoxglovePublisher"; }
    bool uses_viz() const override { return true; }

    /** Unix socket the viz_relay listens on. */
    std::string socket_path = viz::default_socket_path();

    PARSE_CONFIG_FIELDS(PARSE_FIELD_STRING(socket_path))
};

std::shared_ptr<PublisherInterface> make_publisher(
    const PublisherConfiguration &config, std::shared_ptr<VizSink> sink,
    std::shared_ptr<McapRecorder> mcap_recorder = nullptr);
/** Create the viz sink for a publisher config, or nullptr when it does not stream. */
std::shared_ptr<VizSink> make_viz_sink(const PublisherConfiguration &config);
std::unique_ptr<PublisherConfiguration> parse_publisher_config(ConfigParser &parser);
std::unique_ptr<PublisherConfiguration> load_publisher_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
