#pragma once

#include <memory>
#include <stdexcept>

#include "config/config_cast.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "enums.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"

namespace auto_battlebot {
struct RgbdCameraConfiguration {
    std::string type;
    virtual ~RgbdCameraConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopRgbdCameraConfiguration : public RgbdCameraConfiguration {
    NoopRgbdCameraConfiguration() { type = "NoopRgbdCamera"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct ZedRgbdCameraConfiguration : public RgbdCameraConfiguration {
    int camera_fps = 30;
    Resolution camera_resolution = Resolution::RES_1280x720;
    DepthMode depth_mode = DepthMode::ZED_NEURAL_LIGHT;
    std::string svo_file_path = "";
    bool svo_real_time_mode = true;
    bool position_tracking = true;

    ZedRgbdCameraConfiguration() { type = "ZedRgbdCamera"; }

    // clang-format off
        PARSE_CONFIG_FIELDS(
            PARSE_FIELD(camera_fps)
            PARSE_ENUM(camera_resolution, Resolution)
            PARSE_ENUM(depth_mode, DepthMode)
            PARSE_FIELD_STRING(svo_file_path)
            PARSE_FIELD_BOOL(svo_real_time_mode)
            PARSE_FIELD_BOOL(position_tracking)
        )
    // clang-format on
};

struct SimRgbdCameraConfiguration : public RgbdCameraConfiguration {
    // TCP settings
    std::string tcp_host = "127.0.0.1";
    int tcp_port = 18707;
    int tcp_connect_timeout_ms = 5000;
    int tcp_read_timeout_ms = 100;
    bool tcp_auto_reconnect = true;

    SimRgbdCameraConfiguration() { type = "SimRgbdCamera"; }

    // clang-format off
        PARSE_CONFIG_FIELDS(
            PARSE_FIELD_STRING(tcp_host)
            PARSE_FIELD(tcp_port)
            PARSE_FIELD(tcp_connect_timeout_ms)
            PARSE_FIELD(tcp_read_timeout_ms)
            PARSE_FIELD_BOOL(tcp_auto_reconnect)
        )
    // clang-format on
};

std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config);
std::unique_ptr<RgbdCameraConfiguration> parse_rgbd_camera_config(ConfigParser &parser);
std::unique_ptr<RgbdCameraConfiguration> load_camera_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
