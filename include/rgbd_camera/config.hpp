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
    int svo_start_frame = 0;
    bool svo_real_time_mode = true;
    bool position_tracking = true;
    bool svo_recording = true;
    uint64_t svo_max_size_gb = 10;
    uint64_t svo_holding_dir_max_size_gb = 50;

    ZedRgbdCameraConfiguration() { type = "ZedRgbdCamera"; }

    // clang-format off
        PARSE_CONFIG_FIELDS(
            PARSE_FIELD(camera_fps)
            PARSE_ENUM(camera_resolution, Resolution)
            PARSE_ENUM(depth_mode, DepthMode)
            PARSE_FIELD_STRING(svo_file_path)
            PARSE_FIELD(svo_start_frame)
            PARSE_FIELD_BOOL(svo_real_time_mode)
            PARSE_FIELD_BOOL(position_tracking)
            PARSE_FIELD_BOOL(svo_recording)
            PARSE_FIELD(svo_max_size_gb)
            PARSE_FIELD(svo_holding_dir_max_size_gb)
        )
    // clang-format on
};

struct SimRgbdCameraConfiguration : public RgbdCameraConfiguration {
    std::string sim_host = "127.0.0.1";
    int sim_port = 14882;

    SimRgbdCameraConfiguration() { type = "SimRgbdCamera"; }

    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_STRING(sim_host)
        PARSE_FIELD(sim_port)
    )
};

std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config);
std::unique_ptr<RgbdCameraConfiguration> parse_rgbd_camera_config(ConfigParser &parser);
std::unique_ptr<RgbdCameraConfiguration> load_camera_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
