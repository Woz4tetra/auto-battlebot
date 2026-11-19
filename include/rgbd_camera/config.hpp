#pragma once

#include <memory>
#include <stdexcept>

#include "config/config_cast.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "enums.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"

namespace auto_battlebot {
struct RgbdCameraConfiguration {
    std::string type;
    virtual ~RgbdCameraConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopRgbdCameraConfiguration : public RgbdCameraConfiguration {
    NoopRgbdCameraConfiguration() { type = "NoopRgbdCamera"; }

    PARSE_CONFIG_FIELDS()
};

struct ZedRgbdCameraConfiguration : public RgbdCameraConfiguration {
    int camera_fps = 30;
    Resolution camera_resolution = Resolution::RES_1280x720;
    DepthMode depth_mode = DepthMode::ZED_NEURAL_LIGHT;
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
            PARSE_FIELD_BOOL(position_tracking)
            PARSE_FIELD_BOOL(svo_recording)
            PARSE_FIELD(svo_max_size_gb)
            PARSE_FIELD(svo_holding_dir_max_size_gb)
        )
    // clang-format on
};

struct ZedSvoPlaybackCameraConfiguration : public RgbdCameraConfiguration {
    int camera_fps = 30;
    Resolution camera_resolution = Resolution::RES_1280x720;
    DepthMode depth_mode = DepthMode::ZED_NEURAL_LIGHT;
    std::string svo_file_path = "";
    int svo_start_frame = 0;
    bool svo_real_time_mode = true;
    bool position_tracking = true;
    bool rebase_stamps = true;

    ZedSvoPlaybackCameraConfiguration() { type = "ZedSvoPlaybackCamera"; }

    // clang-format off
        PARSE_CONFIG_FIELDS(
            PARSE_FIELD(camera_fps)
            PARSE_ENUM(camera_resolution, Resolution)
            PARSE_ENUM(depth_mode, DepthMode)
            PARSE_FIELD_STRING(svo_file_path)
            PARSE_FIELD(svo_start_frame)
            PARSE_FIELD_BOOL(svo_real_time_mode)
            PARSE_FIELD_BOOL(position_tracking)
            PARSE_FIELD_BOOL(rebase_stamps)
        )
    // clang-format on
};

struct V4l2RgbCameraConfiguration : public RgbdCameraConfiguration {
    std::string device = "/dev/video0";
    Resolution camera_resolution = Resolution::RES_1920x1200;
    int camera_fps = 60;
    int buffer_count = 4;
    std::string calibration_file = "";
    bool video_recording = true;
    int video_bitrate_kbps = 15000;
    int exposure_us = 0;
    int gain = 0;
    int white_balance_temperature = 0;

    V4l2RgbCameraConfiguration() { type = "V4l2RgbCamera"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_STRING(device)
        PARSE_ENUM(camera_resolution, Resolution)
        PARSE_FIELD(camera_fps)
        PARSE_FIELD(buffer_count)
        PARSE_FIELD_STRING(calibration_file)
        PARSE_FIELD_BOOL(video_recording)
        PARSE_FIELD(video_bitrate_kbps)
        PARSE_FIELD(exposure_us)
        PARSE_FIELD(gain)
        PARSE_FIELD(white_balance_temperature)
    )
    // clang-format on
};

struct ZedOneRgbCameraConfiguration : public RgbdCameraConfiguration {
    Resolution camera_resolution = Resolution::RES_1920x1200;
    int camera_fps = 60;
    bool video_recording = true;
    int video_bitrate_kbps = 15000;

    ZedOneRgbCameraConfiguration() { type = "ZedOneRgbCamera"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_ENUM(camera_resolution, Resolution)
        PARSE_FIELD(camera_fps)
        PARSE_FIELD_BOOL(video_recording)
        PARSE_FIELD(video_bitrate_kbps)
    )
    // clang-format on
};

struct VideoPlaybackCameraConfiguration : public RgbdCameraConfiguration {
    std::string video_file_path = "";
    int start_frame = 0;
    bool real_time_mode = true;
    bool rebase_stamps = true;
    std::string calibration_file = "";

    VideoPlaybackCameraConfiguration() { type = "VideoPlaybackCamera"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_STRING(video_file_path)
        PARSE_FIELD(start_frame)
        PARSE_FIELD_BOOL(real_time_mode)
        PARSE_FIELD_BOOL(rebase_stamps)
        PARSE_FIELD_STRING(calibration_file)
    )
    // clang-format on
};

struct SimRgbdCameraConfiguration : public RgbdCameraConfiguration {
    std::string sim_host = "127.0.0.1";
    int sim_port = 14882;

    SimRgbdCameraConfiguration() { type = "SimRgbdCamera"; }

    PARSE_CONFIG_FIELDS(PARSE_FIELD_STRING(sim_host) PARSE_FIELD(sim_port))
};

void reserve_camera_input_paths(const RgbdCameraConfiguration &config);

std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config,
                                                      std::shared_ptr<McapRecorder> mcap_recorder);
std::unique_ptr<RgbdCameraConfiguration> parse_rgbd_camera_config(ConfigParser &parser);
std::unique_ptr<RgbdCameraConfiguration> load_camera_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
