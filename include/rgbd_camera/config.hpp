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

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
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

/** SVO replay. Separate from the live camera so recording, reconnection, and the capture thread
 *  are not reachable here, and so svo_recording cannot be combined with an svo_file_path. */
struct ZedSvoPlaybackCameraConfiguration : public RgbdCameraConfiguration {
    int camera_fps = 30;
    Resolution camera_resolution = Resolution::RES_1280x720;
    DepthMode depth_mode = DepthMode::ZED_NEURAL_LIGHT;
    std::string svo_file_path = "";
    int svo_start_frame = 0;
    bool svo_real_time_mode = true;
    bool position_tracking = true;
    /** Shift frame stamps to the current wall clock. Turn off to compare recordings between
     *  builds, where the offset otherwise makes every payload differ. */
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

/**
 * e-CAM25_CUONX over raw V4L2.
 *
 * No pixel-format or codec knob: the sensor emits UYVY and we encode H.264, so both are
 * hard-coded. There is one correct behaviour and one consumer; if a second capture format ever
 * turns up, that is when it becomes an enum, not now. `device` and `calibration_file` stay strings
 * because they are paths; `camera_resolution` is an enum because it has a fixed set of valid
 * values, so a misspelling fails at parse with the alternatives listed.
 */
struct V4l2RgbCameraConfiguration : public RgbdCameraConfiguration {
    std::string device = "/dev/video0";
    Resolution camera_resolution = Resolution::RES_1920x1200;
    int camera_fps = 60;
    /** Four, not the driver default. More buffers means the pipeline reads staler frames when it
     *  falls behind; four costs at most one frame of queueing. */
    int buffer_count = 4;
    /** config/cameras/<serial>.toml. Required: the lens is not a pinhole and everything
     *  downstream assumes one. */
    std::string calibration_file = "";
    bool video_recording = true;
    /** 15 Mbps is about 6.8 GB per hour and 340 MB for a three-minute match. */
    int video_bitrate_kbps = 15000;
    /** 0 keeps the driver default. Pinning exposure and white balance matters because
     *  auto-exposure hunting between a lit arena and a dark robot is a plausible source of
     *  frame-to-frame detector instability. */
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

/** Replay of an MCAP recording carrying /camera/video. Separate from the live camera for the same
 *  reason SVO playback is: no recorder, no reconnection, no capture thread, so every frame reaches
 *  the pipeline in order and replay stays reproducible. */
struct VideoPlaybackCameraConfiguration : public RgbdCameraConfiguration {
    std::string video_file_path = "";
    int start_frame = 0;
    bool real_time_mode = true;
    /** Shift frame stamps to the current wall clock. Turn off to compare recordings between
     *  builds, where the offset otherwise makes every payload differ. */
    bool rebase_stamps = true;
    /** Empty means rectify with whatever the recording itself names. Set it to re-rectify old
     *  footage against a revised calibration. */
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

/** Tell the recorder which files a camera will be replaying, so it cannot pick one of them as its
 *  own output. Call before constructing the recorder; a replay reads file A and records file B,
 *  and pointing the output at the input would corrupt the source. */
void reserve_camera_input_paths(const RgbdCameraConfiguration &config);

/** The RGB camera writes /camera/video straight to the recorder, so the factory needs it.
 *  Null when recording is off. */
std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config,
                                                      std::shared_ptr<McapRecorder> mcap_recorder);
std::unique_ptr<RgbdCameraConfiguration> parse_rgbd_camera_config(ConfigParser &parser);
std::unique_ptr<RgbdCameraConfiguration> load_camera_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
