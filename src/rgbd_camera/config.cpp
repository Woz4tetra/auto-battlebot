#include "rgbd_camera/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <filesystem>

#include "config/config_parser.hpp"
#include "directories.hpp"
#include "rgbd_camera/noop_rgbd_camera.hpp"
#include "rgbd_camera/sim_rgbd_camera.hpp"
#include "rgbd_camera/v4l2_rgb_camera.hpp"
#include "rgbd_camera/video_playback_camera.hpp"
#ifdef BUILD_WITH_ZED
#include "rgbd_camera/zed_rgbd_camera.hpp"
#include "rgbd_camera/zed_svo_playback_camera.hpp"
#endif

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(RgbdCameraConfiguration, NoopRgbdCameraConfiguration, "NoopRgbdCamera")
REGISTER_CONFIG(RgbdCameraConfiguration, SimRgbdCameraConfiguration, "SimRgbdCamera")
REGISTER_CONFIG(RgbdCameraConfiguration, V4l2RgbCameraConfiguration, "V4l2RgbCamera")
REGISTER_CONFIG(RgbdCameraConfiguration, VideoPlaybackCameraConfiguration, "VideoPlaybackCamera")
#ifdef BUILD_WITH_ZED
REGISTER_CONFIG(RgbdCameraConfiguration, ZedRgbdCameraConfiguration, "ZedRgbdCamera")
REGISTER_CONFIG(RgbdCameraConfiguration, ZedSvoPlaybackCameraConfiguration, "ZedSvoPlaybackCamera")
#endif

std::unique_ptr<RgbdCameraConfiguration> parse_rgbd_camera_config(ConfigParser &parser) {
    return ConfigFactory<RgbdCameraConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<RgbdCameraConfiguration> load_camera_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["rgbd_camera"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [rgbd_camera]");
    }
    ConfigParser parser(*section, "rgbd_camera");
    auto config = parse_rgbd_camera_config(parser);
    parsed_sections.push_back("rgbd_camera");
    return config;
}

void reserve_camera_input_paths(const RgbdCameraConfiguration &config) {
    if (config.type != "VideoPlaybackCamera") {
        return;
    }
    const auto &playback = config_cast<VideoPlaybackCameraConfiguration>(config);
    if (playback.video_file_path.empty()) {
        return;
    }
    std::filesystem::path path(playback.video_file_path);
    if (!path.is_absolute()) {
        path = get_project_root() / path;
    }
    McapRecorder::reserve_input_path(path);
}

std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config,
                                                      std::shared_ptr<McapRecorder> mcap_recorder) {
    spdlog::info("Selected {} for RgbdCamera", config.type);
    if (config.type == "NoopRgbdCamera") {
        return std::make_shared<NoopRgbdCamera>();
#ifdef BUILD_WITH_ZED
    } else if (config.type == "ZedRgbdCamera") {
        return std::make_shared<ZedRgbdCamera>(config_cast<ZedRgbdCameraConfiguration>(config));
    } else if (config.type == "ZedSvoPlaybackCamera") {
        return std::make_shared<ZedSvoPlaybackCamera>(
            config_cast<ZedSvoPlaybackCameraConfiguration>(config));
#endif
    } else if (config.type == "V4l2RgbCamera") {
        return std::make_shared<V4l2RgbCamera>(config_cast<V4l2RgbCameraConfiguration>(config),
                                               std::move(mcap_recorder));
    } else if (config.type == "VideoPlaybackCamera") {
        return std::make_shared<VideoPlaybackCamera>(
            config_cast<VideoPlaybackCameraConfiguration>(config));
    } else if (config.type == "SimRgbdCamera") {
        return std::make_shared<SimRgbdCamera>(config_cast<SimRgbdCameraConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load RgbdCamera of type " + config.type);
}
}  // namespace auto_battlebot
