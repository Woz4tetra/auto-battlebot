#pragma once

#include <stdexcept>
#include <memory>

#include "enums.hpp"
#include "data_structures.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "config_factory.hpp"
#include "config_parser.hpp"
#include "config_cast.hpp"

namespace auto_battlebot
{
    struct RgbdCameraConfiguration
    {
        std::string type;
        virtual ~RgbdCameraConfiguration() = default;
        virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    };

    struct NoopRgbdCameraConfiguration : public RgbdCameraConfiguration
    {
        NoopRgbdCameraConfiguration()
        {
            type = "NoopRgbdCamera";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    struct ZedRgbdCameraConfiguration : public RgbdCameraConfiguration
    {
        int camera_fps = 30;
        Resolution camera_resolution = Resolution::RES_1280x720;
        DepthMode depth_mode = DepthMode::ZED_NEURAL;
        std::string svo_file_path = "";
        bool svo_real_time_mode = true;
        bool position_tracking = true;

        ZedRgbdCameraConfiguration()
        {
            type = "ZedRgbdCamera";
        }

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

    std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config);
    std::unique_ptr<RgbdCameraConfiguration> parse_rgbd_camera_config(ConfigParser &parser);
} // namespace auto_battlebot
