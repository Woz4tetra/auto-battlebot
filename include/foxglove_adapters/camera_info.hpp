#pragma once

#include <foxglove/schemas.hpp>
#include <string>

#include "data_structures/camera.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::CameraCalibration to_camera_calibration(const CameraInfo &camera_info);

std::string to_frame_meta_json(const FrameIdentity &identity);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
