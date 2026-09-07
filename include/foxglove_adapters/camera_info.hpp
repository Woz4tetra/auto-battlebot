#pragma once

#include <foxglove/schemas.hpp>
#include <string>

#include "data_structures/camera.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

/** CameraInfo to foxglove.CameraCalibration: K from the intrinsics, D from the distortion
 *  (plumb_bob when present), R identity, P = [K | 0]. */
foxglove::schemas::CameraCalibration to_camera_calibration(const CameraInfo &camera_info);

/** FrameIdentity as the `/camera/frame_meta` JSON payload. `image_stamp_ns` is emitted as a
 *  string: it exceeds 2^53 and a JSON number would be rounded by JavaScript consumers. */
std::string to_frame_meta_json(const FrameIdentity &identity);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
