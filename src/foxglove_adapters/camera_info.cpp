#include "foxglove_adapters/camera_info.hpp"

#include <cstdio>

#include "foxglove_adapters/common.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::CameraCalibration to_camera_calibration(const CameraInfo &camera_info) {
    foxglove::schemas::CameraCalibration calibration;
    calibration.timestamp = to_timestamp(camera_info.header.stamp);
    calibration.frame_id = frame_id_string(camera_info.header.frame_id);
    calibration.width = static_cast<uint32_t>(camera_info.width);
    calibration.height = static_cast<uint32_t>(camera_info.height);

    calibration.k.fill(0.0);
    if (camera_info.intrinsics.rows == 3 && camera_info.intrinsics.cols == 3) {
        for (int i = 0; i < 9; ++i) {
            calibration.k[i] = camera_info.intrinsics.at<double>(i / 3, i % 3);
        }
    }

    if (!camera_info.distortion.empty()) {
        calibration.d.resize(camera_info.distortion.total());
        for (size_t i = 0; i < camera_info.distortion.total(); ++i) {
            calibration.d[i] = camera_info.distortion.at<double>(static_cast<int>(i));
        }
        calibration.distortion_model = "plumb_bob";
    }

    for (int i = 0; i < 9; ++i) calibration.r[i] = (i % 4 == 0) ? 1.0 : 0.0;

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            calibration.p[row * 4 + col] = calibration.k[row * 3 + col];
        }
        calibration.p[row * 4 + 3] = 0.0;
    }
    return calibration;
}

std::string to_frame_meta_json(const FrameIdentity &identity) {
    char buffer[64];
    std::string json = "{\"image_stamp_ns\":\"";
    std::snprintf(buffer, sizeof(buffer), "%llu",
                  static_cast<unsigned long long>(identity.image_stamp_ns));
    json += buffer;
    json += "\",\"svo_frame_index\":";
    std::snprintf(buffer, sizeof(buffer), "%lld", static_cast<long long>(identity.svo_frame_index));
    json += buffer;
    json += ",\"svo_path\":\"";
    json += json_escape(identity.svo_path);
    json += "\"}";
    return json;
}

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
