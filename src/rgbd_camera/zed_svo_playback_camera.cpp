#include "rgbd_camera/zed_svo_playback_camera.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <string>

#include "config/config_parser.hpp"

namespace auto_battlebot {

ZedSvoPlaybackCamera::ZedSvoPlaybackCamera(const ZedSvoPlaybackCameraConfiguration &config,
                                           std::unique_ptr<ZedDevice> device)
    : device_(std::move(device)),
      svo_start_frame_(config.svo_start_frame),
      svo_max_frames_(config.svo_max_frames),
      svo_decode_to_start_(config.svo_decode_to_start),
      rebase_stamps_(config.rebase_stamps),
      diagnostics_logger_(DiagnosticsLogger::get_logger("zed_svo_playback_camera")) {
    device_->set_tracking_enabled(config.position_tracking);

    const std::filesystem::path svo_abs_path =
        std::filesystem::absolute(std::filesystem::path(config.svo_file_path.c_str()));
    spdlog::info("Resolved SVO path: {}", svo_abs_path.string());
    svo_path_ = svo_abs_path.string();

    if (!rebase_stamps_) {
        spdlog::warn(
            "rebase_stamps = false: SVO stamps stay on the recording's clock; latency "
            "compensation and stamp-difference diagnostics are invalid");
    }
    if (!config.svo_real_time_mode) {
        spdlog::warn(
            "svo_real_time_mode = false: field timing, including latency compensation, is not "
            "reproduced");
    }

    sl::InitParameters params;
    params.camera_fps = config.camera_fps;
    params.camera_resolution = get_zed_resolution(config.camera_resolution);
    params.depth_mode = get_zed_depth_mode(config.depth_mode);
    params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    params.coordinate_units = sl::UNIT::METER;
    params.input.setFromSVOFile(svo_abs_path.c_str());
    params.svo_real_time_mode = config.svo_real_time_mode;
    device_->set_params(params);
}

ZedSvoPlaybackCamera::~ZedSvoPlaybackCamera() {
    cancel_open_ = true;
    if (device_->has_pending_open()) {
        if (!device_->await_or_leak_open("ZedSvoPlaybackCamera destructor",
                                         "pending_open_wait_destructor")) {
            return;
        }
    }
    if (is_initialized_) device_->close();
}

void ZedSvoPlaybackCamera::cancel_initialize() { cancel_open_ = true; }

bool ZedSvoPlaybackCamera::initialize() {
    if (is_initialized_) {
        device_->close();
        is_initialized_ = false;
    }
    should_close_ = false;
    stamp_offset_initialized_ = false;
    frames_delivered_ = 0;
    device_->reset_runtime_state();

    const ZedDevice::OpenResult open_result = device_->open(cancel_open_);
    if (open_result == ZedDevice::OpenResult::Leaked) {
        should_close_ = true;
        return false;
    }
    if (open_result != ZedDevice::OpenResult::Opened) return false;

    if (svo_start_frame_ > 0) {
        const int n_frames = device_->svo_frame_count();
        if (n_frames > 0 && svo_start_frame_ >= n_frames) {
            throw ConfigValidationError("rgbd_camera.svo_start_frame is " +
                                        std::to_string(svo_start_frame_) + " but " + svo_path_ +
                                        " has only " + std::to_string(n_frames) +
                                        " frames; set svo_start_frame alongside svo_file_path");
        }
        if (n_frames > 0) {
            spdlog::info("SVO starting at frame {} of {}", svo_start_frame_, n_frames);
            if (svo_decode_to_start_) {
                if (!decode_to_start()) return false;
            } else {
                device_->set_svo_position(svo_start_frame_);
            }
        }
    }

    if (!device_->enable_tracking()) return false;

    const std::optional<CameraInfo> camera_info = device_->read_camera_info();
    if (!camera_info) return false;
    latest_data_.camera_info = *camera_info;
    is_initialized_ = true;
    return true;
}

bool ZedSvoPlaybackCamera::get(CameraData &data) {
    if (!is_initialized_ || should_close_) return false;

    if (svo_max_frames_ > 0 && frames_delivered_ >= svo_max_frames_) {
        spdlog::info("Delivered svo_max_frames = {} frames; closing.", svo_max_frames_);
        should_close_ = true;
        return false;
    }

    const ZedDevice::GrabStatus status = device_->grab();
    if (status == ZedDevice::GrabStatus::EndOfFile) {
        spdlog::info("End of SVO file reached.");
        should_close_ = true;
        return false;
    }
    if (status == ZedDevice::GrabStatus::TransientError) return false;
    if (status != ZedDevice::GrabStatus::Ok) {
        should_close_ = true;
        return false;
    }

    if (!device_->retrieve(latest_data_)) return false;

    const int grabbed_frame_offset = 1;
    const int svo_position = device_->svo_position();
    latest_data_.frame_identity.video_frame_index =
        svo_position > 0 ? svo_position - grabbed_frame_offset : -1;

    if (rebase_stamps_) {
        const double raw_stamp =
            static_cast<double>(latest_data_.frame_identity.image_stamp_ns) / 1e9;
        if (!stamp_offset_initialized_) {
            const double now_s =
                std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
                    .count();
            stamp_offset_s_ = now_s - raw_stamp;
            stamp_offset_initialized_ = true;
        }
        const double stamp = raw_stamp + stamp_offset_s_;
        latest_data_.rgb.header.stamp = stamp;
        latest_data_.depth.header.stamp = stamp;
        latest_data_.camera_info.header.stamp = stamp;
        latest_data_.tf_visodom_from_camera.header.stamp = stamp;
    }

    data = latest_data_;
    ++frames_delivered_;
    return true;
}

bool ZedSvoPlaybackCamera::decode_to_start() {
    while (device_->svo_position() < svo_start_frame_) {
        if (cancel_open_) return false;
        if (device_->grab_without_depth() != ZedDevice::GrabStatus::Ok) {
            spdlog::error("SVO ended at frame {} while decoding to start frame {}",
                          device_->svo_position(), svo_start_frame_);
            should_close_ = true;
            return false;
        }
    }
    return true;
}

bool ZedSvoPlaybackCamera::set_recording_enabled(bool enabled) {
    if (enabled) {
        spdlog::warn("Ignoring SVO recording toggle while using SVO playback input.");
    }
    return false;
}

}  // namespace auto_battlebot
