#include "rgbd_camera/zed_svo_playback_camera.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <filesystem>

namespace auto_battlebot {

ZedSvoPlaybackCamera::ZedSvoPlaybackCamera(ZedSvoPlaybackCameraConfiguration &config)
    : svo_start_frame_(config.svo_start_frame),
      rebase_stamps_(config.rebase_stamps),
      diagnostics_logger_(DiagnosticsLogger::get_logger("zed_svo_playback_camera")) {
    device_.set_tracking_enabled(config.position_tracking);

    const std::filesystem::path svo_abs_path =
        std::filesystem::absolute(std::filesystem::path(config.svo_file_path.c_str()));
    spdlog::info("Resolved SVO path: {}", svo_abs_path.string());
    svo_path_ = svo_abs_path.string();

    sl::InitParameters params;
    params.camera_fps = config.camera_fps;
    params.camera_resolution = get_zed_resolution(config.camera_resolution);
    params.depth_mode = get_zed_depth_mode(config.depth_mode);
    params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    params.coordinate_units = sl::UNIT::METER;
    params.input.setFromSVOFile(svo_abs_path.c_str());
    params.svo_real_time_mode = config.svo_real_time_mode;
    device_.set_params(params);
}

ZedSvoPlaybackCamera::~ZedSvoPlaybackCamera() {
    cancel_open_ = true;
    if (device_.has_pending_open()) {
        // A leaked open leaves the handle in use by a background SDK call; closing it would race.
        if (!device_.await_or_leak_open("ZedSvoPlaybackCamera destructor",
                                        "pending_open_wait_destructor")) {
            return;
        }
    }
    if (is_initialized_) device_.close();
}

void ZedSvoPlaybackCamera::cancel_initialize() { cancel_open_ = true; }

bool ZedSvoPlaybackCamera::initialize() {
    if (is_initialized_) {
        device_.close();
        is_initialized_ = false;
    }
    should_close_ = false;
    stamp_offset_initialized_ = false;
    device_.reset_runtime_state();

    cancel_open_ = false;
    const ZedDevice::OpenResult open_result = device_.open(cancel_open_);
    if (open_result == ZedDevice::OpenResult::Leaked) {
        should_close_ = true;
        return false;
    }
    if (open_result != ZedDevice::OpenResult::Opened) return false;

    if (svo_start_frame_ > 0) {
        const int n_frames = device_.svo_frame_count();
        if (n_frames > 0) {
            const int frame = std::clamp(svo_start_frame_, 0, n_frames - 1);
            if (frame != svo_start_frame_) {
                spdlog::warn("svo_start_frame {} out of range for this SVO ({} frames), using {}",
                             svo_start_frame_, n_frames, frame);
            }
            spdlog::info("SVO starting at frame {} of {}", frame, n_frames);
            device_.set_svo_position(frame);
        }
    }

    if (!device_.enable_tracking()) return false;

    latest_data_.camera_info = device_.read_camera_info();
    is_initialized_ = true;
    return true;
}

bool ZedSvoPlaybackCamera::get(CameraData &data, bool get_depth) {
    if (!is_initialized_ || should_close_) return false;

    // Grab on the caller's thread. Every frame reaches the pipeline, in order, and depth is
    // retrieved for this frame rather than requested for a later one.
    const ZedDevice::GrabStatus status = device_.grab();
    if (status == ZedDevice::GrabStatus::EndOfFile) {
        spdlog::info("End of SVO file reached.");
        should_close_ = true;
        return false;
    }
    if (status == ZedDevice::GrabStatus::TransientError) return false;
    if (status != ZedDevice::GrabStatus::Ok) {
        // A file does not disconnect, so a real grab error here is not recoverable by retrying.
        should_close_ = true;
        return false;
    }

    if (!device_.retrieve(get_depth, latest_data_)) return false;

    // getSVOPosition() counts frames already read, so the one just grabbed is one behind it.
    const int svo_position = device_.svo_position();
    latest_data_.frame_identity.svo_frame_index = svo_position > 0 ? svo_position - 1 : -1;
    latest_data_.frame_identity.svo_path = svo_path_;

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
    return true;
}

bool ZedSvoPlaybackCamera::set_recording_enabled(bool enabled) {
    if (enabled) {
        spdlog::warn("Ignoring SVO recording toggle while using SVO playback input.");
    }
    return false;
}

}  // namespace auto_battlebot
