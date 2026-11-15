#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "rgbd_camera/camera_calibration.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/video_decoder.hpp"

namespace auto_battlebot {
class VideoPlaybackCamera : public RgbdCameraInterface {
   public:
    explicit VideoPlaybackCamera(const VideoPlaybackCameraConfiguration &config);
    ~VideoPlaybackCamera() override;

    bool initialize() override;
    bool get(CameraData &data) override;
    bool should_close() override { return should_close_; }

    const std::string &recorded_calibration_id() const { return recorded_calibration_id_; }

   private:
    struct VideoFrameIndex {
        uint64_t log_time_ns = 0;
        bool keyframe = false;
    };

    bool scan_file();
    bool advance_to_next_frame(cv::Mat &bgr, uint64_t &log_time_ns, uint64_t &image_stamp_ns);
    void pace(double stamp_seconds);

    VideoPlaybackCameraConfiguration config_;
    std::string video_file_path_;
    std::string recorded_calibration_id_;
    std::string recorded_calibration_toml_;

    struct ReaderState;
    std::unique_ptr<ReaderState> state_;
    std::vector<VideoFrameIndex> frame_index_;
    /** image_stamp_ns by video frame ordinal, from /camera/frame_meta.
     *
     * Joined on the ordinal, never on the timestamp. The two channels are written by different
     * threads and carry different clocks: /camera/video's log time is the capture instant the
     * encoder carried through, while frame_meta is logged when the publisher gets to it. Pairing
     * them by iteration order silently ran eleven frames ahead. */
    std::unordered_map<int64_t, uint64_t> image_stamp_by_ordinal_;

    VideoDecoder decoder_;
    CameraCalibration calibration_;
    Rectifier rectifier_;
    CameraInfo camera_info_;

    int64_t next_frame_ordinal_ = 0;
    bool should_close_ = false;
    bool stamp_offset_initialized_ = false;
    double stamp_offset_s_ = 0.0;

    /** Real-time pacing anchor: the wall-clock instant one frame stamp was played at. Every later
     *  frame is due at that instant plus its offset in the recording's own timeline. Pacing has to
     *  work against a deadline rather than sleeping an interval per frame, or the pipeline's work
     *  on each frame lands on top of the interval instead of inside it. */
    bool pace_anchored_ = false;
    std::chrono::steady_clock::time_point pace_anchor_{};
    double pace_anchor_stamp_s_ = 0.0;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
