#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "rgbd_camera/camera_calibration.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/grab_health_monitor.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/video_encoder.hpp"

namespace auto_battlebot {
/**
 * @brief e-CAM25_CUONX over raw V4L2, rectified in the camera, recorded as H.264.
 *
 * Raw ioctls rather than cv::VideoCapture or GStreamer. cv::VideoCapture hides the buffer count
 * and does not expose `v4l2_buffer.timestamp`, which is the kernel's monotonic capture instant and
 * the thing `sl::TIME_REFERENCE::IMAGE` used to give us; the ManualClock replay path depends on
 * the frame stamp being that instant. GStreamer with nvv4l2camerasrc adds a dependency and a
 * queue of unknown depth, and queue depth is latency in a 60 ms budget.
 *
 * `CameraData::depth` is left empty. Nothing crashes on that: KeypointHeightGate abstains,
 * to_field_point_cloud returns nullopt and the publisher already handles it, and the field filter
 * is one of the RGB ones. `tf_visodom_from_camera` is identity, because a clamped camera does not
 * move.
 */
class V4l2RgbCamera : public RgbdCameraInterface {
   public:
    V4l2RgbCamera(const V4l2RgbCameraConfiguration &config,
                  std::shared_ptr<McapRecorder> mcap_recorder);
    ~V4l2RgbCamera() override;

    bool initialize() override;
    bool get(CameraData &data) override;
    bool should_close() override { return should_close_.load(); }
    bool set_recording_enabled(bool enabled) override;
    bool is_recording_enabled() const override;

   private:
    struct MappedBuffer {
        void *start = nullptr;
        size_t length = 0;
    };

    bool open_device();
    bool negotiate_format();
    bool apply_controls();
    bool map_buffers();
    bool start_streaming();
    void stop_streaming();
    void unmap_buffers();
    void close_device();
    void capture_thread_loop();
    bool capture_frame();
    /** Dequeue every queued buffer and keep the newest. More buffers means the pipeline reads
     *  staler frames when it falls behind, so the queue is drained rather than consumed in order.
     */
    int dequeue_newest(uint64_t &capture_time_ns);

    V4l2RgbCameraConfiguration config_;
    int fd_ = -1;
    int width_ = 0;
    int height_ = 0;
    std::vector<MappedBuffer> buffers_;

    CameraCalibration calibration_;
    Rectifier rectifier_;

    CameraData latest_data_;
    cv::Mat latest_raw_;
    mutable std::mutex data_mutex_;
    mutable std::condition_variable data_cv_;
    std::thread capture_thread_;

    std::atomic<bool> is_initialized_{false};
    std::atomic<bool> should_close_{false};
    std::atomic<bool> stop_thread_{false};
    std::atomic<uint64_t> frame_counter_{0};
    mutable uint64_t last_returned_frame_counter_ = 0;
    std::atomic<int64_t> video_frame_index_{-1};

    GrabHealthMonitor grab_health_;

    std::shared_ptr<McapRecorder> mcap_recorder_;
    std::unique_ptr<OutputChannel> video_channel_;
    VideoEncoder encoder_;
    std::atomic<bool> recording_desired_{true};
    uint64_t reported_drops_ = 0;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
