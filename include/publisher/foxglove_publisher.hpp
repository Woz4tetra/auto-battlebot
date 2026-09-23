#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/config.hpp"
#include "publisher/output_channel.hpp"
#include "publisher/publisher_interface.hpp"
#include "rgbd_camera/video_encoder.hpp"
#include "viz/image_encoder_worker.hpp"
#include "viz/jpeg_encoder.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {

/**
 * Publishes pipeline output as Foxglove schemas to the live relay (via VizSink) and the MCAP
 * recorder. Each message is encoded once and fanned out by OutputChannel. Either sink may be
 * null: with no relay the app still records, with no recorder it still streams.
 */
class FoxglovePublisher : public PublisherInterface {
   public:
    FoxglovePublisher(std::shared_ptr<VizSink> sink, std::shared_ptr<McapRecorder> mcap_recorder,
                      const FoxglovePublisherConfiguration &config = {});

    void publish_camera_data(const CameraData &data) override;
    void publish_field_mask(const MaskStamped &field_mask, const RgbImage &image,
                            const CameraInfo &camera_info) override;
    void publish_initial_field_description(const FieldDescriptionWithInlierPoints &field) override;
    void publish_field_description(
        const FieldDescription &field_description,
        const FieldDescriptionWithInlierPoints &initial_field_description) override;
    void publish_hazards(const FieldDescription &field_description) override;
    void publish_robots(const RobotDescriptionsStamped &robots) override;
    void publish_blob_detections(const DetectionsStamped &detections) override;
    void publish_keypoint_detections(const DetectionsStamped &detections) override;
    void publish_navigation(const NavigationVisualization &nav) override;

   private:
    // Shared impl for /blob_detections and /keypoint_detections plus their annotation topics.
    void publish_detections_on(OutputChannel &json_channel, OutputChannel &annotations_channel,
                               const DetectionsStamped &detections);
    /** Loop-side half of /camera/preview and /camera/preview_video: rate limit, then one resize
     *  into a fresh buffer shared by both. */
    void submit_previews(const RgbImage &rgb, uint64_t log_time);
    cv::Mat resize_for_preview(const cv::Mat &image) const;
    void submit_preview_video(const cv::Mat &frame, uint64_t log_time);
    void log_encoder_stats();

    std::shared_ptr<VizSink> sink_;
    std::shared_ptr<McapRecorder> mcap_recorder_;

    OutputChannel rgb_image_;
    OutputChannel camera_info_;
    OutputChannel frame_meta_;
    OutputChannel tf_;
    OutputChannel field_mask_;
    OutputChannel field_mask_camera_info_;
    OutputChannel field_markers_;
    OutputChannel field_points_;
    OutputChannel hazard_markers_;
    OutputChannel robot_markers_;
    OutputChannel nav_markers_;
    OutputChannel blob_detections_;
    OutputChannel blob_annotations_;
    OutputChannel keypoint_detections_;
    OutputChannel keypoint_annotations_;
    /** Downscaled JPEG for the web dashboard. Live only, never recorded. */
    OutputChannel preview_image_;
    /** H.264 preview from NVENC. Only created when FFmpeg has a hardware H.264 encoder: software
     *  H.264 on the Orin CPU costs more than the NVJPG JPEG preview it would replace. */
    std::unique_ptr<OutputChannel> preview_video_;

    FoxglovePublisherConfiguration config_;
    std::unique_ptr<JpegEncoderInterface> field_mask_encoder_;
    std::chrono::steady_clock::time_point last_preview_{};
    std::chrono::steady_clock::time_point last_preview_video_{};
    uint32_t preview_video_subscribers_ = 0;
    bool preview_video_failed_ = false;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    // Declared last so they stop before the channels they log to are destroyed.
    std::unique_ptr<ImageEncoderWorker> image_worker_;
    std::unique_ptr<ImageEncoderWorker> preview_worker_;
    VideoEncoder preview_video_encoder_;
};

}  // namespace auto_battlebot
