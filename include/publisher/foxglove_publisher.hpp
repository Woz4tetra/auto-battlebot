#pragma once

#include <memory>
#include <string>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "publisher/publisher_interface.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {

/**
 * Publishes pipeline output as Foxglove schemas to the live relay (via VizSink) and the MCAP
 * recorder. Each message is encoded once and fanned out by OutputChannel. Either sink may be
 * null: with no relay the app still records, with no recorder it still streams.
 */
class FoxglovePublisher : public PublisherInterface {
   public:
    FoxglovePublisher(std::shared_ptr<VizSink> sink, std::shared_ptr<McapRecorder> mcap_recorder);

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

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};

}  // namespace auto_battlebot
