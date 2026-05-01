#pragma once

#include <sensor_msgs/CameraInfo.hxx>
#include <sensor_msgs/CompressedImage.hxx>
#include <tf2_msgs/TFMessage.hxx>
#include <visualization_msgs/MarkerArray.hxx>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/publisher_interface.hpp"
#include "ros/typed_publisher.hpp"

namespace auto_battlebot {
class RosPublisher : public PublisherInterface {
   public:
    RosPublisher(TypedPublisher<sensor_msgs::CompressedImage> rgb_image_publisher,
                 TypedPublisher<sensor_msgs::CameraInfo> camera_info_publisher,
                 TypedPublisher<sensor_msgs::CompressedImage> field_mask_publisher,
                 TypedPublisher<tf2_msgs::TFMessage> tf_publisher,
                 TypedPublisher<tf2_msgs::TFMessage> static_tf_publisher,
                 TypedPublisher<visualization_msgs::MarkerArray> field_marker_publisher,
                 TypedPublisher<visualization_msgs::MarkerArray> robot_marker_publisher,
                 TypedPublisher<visualization_msgs::MarkerArray> nav_marker_publisher,
                 std::shared_ptr<McapRecorder> mcap_recorder  // optional, may be null
    );
    void publish_camera_data(const CameraData &data) override;
    void publish_field_mask(const MaskStamped &field_mask, const RgbImage &image) override;
    void publish_initial_field_description(const FieldDescriptionWithInlierPoints &field) override;
    void publish_field_description(
        const FieldDescription &field_description,
        const FieldDescriptionWithInlierPoints &initial_field_description) override;
    void publish_robots(const RobotDescriptionsStamped &robots) override;
    void publish_navigation(const NavigationVisualization &nav) override;

   private:
    TypedPublisher<sensor_msgs::CompressedImage> rgb_image_publisher_;
    TypedPublisher<sensor_msgs::CameraInfo> camera_info_publisher_;
    TypedPublisher<sensor_msgs::CompressedImage> field_mask_publisher_;
    TypedPublisher<tf2_msgs::TFMessage> tf_publisher_;
    TypedPublisher<tf2_msgs::TFMessage> static_tf_publisher_;
    TypedPublisher<visualization_msgs::MarkerArray> field_marker_publisher_;
    TypedPublisher<visualization_msgs::MarkerArray> robot_marker_publisher_;
    TypedPublisher<visualization_msgs::MarkerArray> nav_marker_publisher_;

    std::shared_ptr<McapRecorder> mcap_recorder_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
