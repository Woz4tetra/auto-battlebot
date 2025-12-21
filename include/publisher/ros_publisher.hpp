#pragma once

#include <miniros/publisher.h>
#include "publisher/publisher_interface.hpp"

namespace auto_battlebot
{
    class RosPublisher : public PublisherInterface
    {
    public:
        RosPublisher(
            std::shared_ptr<miniros::Publisher> rgb_image_publisher,       // sensor_msgs::Image
            std::shared_ptr<miniros::Publisher> camera_info_publisher,     // sensor_msgs::CameraInfo
            std::shared_ptr<miniros::Publisher> field_mask_publisher,      // sensor_msgs::Image
            std::shared_ptr<miniros::Publisher> tf_publisher,              // tf2_msgs::TFMessage
            std::shared_ptr<miniros::Publisher> static_tf_publisher,       // tf2_msgs::TFMessage
            std::shared_ptr<miniros::Publisher> field_marker_publisher,    // visualization_msgs::Marker
            std::shared_ptr<miniros::Publisher> keypoint_marker_publisher, // visualization_msgs::Marker
            std::shared_ptr<miniros::Publisher> robot_marker_publisher     // visualization_msgs::Marker
        );
        void publish_camera_data(const CameraData &data) override;
        void publish_field_mask(const FieldMaskStamped &field_mask, const RgbImage &image) override;
        void publish_field_description(const FieldDescription &field) override;
        void publish_keypoints(const KeypointsStamped &keypoints) override;
        void publish_robots(const RobotDescriptionsStamped &robots) override;

    private:
        std::shared_ptr<miniros::Publisher> rgb_image_publisher_;
        std::shared_ptr<miniros::Publisher> camera_info_publisher_;
        std::shared_ptr<miniros::Publisher> field_mask_publisher_;
        std::shared_ptr<miniros::Publisher> tf_publisher_;
        std::shared_ptr<miniros::Publisher> static_tf_publisher_;
        std::shared_ptr<miniros::Publisher> field_marker_publisher_;
        std::shared_ptr<miniros::Publisher> keypoint_marker_publisher_;
        std::shared_ptr<miniros::Publisher> robot_marker_publisher_;
    };
} // namespace auto_battlebot
