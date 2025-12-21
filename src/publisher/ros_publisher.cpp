#include "publisher/ros_publisher.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"
#include "ros/ros_message_adapters/ros_camera_info.hpp"
#include "ros/ros_message_adapters/ros_marker.hpp"
#include "ros/ros_message_adapters/ros_tf2.hpp"

namespace auto_battlebot
{
    RosPublisher::RosPublisher(
        std::shared_ptr<miniros::Publisher> rgb_image_publisher,       // sensor_msgs::Image
        std::shared_ptr<miniros::Publisher> camera_info_publisher,     // sensor_msgs::CameraInfo
        std::shared_ptr<miniros::Publisher> field_mask_publisher,      // sensor_msgs::Image
        std::shared_ptr<miniros::Publisher> tf_publisher,              // tf2_msgs::TFMessage
        std::shared_ptr<miniros::Publisher> static_tf_publisher,       // tf2_msgs::TFMessage
        std::shared_ptr<miniros::Publisher> field_marker_publisher,    // visualization_msgs::Marker
        std::shared_ptr<miniros::Publisher> keypoint_marker_publisher, // visualization_msgs::Marker
        std::shared_ptr<miniros::Publisher> robot_marker_publisher     // visualization_msgs::Marker
    )
    {
        rgb_image_publisher_ = rgb_image_publisher;
        camera_info_publisher_ = camera_info_publisher;
        field_mask_publisher_ = field_mask_publisher;
        tf_publisher_ = tf_publisher;
        static_tf_publisher_ = static_tf_publisher;
        field_marker_publisher_ = field_marker_publisher;
        keypoint_marker_publisher_ = keypoint_marker_publisher;
        robot_marker_publisher_ = robot_marker_publisher;
    }

    void RosPublisher::publish_camera_data(const CameraData &data)
    {
        // Publish RGB image
        if (rgb_image_publisher_)
        {
            auto rgb_msg = ros_adapters::to_ros_image(data.rgb, data.tf_visodom_from_camera.header);
            rgb_image_publisher_->publish(rgb_msg);
        }

        // Publish camera info
        if (camera_info_publisher_)
        {
            auto camera_info_msg = ros_adapters::to_ros_camera_info(data.camera_info, data.tf_visodom_from_camera.header);
            camera_info_publisher_->publish(camera_info_msg);
        }

        // Publish transform
        if (tf_publisher_)
        {
            auto tf_msg = ros_adapters::to_ros_tf_message(data.tf_visodom_from_camera);
            tf_publisher_->publish(tf_msg);
        }
    }

    void RosPublisher::publish_field_mask(const FieldMaskStamped &field_mask)
    {
        if (field_mask_publisher_)
        {
            // Convert mask to ROS image
            RgbImage mask_as_image;
            cv::Mat rgb_mask;
            // Convert single channel mask to 3-channel for visualization
            cv::cvtColor(field_mask.mask.mask, rgb_mask, cv::COLOR_GRAY2BGR);
            mask_as_image.image = rgb_mask;

            auto mask_msg = ros_adapters::to_ros_image(mask_as_image, field_mask.header);
            field_mask_publisher_->publish(mask_msg);
        }
    }

    void RosPublisher::publish_field_description(const FieldDescription &field)
    {
        if (field_marker_publisher_)
        {
            auto marker = ros_adapters::to_ros_field_marker(field);
            field_marker_publisher_->publish(marker);
        }

        if (static_tf_publisher_)
        {
            // Publish static transform for field
            TransformStamped field_tf;
            field_tf.header = field.header;
            field_tf.child_frame_id = FrameId::FIELD;
            field_tf.transform = field.tf_fieldcenter_from_camera;

            auto tf_msg = ros_adapters::to_ros_tf_message(field_tf);
            static_tf_publisher_->publish(tf_msg);
        }
    }

    void RosPublisher::publish_keypoints(const KeypointsStamped &keypoints)
    {
        if (keypoint_marker_publisher_)
        {
            auto markers = ros_adapters::to_ros_keypoint_markers(keypoints);
            for (const auto &marker : markers)
            {
                keypoint_marker_publisher_->publish(marker);
            }
        }
    }

    void RosPublisher::publish_robots(const RobotDescriptionsStamped &robots)
    {
        if (robot_marker_publisher_)
        {
            auto markers = ros_adapters::to_ros_robot_markers(robots);
            for (const auto &marker : markers)
            {
                robot_marker_publisher_->publish(marker);
            }
        }

        if (tf_publisher_)
        {
            // Publish transforms for each robot
            tf2_msgs::TFMessage tf_msg;
            for (const auto &robot : robots.descriptions)
            {
                TransformStamped robot_tf;
                robot_tf.header = robots.header;
                robot_tf.child_frame_id = static_cast<FrameId>(static_cast<int>(robot.label));

                // Convert pose to transform
                Eigen::MatrixXd tf_matrix = Eigen::MatrixXd::Identity(4, 4);
                tf_matrix(0, 3) = robot.pose.position.x;
                tf_matrix(1, 3) = robot.pose.position.y;
                tf_matrix(2, 3) = robot.pose.position.z;

                // Convert quaternion to rotation matrix
                Eigen::Quaterniond quat(robot.pose.rotation.w, robot.pose.rotation.x,
                                        robot.pose.rotation.y, robot.pose.rotation.z);
                tf_matrix.block<3, 3>(0, 0) = quat.toRotationMatrix();

                robot_tf.transform.tf = tf_matrix;

                tf_msg.transforms.push_back(ros_adapters::to_ros_transform_stamped(robot_tf));
            }

            if (!tf_msg.transforms.empty())
            {
                tf_publisher_->publish(tf_msg);
            }
        }
    }

} // namespace auto_battlebot
