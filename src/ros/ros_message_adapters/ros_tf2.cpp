#include "ros/ros_message_adapters/ros_tf2.hpp"

namespace auto_battlebot
{
    namespace ros_adapters
    {
        geometry_msgs::TransformStamped to_ros_transform_stamped(const TransformStamped &transform)
        {
            geometry_msgs::TransformStamped ros_tf;
            ros_tf.header = to_ros_header(transform.header);
            ros_tf.child_frame_id = enum_to_string_lower(transform.child_frame_id);

            const auto &tf = transform.transform.tf;

            // Extract translation from the transformation matrix
            if (tf.rows() >= 3 && tf.cols() >= 4)
            {
                ros_tf.transform.translation.x = tf(0, 3);
                ros_tf.transform.translation.y = tf(1, 3);
                ros_tf.transform.translation.z = tf(2, 3);

                // Extract rotation from the transformation matrix
                // Convert rotation matrix to quaternion
                Eigen::Matrix3d rotation_matrix = tf.block<3, 3>(0, 0);
                Eigen::Quaterniond quat(rotation_matrix);

                ros_tf.transform.rotation.w = quat.w();
                ros_tf.transform.rotation.x = quat.x();
                ros_tf.transform.rotation.y = quat.y();
                ros_tf.transform.rotation.z = quat.z();
            }
            else
            {
                // Identity transform as fallback
                ros_tf.transform.translation.x = 0.0;
                ros_tf.transform.translation.y = 0.0;
                ros_tf.transform.translation.z = 0.0;
                ros_tf.transform.rotation.w = 1.0;
                ros_tf.transform.rotation.x = 0.0;
                ros_tf.transform.rotation.y = 0.0;
                ros_tf.transform.rotation.z = 0.0;
            }

            return ros_tf;
        }

        tf2_msgs::TFMessage to_ros_tf_message(const TransformStamped &transform)
        {
            tf2_msgs::TFMessage tf_msg;
            tf_msg.transforms.push_back(to_ros_transform_stamped(transform));
            return tf_msg;
        }

    } // namespace ros_adapters
} // namespace auto_battlebot
