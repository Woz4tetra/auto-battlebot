#include "ros/ros_message_adapters/ros_camera_info.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"

namespace auto_battlebot
{
    namespace ros_adapters
    {
        sensor_msgs::CameraInfo to_ros_camera_info(const CameraInfo &camera_info, const Header &header)
        {
            sensor_msgs::CameraInfo ros_camera_info;
            ros_camera_info.header = to_ros_header(header);
            ros_camera_info.height = camera_info.height;
            ros_camera_info.width = camera_info.width;

            // Convert intrinsics matrix (3x3) to K array
            if (camera_info.intrinsics.rows == 3 && camera_info.intrinsics.cols == 3)
            {
                for (int i = 0; i < 9; ++i)
                {
                    ros_camera_info.K[i] = camera_info.intrinsics.at<double>(i / 3, i % 3);
                }
            }

            // Convert distortion coefficients
            if (!camera_info.distortion.empty())
            {
                ros_camera_info.D.resize(camera_info.distortion.total());
                for (size_t i = 0; i < camera_info.distortion.total(); ++i)
                {
                    ros_camera_info.D[i] = camera_info.distortion.at<double>(i);
                }
                ros_camera_info.distortion_model = "plumb_bob";
            }

            // Set R to identity (no rectification by default)
            for (int i = 0; i < 9; ++i)
            {
                ros_camera_info.R[i] = (i % 4 == 0) ? 1.0 : 0.0; // Identity matrix
            }

            // Set P to [K | 0]
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    ros_camera_info.P[row * 4 + col] = ros_camera_info.K[row * 3 + col];
                }
                ros_camera_info.P[row * 4 + 3] = 0.0; // Fourth column is zero
            }

            return ros_camera_info;
        }

    } // namespace ros_adapters
} // namespace auto_battlebot
