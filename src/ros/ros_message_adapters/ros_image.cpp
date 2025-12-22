#include "ros/ros_message_adapters/ros_image.hpp"

namespace auto_battlebot
{
    namespace ros_adapters
    {
        std_msgs::Header to_ros_header(const Header &header)
        {
            std_msgs::Header ros_header;
            ros_header.stamp = miniros::Time(header.stamp);
            ros_header.frame_id = enum_to_string_lower(header.frame_id);
            return ros_header;
        }

        sensor_msgs::Image to_ros_image(const RgbImage &rgb_image)
        {
            sensor_msgs::Image ros_image;
            ros_image.header = to_ros_header(rgb_image.header);
            ros_image.height = rgb_image.image.rows;
            ros_image.width = rgb_image.image.cols;
            ros_image.encoding = sensor_msgs::image_encodings::BGR8;
            ros_image.is_bigendian = false;
            ros_image.step = rgb_image.image.cols * rgb_image.image.elemSize();

            size_t data_size = ros_image.step * ros_image.height;
            ros_image.data.resize(data_size);
            std::memcpy(ros_image.data.data(), rgb_image.image.data, data_size);

            return ros_image;
        }

        sensor_msgs::Image to_ros_image(const DepthImage &depth_image)
        {
            sensor_msgs::Image ros_image;
            ros_image.header = to_ros_header(depth_image.header);
            ros_image.height = depth_image.image.rows;
            ros_image.width = depth_image.image.cols;
            ros_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            ros_image.is_bigendian = false;
            ros_image.step = depth_image.image.cols * depth_image.image.elemSize();

            size_t data_size = ros_image.step * ros_image.height;
            ros_image.data.resize(data_size);
            std::memcpy(ros_image.data.data(), depth_image.image.data, data_size);

            return ros_image;
        }

    } // namespace ros_adapters
} // namespace auto_battlebot
