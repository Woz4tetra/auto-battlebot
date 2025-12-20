#include "ros/ros_message_adapters/ros_point_cloud2.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"
#include <cstring>

namespace auto_battlebot
{
    namespace ros_adapters
    {
        sensor_msgs::PointCloud2 to_ros_point_cloud2(
            const DepthImage& depth_image,
            const RgbImage& rgb_image,
            const CameraInfo& camera_info,
            const Header& header)
        {
            sensor_msgs::PointCloud2 cloud;
            cloud.header = to_ros_header(header);
            cloud.height = depth_image.image.rows;
            cloud.width = depth_image.image.cols;
            cloud.is_bigendian = false;
            cloud.is_dense = false;

            // Define point cloud fields (x, y, z, rgb)
            sensor_msgs::PointField field_x, field_y, field_z, field_rgb;
            
            field_x.name = "x";
            field_x.offset = 0;
            field_x.datatype = sensor_msgs::PointField::FLOAT32;
            field_x.count = 1;
            
            field_y.name = "y";
            field_y.offset = 4;
            field_y.datatype = sensor_msgs::PointField::FLOAT32;
            field_y.count = 1;
            
            field_z.name = "z";
            field_z.offset = 8;
            field_z.datatype = sensor_msgs::PointField::FLOAT32;
            field_z.count = 1;
            
            field_rgb.name = "rgb";
            field_rgb.offset = 12;
            field_rgb.datatype = sensor_msgs::PointField::FLOAT32;
            field_rgb.count = 1;
            
            cloud.fields.push_back(field_x);
            cloud.fields.push_back(field_y);
            cloud.fields.push_back(field_z);
            cloud.fields.push_back(field_rgb);
            
            cloud.point_step = 16;  // 4 floats * 4 bytes
            cloud.row_step = cloud.point_step * cloud.width;
            cloud.data.resize(cloud.row_step * cloud.height);

            // Extract camera intrinsics
            double fx = camera_info.intrinsics.at<double>(0, 0);
            double fy = camera_info.intrinsics.at<double>(1, 1);
            double cx = camera_info.intrinsics.at<double>(0, 2);
            double cy = camera_info.intrinsics.at<double>(1, 2);

            // Fill point cloud data
            for (int v = 0; v < depth_image.image.rows; ++v)
            {
                for (int u = 0; u < depth_image.image.cols; ++u)
                {
                    float depth = depth_image.image.at<float>(v, u);
                    int index = v * cloud.width + u;
                    int offset = index * cloud.point_step;

                    // Calculate 3D point
                    float x = (u - cx) * depth / fx;
                    float y = (v - cy) * depth / fy;
                    float z = depth;

                    // Pack RGB
                    uint8_t r = 128, g = 128, b = 128;
                    if (!rgb_image.image.empty() && v < rgb_image.image.rows && u < rgb_image.image.cols)
                    {
                        cv::Vec3b bgr = rgb_image.image.at<cv::Vec3b>(v, u);
                        b = bgr[0];
                        g = bgr[1];
                        r = bgr[2];
                    }
                    uint32_t rgb = (static_cast<uint32_t>(r) << 16) | 
                                   (static_cast<uint32_t>(g) << 8) | 
                                   static_cast<uint32_t>(b);
                    float rgb_float;
                    std::memcpy(&rgb_float, &rgb, sizeof(float));

                    // Write to cloud data
                    std::memcpy(&cloud.data[offset + 0], &x, sizeof(float));
                    std::memcpy(&cloud.data[offset + 4], &y, sizeof(float));
                    std::memcpy(&cloud.data[offset + 8], &z, sizeof(float));
                    std::memcpy(&cloud.data[offset + 12], &rgb_float, sizeof(float));
                }
            }

            return cloud;
        }

    } // namespace ros_adapters
} // namespace auto_battlebot
