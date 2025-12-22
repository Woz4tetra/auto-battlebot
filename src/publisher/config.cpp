#include "publisher/config.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(PublisherConfiguration, NoopPublisherConfiguration, "NoopPublisher")
    REGISTER_CONFIG(PublisherConfiguration, RosPublisherConfiguration, "RosPublisher")

    std::unique_ptr<PublisherConfiguration> parse_publisher_config(ConfigParser &parser)
    {
        return ConfigFactory<PublisherConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<PublisherInterface> make_publisher(miniros::NodeHandle &nh, const PublisherConfiguration &config)
    {
        std::cout << "Selected " + config.type + " for Publisher" << std::endl;
        if (config.type == "NoopPublisher")
        {
            return std::make_shared<NoopPublisher>();
        }
        else if (config.type == "RosPublisher")
        {
            auto rgb_image_publisher = std::make_shared<miniros::Publisher>(nh.advertise<sensor_msgs::Image>("/camera/image", 10));
            auto camera_info_publisher = std::make_shared<miniros::Publisher>(nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 10));
            auto field_mask_publisher = std::make_shared<miniros::Publisher>(nh.advertise<sensor_msgs::Image>("/field_mask", 10, true));
            auto tf_publisher = std::make_shared<miniros::Publisher>(nh.advertise<tf2_msgs::TFMessage>("/tf", 10));
            auto static_tf_publisher = std::make_shared<miniros::Publisher>(nh.advertise<tf2_msgs::TFMessage>("/tf_static", 10));
            auto field_marker_publisher = std::make_shared<miniros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/field_markers", 10, true));
            auto keypoint_marker_publisher = std::make_shared<miniros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/keypoint_markers", 10, true));
            auto robot_marker_publisher = std::make_shared<miniros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/robot_markers", 10, true));

            return std::make_shared<RosPublisher>(
                rgb_image_publisher,
                camera_info_publisher,
                field_mask_publisher,
                tf_publisher,
                static_tf_publisher,
                field_marker_publisher,
                keypoint_marker_publisher,
                robot_marker_publisher);
        }
        throw std::invalid_argument("Failed to load Publisher of type " + config.type);
    }
} // namespace auto_battlebot
