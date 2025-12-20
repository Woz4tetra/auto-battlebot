#include "ros/initialize_ros_topics.hpp"

namespace auto_battlebot
{
    void initialize_ros_topics(miniros::NodeHandle &nh)
    {
        auto diagnostics_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100));

        // Initialize the DiagnosticsLogger singleton
        DiagnosticsLogger::initialize("auto_battlebot", diagnostics_publisher);
    }
}