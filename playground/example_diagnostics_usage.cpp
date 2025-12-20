#include <miniros/ros.h>
#include <miniros/node_handle.h>
#include "diagnostics_logger/diagnostics_logger.hpp"

/**
 * Example usage of the DiagnosticsLogger system
 *
 * This demonstrates how to:
 * 1. Initialize the diagnostics logger with a ROS publisher
 * 2. Create module-specific loggers
 * 3. Log various types of diagnostic information
 * 4. Publish diagnostics periodically
 */

namespace auto_battlebot
{
    void example_diagnostics_usage()
    {
        // 1. Initialize ROS and create a publisher
        miniros::NodeHandle nh;
        auto diagnostics_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10));

        // 2. Initialize the DiagnosticsLogger singleton
        DiagnosticsLogger::initialize("example_app", diagnostics_publisher);

        // 3. Get loggers for different modules
        auto sensor_logger = DiagnosticsLogger::get_logger("sensors");
        auto motor_logger = DiagnosticsLogger::get_logger("motors");
        auto navigation_logger = DiagnosticsLogger::get_logger("navigation");

        // 4. Log various types of diagnostics

        // Simple info message
        sensor_logger->info({}, "Sensor system initialized");

        // Debug with data
        DiagnosticsData sensor_data = {
            {"temperature", 25.5},
            {"humidity", 65}};
        sensor_logger->debug(sensor_data);

        // Warning with message
        motor_logger->warning({}, "Motor temperature high");

        // Error with data and message
        DiagnosticsData error_data = {
            {"error_code", 123},
            {"voltage", 11.5}};
        motor_logger->error(error_data, "Motor voltage below threshold");

        // Complex nested data with arrays
        DiagnosticsData complex_data = {
            {"temperatures", std::vector<int>{95, 94, 90}},
            {"voltage", 12.5},
            {"current", 3.2}};
        motor_logger->error(complex_data, "Multiple motor issues detected");

        // Multiple messages will be concatenated
        navigation_logger->warning({}, "Low battery");
        navigation_logger->error({}, "GPS signal lost");
        // Results in: "Low battery | GPS signal lost"

        // 5. Publish all accumulated diagnostics
        // This should be called periodically (e.g., in a timer callback)
        DiagnosticsLogger::publish();

        // After publish(), all loggers are cleared and ready for the next cycle
    }

    void example_with_ros_timer()
    {
        miniros::NodeHandle nh;

        // Setup publisher
        auto diagnostics_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10));

        // Initialize
        DiagnosticsLogger::initialize("my_robot", diagnostics_publisher);

        // Create a timer to publish diagnostics at 1Hz
        miniros::Timer timer = nh.createTimer(
            miniros::Duration(1.0),
            [](const miniros::TimerEvent &)
            {
                DiagnosticsLogger::publish();
            });

        // Main loop - loggers can be used throughout the application
        miniros::Rate rate(10); // 10 Hz
        while (miniros::ok())
        {
            auto logger = DiagnosticsLogger::get_logger("main_loop");

            // Do work and log diagnostics...
            logger->debug({{"loop_count", 1}}, "Loop iteration");

            miniros::spinOnce();
            rate.sleep();
        }
    }

} // namespace auto_battlebot

int main(int argc, char **argv)
{
    // Initialize ROS
    miniros::init(argc, argv, "diagnostics_example");

    // Run example
    auto_battlebot::example_diagnostics_usage();

    return 0;
}
