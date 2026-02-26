#include "diagnostics_logger/initialize_diagnostics_logger.hpp"
#include "diagnostics_logger/ros_diagnostics_backend.hpp"
#include <diagnostic_msgs/DiagnosticArray.hxx>

namespace auto_battlebot
{
    void initialize_diagnostics_logger(miniros::NodeHandle &nh)
    {
        auto diagnostics_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100));
        auto ros_backend = std::make_shared<RosDiagnosticsBackend>(diagnostics_publisher);
        DiagnosticsLogger::initialize({ros_backend});
    }
}