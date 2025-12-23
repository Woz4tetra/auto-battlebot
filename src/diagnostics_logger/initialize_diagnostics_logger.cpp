#include "diagnostics_logger/initialize_diagnostics_logger.hpp"

namespace auto_battlebot
{
    void initialize_diagnostics_logger(miniros::NodeHandle &nh)
    {
        auto diagnostics_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100));

        // Initialize the DiagnosticsLogger singleton
        DiagnosticsLogger::initialize(diagnostics_publisher);
    }
}