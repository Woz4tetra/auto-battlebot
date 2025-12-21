#pragma once

#include <miniros/ros.h>
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot
{
    void initialize_diagnostics_logger(miniros::NodeHandle &nh);

} // namespace auto_battlebot
