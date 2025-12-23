#pragma once

#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot
{
    /**
     * @brief Test-only subclass of DiagnosticsLogger with reset capability
     *
     * This class should ONLY be used in unit tests. It provides a reset()
     * method to clear the singleton state between tests.
     *
     * WARNING: Do not use this class in production code!
     */
    class TestDiagnosticsLogger : public DiagnosticsLogger
    {
    public:
        /**
         * @brief Reset the singleton state for testing
         *
         * Clears all loggers and resets the initialization state.
         * This allows tests to start with a clean slate.
         */
        static void reset()
        {
            loggers_.clear();
            diagnostics_publisher_.reset();
        }
    };

} // namespace auto_battlebot
