#pragma once

#include "diagnostics_logger/diagnostics_logger.hpp"
#include <diagnostic_msgs/DiagnosticArray.hxx>

namespace auto_battlebot
{
    /**
     * @brief Mock publisher for testing that matches miniros::Publisher interface
     *
     * This mock doesn't require ROS initialization and simply records
     * publish calls for verification in tests.
     */
    class MockPublisher
    {
    public:
        MockPublisher() = default;

        void publish(const diagnostic_msgs::DiagnosticArray &msg)
        {
            last_published_message_ = msg;
            publish_count_++;
        }

        const diagnostic_msgs::DiagnosticArray &get_last_message() const
        {
            return last_published_message_;
        }

        int get_publish_count() const
        {
            return publish_count_;
        }

        void reset()
        {
            publish_count_ = 0;
            last_published_message_ = diagnostic_msgs::DiagnosticArray();
        }

    private:
        diagnostic_msgs::DiagnosticArray last_published_message_;
        int publish_count_ = 0;
    };

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
