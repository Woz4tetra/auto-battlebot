#include <gtest/gtest.h>
#include "transmitter/sim_transmitter.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "../diagnostics_logger/test_diagnostics_logger.hpp"

namespace auto_battlebot
{
    class SimTransmitterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Enable test mode to skip actual ROS publishing
            TestDiagnosticsLogger::enable_test_mode();
        }

        void TearDown() override
        {
            TestDiagnosticsLogger::reset();
        }

        SimTransmitterConfiguration config_;
    };

    // Test basic initialization
    TEST_F(SimTransmitterTest, InitializeSimTransmitter)
    {
        SimTransmitter transmitter(config_);

        EXPECT_TRUE(transmitter.initialize());
    }
} // namespace auto_battlebot
