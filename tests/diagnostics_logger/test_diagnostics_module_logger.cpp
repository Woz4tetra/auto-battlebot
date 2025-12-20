#include <gtest/gtest.h>
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include <diagnostic_msgs/DiagnosticStatus.hxx>

namespace auto_battlebot
{
    class DiagnosticsModuleLoggerTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            logger = std::make_unique<DiagnosticsModuleLogger>("test_app", "test_module");
        }

        void TearDown() override
        {
            logger.reset();
        }

        std::unique_ptr<DiagnosticsModuleLogger> logger;
    };

    // Test initial state
    TEST_F(DiagnosticsModuleLoggerTest, InitialState)
    {
        EXPECT_FALSE(logger->has_status());
    }

    // Test debug logging
    TEST_F(DiagnosticsModuleLoggerTest, DebugLogging)
    {
        logger->debug({{"temp", 25}}, "Debug message");

        EXPECT_TRUE(logger->has_status());
        auto status = logger->get_status();
        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::OK);
        EXPECT_EQ(status.name, "test_module");
        EXPECT_EQ(status.message, "Debug message");
        EXPECT_EQ(status.hardware_id, "test_app");
    }

    // Test info logging
    TEST_F(DiagnosticsModuleLoggerTest, InfoLogging)
    {
        logger->info({{"value", 100}}, "Info message");

        EXPECT_TRUE(logger->has_status());
        auto status = logger->get_status();
        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::OK);
        EXPECT_EQ(status.message, "Info message");
    }

    // Test warning logging
    TEST_F(DiagnosticsModuleLoggerTest, WarningLogging)
    {
        logger->warning({{"temp", 85}}, "Temperature high");

        EXPECT_TRUE(logger->has_status());
        auto status = logger->get_status();
        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::WARN);
        EXPECT_EQ(status.message, "Temperature high");
    }

    // Test error logging
    TEST_F(DiagnosticsModuleLoggerTest, ErrorLogging)
    {
        logger->error({{"error_code", 123}}, "Hardware failure");

        EXPECT_TRUE(logger->has_status());
        auto status = logger->get_status();
        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::ERROR);
        EXPECT_EQ(status.message, "Hardware failure");
    }

    // Test level escalation
    TEST_F(DiagnosticsModuleLoggerTest, LevelEscalation)
    {
        logger->debug({}, "Debug");
        EXPECT_EQ(logger->get_status().level, diagnostic_msgs::DiagnosticStatus::OK);
        logger->clear();

        logger->info({}, "Info");
        logger->warning({}, "Warning");
        EXPECT_EQ(logger->get_status().level, diagnostic_msgs::DiagnosticStatus::WARN);
        logger->clear();

        logger->debug({}, "Debug");
        logger->warning({}, "Warning");
        logger->error({}, "Error");
        EXPECT_EQ(logger->get_status().level, diagnostic_msgs::DiagnosticStatus::ERROR);
    }

    // Test message concatenation
    TEST_F(DiagnosticsModuleLoggerTest, MessageConcatenation)
    {
        logger->warning({}, "Low battery");
        logger->error({}, "GPS signal lost");

        auto status = logger->get_status();
        EXPECT_EQ(status.message, "Low battery | GPS signal lost");
        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::ERROR);
    }

    // Test duplicate message prevention
    TEST_F(DiagnosticsModuleLoggerTest, DuplicateMessagePrevention)
    {
        logger->warning({}, "Duplicate message");
        logger->warning({}, "Duplicate message");
        logger->warning({}, "Duplicate message");

        auto status = logger->get_status();
        EXPECT_EQ(status.message, "Duplicate message");
    }

    // Test data accumulation
    TEST_F(DiagnosticsModuleLoggerTest, DataAccumulation)
    {
        logger->info({{"temp", 25}});
        logger->info({{"voltage", 12.5}});

        auto status = logger->get_status();
        EXPECT_EQ(status.values.size(), 2);
    }

    // Test data overwriting
    TEST_F(DiagnosticsModuleLoggerTest, DataOverwriting)
    {
        logger->info({{"temp", 25}});
        logger->info({{"temp", 30}}); // Should overwrite

        auto status = logger->get_status();

        // Find temp value
        bool found = false;
        for (const auto &kv : status.values)
        {
            if (kv.key == "temp")
            {
                EXPECT_EQ(kv.value, "30");
                found = true;
            }
        }
        EXPECT_TRUE(found);
    }

    // Test clear functionality
    TEST_F(DiagnosticsModuleLoggerTest, ClearFunctionality)
    {
        logger->error({{"temp", 100}}, "Error message");
        EXPECT_TRUE(logger->has_status());

        logger->clear();
        EXPECT_FALSE(logger->has_status());
    }

    // Test logging with empty message
    TEST_F(DiagnosticsModuleLoggerTest, EmptyMessage)
    {
        logger->info({{"value", 1}});

        auto status = logger->get_status();
        EXPECT_EQ(status.message, "");
        EXPECT_EQ(status.values.size(), 1);
    }

    // Test logging with empty data
    TEST_F(DiagnosticsModuleLoggerTest, EmptyData)
    {
        logger->info({}, "Message only");

        auto status = logger->get_status();
        EXPECT_EQ(status.message, "Message only");
        EXPECT_EQ(status.values.size(), 0);
    }

    // Test logging with both empty data and message
    TEST_F(DiagnosticsModuleLoggerTest, BothEmpty)
    {
        logger->info();

        EXPECT_FALSE(logger->has_status());
    }

    // Test complex data with arrays
    TEST_F(DiagnosticsModuleLoggerTest, ComplexDataWithArrays)
    {
        DiagnosticsData data = {
            {"temperatures", std::vector<int>{95, 94, 90}},
            {"voltage", 12.5}};
        logger->error(data, "Multiple issues");

        auto status = logger->get_status();
        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::ERROR);
        EXPECT_GE(status.values.size(), 4); // 3 temps + 1 voltage
    }

    // Test multiple loggers don't interfere
    TEST_F(DiagnosticsModuleLoggerTest, MultipleLoggers)
    {
        auto logger1 = std::make_unique<DiagnosticsModuleLogger>("app1", "module1");
        auto logger2 = std::make_unique<DiagnosticsModuleLogger>("app2", "module2");

        logger1->info({{"value", 1}}, "Logger 1");
        logger2->error({{"value", 2}}, "Logger 2");

        auto status1 = logger1->get_status();
        auto status2 = logger2->get_status();

        EXPECT_EQ(status1.name, "module1");
        EXPECT_EQ(status1.hardware_id, "app1");
        EXPECT_EQ(status1.level, diagnostic_msgs::DiagnosticStatus::OK);

        EXPECT_EQ(status2.name, "module2");
        EXPECT_EQ(status2.hardware_id, "app2");
        EXPECT_EQ(status2.level, diagnostic_msgs::DiagnosticStatus::ERROR);
    }

    // Test status after get_status (should still have status until cleared)
    TEST_F(DiagnosticsModuleLoggerTest, StatusPersistsUntilClear)
    {
        logger->info({{"value", 1}}, "Test");

        EXPECT_TRUE(logger->has_status());
        auto status1 = logger->get_status();
        EXPECT_TRUE(logger->has_status()); // Still has status

        logger->clear();
        EXPECT_FALSE(logger->has_status());
    }

    // Test mixed message types
    TEST_F(DiagnosticsModuleLoggerTest, MixedMessageTypes)
    {
        logger->debug({{"d1", 1}}, "Debug msg");
        logger->info({{"i1", 2}}, "Info msg");
        logger->warning({{"w1", 3}}, "Warning msg");

        auto status = logger->get_status();
        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::WARN);
        EXPECT_EQ(status.message, "Debug msg | Info msg | Warning msg");
        EXPECT_EQ(status.values.size(), 3);
    }

} // namespace auto_battlebot
