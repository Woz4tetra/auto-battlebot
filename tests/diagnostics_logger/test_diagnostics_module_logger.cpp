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
            logger = std::make_unique<DiagnosticsModuleLogger>("test_module");
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
        logger->debug("subsection", {{"temp", 25}}, "Debug message");

        EXPECT_TRUE(logger->has_status());
        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::OK);
        EXPECT_EQ(statuses[0].name, "subsection");
        EXPECT_EQ(statuses[0].message, "Debug message");
    }

    // Test info logging
    TEST_F(DiagnosticsModuleLoggerTest, InfoLogging)
    {
        logger->info("subsection", {{"value", 100}}, "Info message");

        EXPECT_TRUE(logger->has_status());
        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::OK);
        EXPECT_EQ(statuses[0].message, "Info message");
    }

    // Test warning logging
    TEST_F(DiagnosticsModuleLoggerTest, WarningLogging)
    {
        logger->warning("subsection", {{"temp", 85}}, "Temperature high");

        EXPECT_TRUE(logger->has_status());
        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::WARN);
        EXPECT_EQ(statuses[0].message, "Temperature high");
    }

    // Test error logging
    TEST_F(DiagnosticsModuleLoggerTest, ErrorLogging)
    {
        logger->error("subsection", {{"error_code", 123}}, "Hardware failure");

        EXPECT_TRUE(logger->has_status());
        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::ERROR);
        EXPECT_EQ(statuses[0].message, "Hardware failure");
    }

    // Test level escalation
    TEST_F(DiagnosticsModuleLoggerTest, LevelEscalation)
    {
        logger->debug("sub", {}, "Debug");
        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::OK);
        logger->clear();

        logger->info("sub", {}, "Info");
        logger->warning("sub", {}, "Warning");
        statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::WARN);
        logger->clear();

        logger->debug("sub", {}, "Debug");
        logger->warning("sub", {}, "Warning");
        logger->error("sub", {}, "Error");
        statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::ERROR);
    }

    // Test message concatenation within a subsection
    TEST_F(DiagnosticsModuleLoggerTest, MessageConcatenation)
    {
        logger->warning("sub", {}, "Low battery");
        logger->error("sub", {}, "GPS signal lost");

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].message, "Low battery | GPS signal lost");
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::ERROR);
    }

    // Test duplicate messages are allowed
    TEST_F(DiagnosticsModuleLoggerTest, DuplicateMessagesAllowed)
    {
        logger->warning("sub", {}, "Duplicate message");
        logger->warning("sub", {}, "Duplicate message");
        logger->warning("sub", {}, "Duplicate message");

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].message, "Duplicate message | Duplicate message | Duplicate message");
    }

    // Test data accumulation
    TEST_F(DiagnosticsModuleLoggerTest, DataAccumulation)
    {
        logger->info("sub", {{"temp", 25}});
        logger->info("sub", {{"voltage", 12.5}});

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].values.size(), 2);
    }

    // Test data overwriting
    TEST_F(DiagnosticsModuleLoggerTest, DataOverwriting)
    {
        logger->info("sub", {{"temp", 25}});
        logger->info("sub", {{"temp", 30}}); // Should overwrite

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);

        // Find temp value
        bool found = false;
        for (const auto &kv : statuses[0].values)
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
        logger->error("sub", {{"temp", 100}}, "Error message");
        EXPECT_TRUE(logger->has_status());

        logger->clear();
        EXPECT_FALSE(logger->has_status());
    }

    // Test logging with empty message
    TEST_F(DiagnosticsModuleLoggerTest, EmptyMessage)
    {
        logger->info("sub", {{"value", 1}});

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].message, "");
        EXPECT_EQ(statuses[0].values.size(), 1);
    }

    // Test logging with empty data
    TEST_F(DiagnosticsModuleLoggerTest, EmptyData)
    {
        logger->info("sub", {}, "Message only");

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].message, "Message only");
        EXPECT_EQ(statuses[0].values.size(), 0);
    }

    // Test logging with both empty data and message
    TEST_F(DiagnosticsModuleLoggerTest, BothEmpty)
    {
        logger->info("sub", DiagnosticsData{});

        EXPECT_TRUE(logger->has_status());
    }

    // Test complex data with arrays
    TEST_F(DiagnosticsModuleLoggerTest, ComplexDataWithArrays)
    {
        DiagnosticsData data = {
            {"temperatures", std::vector<int>{95, 94, 90}},
            {"voltage", 12.5}};
        logger->error("sub", data, "Multiple issues");

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::ERROR);
        EXPECT_GE(statuses[0].values.size(), 4); // 3 temps + 1 voltage
    }

    // Test multiple loggers don't interfere
    TEST_F(DiagnosticsModuleLoggerTest, MultipleLoggers)
    {
        auto logger1 = std::make_unique<DiagnosticsModuleLogger>("module1");
        auto logger2 = std::make_unique<DiagnosticsModuleLogger>("module2");

        logger1->info("sub", {{"value", 1}}, "Logger 1");
        logger2->error("sub", {{"value", 2}}, "Logger 2");

        auto statuses1 = logger1->get_status();
        auto statuses2 = logger2->get_status();

        ASSERT_EQ(statuses1.size(), 1);
        ASSERT_EQ(statuses2.size(), 1);
        EXPECT_EQ(statuses1[0].name, "sub");
        EXPECT_EQ(statuses1[0].level, diagnostic_msgs::DiagnosticStatus::OK);
        EXPECT_EQ(logger1->get_name(), "module1");

        EXPECT_EQ(statuses2[0].name, "sub");
        EXPECT_EQ(statuses2[0].level, diagnostic_msgs::DiagnosticStatus::ERROR);
        EXPECT_EQ(logger2->get_name(), "module2");
    }

    // Test status after get_status (should still have status until cleared)
    TEST_F(DiagnosticsModuleLoggerTest, StatusPersistsUntilClear)
    {
        logger->info("sub", {{"value", 1}}, "Test");

        EXPECT_TRUE(logger->has_status());
        auto statuses = logger->get_status();
        EXPECT_TRUE(logger->has_status()); // Still has status

        logger->clear();
        EXPECT_FALSE(logger->has_status());
    }

    // Test mixed message types in same subsection
    TEST_F(DiagnosticsModuleLoggerTest, MixedMessageTypes)
    {
        logger->debug("sub", {{"d1", 1}}, "Debug msg");
        logger->info("sub", {{"i1", 2}}, "Info msg");
        logger->warning("sub", {{"w1", 3}}, "Warning msg");

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 1);
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::WARN);
        EXPECT_EQ(statuses[0].message, "Debug msg | Info msg | Warning msg");
        EXPECT_EQ(statuses[0].values.size(), 3);
    }

    // Test multiple subsections with separate messages
    TEST_F(DiagnosticsModuleLoggerTest, MultipleSubsections)
    {
        logger->info("subsection1", {{"value1", 1}}, "Message 1");
        logger->warning("subsection2", {{"value2", 2}}, "Message 2");
        logger->error("subsection1", {{"value3", 3}}, "Message 3");

        auto statuses = logger->get_status();
        ASSERT_EQ(statuses.size(), 2);

        // Find each subsection
        for (const auto &status : statuses)
        {
            if (status.name == "test_module:subsection1")
            {
                EXPECT_EQ(status.message, "Message 1 | Message 3");
                EXPECT_EQ(status.values.size(), 2);
            }
            else if (status.name == "test_module:subsection2")
            {
                EXPECT_EQ(status.message, "Message 2");
                EXPECT_EQ(status.values.size(), 1);
            }
        }
    }

} // namespace auto_battlebot
