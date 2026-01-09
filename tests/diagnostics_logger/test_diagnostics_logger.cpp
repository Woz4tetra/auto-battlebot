#include <gtest/gtest.h>
#include "test_diagnostics_logger.hpp"

namespace auto_battlebot
{
    class DiagnosticsLoggerTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            miniros::Time::init();

            // Create a mock publisher (casted to the base type for DiagnosticsLogger)
            mock_publisher_ = std::make_shared<MockPublisher>();
        }

        void TearDown() override
        {
            // Reset the singleton state for next test using test-only subclass
            TestDiagnosticsLogger::reset();
            mock_publisher_.reset();
        }

        std::shared_ptr<MockPublisher> mock_publisher_;
    };

    // Test initialization
    TEST_F(DiagnosticsLoggerTest, Initialization)
    {
        EXPECT_FALSE(DiagnosticsLogger::is_initialized());

        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        EXPECT_TRUE(DiagnosticsLogger::is_initialized());
    }

    // Test double initialization throws
    TEST_F(DiagnosticsLoggerTest, DoubleInitializationThrows)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        EXPECT_THROW(
            DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_)),
            std::runtime_error);
    }

    // Test get_logger creates new logger
    TEST_F(DiagnosticsLoggerTest, GetLoggerCreatesNew)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger1 = DiagnosticsLogger::get_logger("module1");
        EXPECT_NE(logger1, nullptr);

        auto logger2 = DiagnosticsLogger::get_logger("module2");
        EXPECT_NE(logger2, nullptr);

        EXPECT_NE(logger1, logger2);
    }

    // Test get_logger returns same instance
    TEST_F(DiagnosticsLoggerTest, GetLoggerReturnsSameInstance)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger1 = DiagnosticsLogger::get_logger("module1");
        auto logger2 = DiagnosticsLogger::get_logger("module1");

        EXPECT_EQ(logger1, logger2);
    }

    // Test remove_logger
    TEST_F(DiagnosticsLoggerTest, RemoveLogger)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger = DiagnosticsLogger::get_logger("module1");
        logger->info("", {{"value", 1}}, "Test");
        EXPECT_TRUE(logger->has_status());

        DiagnosticsLogger::remove_logger("module1");

        // Getting the logger again should create a new one
        auto new_logger = DiagnosticsLogger::get_logger("module1");
        EXPECT_FALSE(new_logger->has_status());
    }

    // Test publish without initialization throws
    TEST_F(DiagnosticsLoggerTest, PublishWithoutInitializationThrows)
    {
        EXPECT_THROW(
            DiagnosticsLogger::publish(),
            std::runtime_error);
    }

    // Test publish with no statuses doesn't crash
    TEST_F(DiagnosticsLoggerTest, PublishWithNoStatuses)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        // Should not throw
        EXPECT_NO_THROW(DiagnosticsLogger::publish());
    }

    // Test publish clears loggers
    TEST_F(DiagnosticsLoggerTest, PublishClearsLoggers)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger1 = DiagnosticsLogger::get_logger("module1");
        auto logger2 = DiagnosticsLogger::get_logger("module2");

        logger1->info("", {{"value", 1}}, "Message 1");
        logger2->warning("", {{"value", 2}}, "Message 2");

        EXPECT_TRUE(logger1->has_status());
        EXPECT_TRUE(logger2->has_status());

        // Publish should clear all loggers
        // Note: This won't actually publish without a real ROS node
        // but it will process the loggers
        EXPECT_NO_THROW(DiagnosticsLogger::publish());

        EXPECT_FALSE(logger1->has_status());
        EXPECT_FALSE(logger2->has_status());
    }

    // Test multiple modules with different levels
    TEST_F(DiagnosticsLoggerTest, MultipleModulesDifferentLevels)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger1 = DiagnosticsLogger::get_logger("sensors");
        auto logger2 = DiagnosticsLogger::get_logger("motors");
        auto logger3 = DiagnosticsLogger::get_logger("navigation");

        logger1->debug("", {{"temp", 25}}, "Sensor OK");
        logger2->warning("", {{"current", 5}}, "High current");
        logger3->error("", {{"error_code", 404}}, "Path not found");

        // All should have status before publish
        EXPECT_TRUE(logger1->has_status());
        EXPECT_TRUE(logger2->has_status());
        EXPECT_TRUE(logger3->has_status());

        EXPECT_NO_THROW(DiagnosticsLogger::publish());

        // All should be cleared after publish
        EXPECT_FALSE(logger1->has_status());
        EXPECT_FALSE(logger2->has_status());
        EXPECT_FALSE(logger3->has_status());
    }

    // Test logger accumulates across multiple calls
    TEST_F(DiagnosticsLoggerTest, LoggerAccumulatesData)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger = DiagnosticsLogger::get_logger("test");

        logger->info("", {{"value1", 1}}, "Message 1");
        logger->warning("", {{"value2", 2}}, "Message 2");
        logger->error("", {{"value3", 3}}, "Message 3");

        auto statuses = logger->get_status();

        ASSERT_EQ(statuses.size(), 1);

        // Should have highest level
        EXPECT_EQ(statuses[0].level, diagnostic_msgs::DiagnosticStatus::ERROR);

        // Should have concatenated messages
        EXPECT_EQ(statuses[0].message, "Message 1 | Message 2 | Message 3");

        // Should have all data
        EXPECT_EQ(statuses[0].values.size(), 3);
    }

    // Test empty logger doesn't publish
    TEST_F(DiagnosticsLoggerTest, EmptyLoggerDoesntPublish)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger = DiagnosticsLogger::get_logger("test");

        EXPECT_FALSE(logger->has_status());

        // Should not throw even with empty logger
        EXPECT_NO_THROW(DiagnosticsLogger::publish());
    }

    // Test mixed empty and non-empty loggers
    TEST_F(DiagnosticsLoggerTest, MixedEmptyAndNonEmptyLoggers)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger1 = DiagnosticsLogger::get_logger("module1");
        auto logger2 = DiagnosticsLogger::get_logger("module2");
        auto logger3 = DiagnosticsLogger::get_logger("module3");

        logger1->info("", {{"value", 1}}, "Has status");
        // logger2 has no status
        logger3->error("", {{"value", 3}}, "Also has status");

        EXPECT_TRUE(logger1->has_status());
        EXPECT_FALSE(logger2->has_status());
        EXPECT_TRUE(logger3->has_status());

        EXPECT_NO_THROW(DiagnosticsLogger::publish());

        // All should be cleared
        EXPECT_FALSE(logger1->has_status());
        EXPECT_FALSE(logger2->has_status());
        EXPECT_FALSE(logger3->has_status());
    }

    // Test publish cycle
    TEST_F(DiagnosticsLoggerTest, PublishCycle)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger = DiagnosticsLogger::get_logger("test");

        // First cycle
        logger->info("", {{"cycle", 1}}, "Cycle 1");
        EXPECT_TRUE(logger->has_status());
        DiagnosticsLogger::publish();
        EXPECT_FALSE(logger->has_status());

        // Second cycle
        logger->warning("", {{"cycle", 2}}, "Cycle 2");
        EXPECT_TRUE(logger->has_status());
        DiagnosticsLogger::publish();
        EXPECT_FALSE(logger->has_status());

        // Third cycle - no logging
        EXPECT_FALSE(logger->has_status());
        DiagnosticsLogger::publish();
        EXPECT_FALSE(logger->has_status());
    }

    // Test logger persistence across publish
    TEST_F(DiagnosticsLoggerTest, LoggerPersistenceAcrossPublish)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger1 = DiagnosticsLogger::get_logger("module1");
        auto logger2 = DiagnosticsLogger::get_logger("module2");

        logger1->info({{"value", 1}});
        DiagnosticsLogger::publish();

        // Get loggers again - should be same instances
        auto logger1_again = DiagnosticsLogger::get_logger("module1");
        auto logger2_again = DiagnosticsLogger::get_logger("module2");

        EXPECT_EQ(logger1, logger1_again);
        EXPECT_EQ(logger2, logger2_again);
    }

    // Test remove non-existent logger doesn't crash
    TEST_F(DiagnosticsLoggerTest, RemoveNonExistentLogger)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        EXPECT_NO_THROW(DiagnosticsLogger::remove_logger("non_existent"));
    }

    // Test multiple sequential publishes
    TEST_F(DiagnosticsLoggerTest, MultipleSequentialPublishes)
    {
        DiagnosticsLogger::initialize(std::reinterpret_pointer_cast<miniros::Publisher>(mock_publisher_));

        auto logger = DiagnosticsLogger::get_logger("test");

        for (int i = 0; i < 5; ++i)
        {
            logger->info("", {{"iteration", i}}, "Iteration " + std::to_string(i));
            EXPECT_TRUE(logger->has_status());
            EXPECT_NO_THROW(DiagnosticsLogger::publish());
            EXPECT_FALSE(logger->has_status());
        }
    }

} // namespace auto_battlebot
