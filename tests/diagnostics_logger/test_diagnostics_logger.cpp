#include "test_diagnostics_logger.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

namespace auto_battlebot {
class DiagnosticsLoggerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        TestDiagnosticsLogger::enable_test_mode();

        backend_ = std::make_shared<RecordingBackend>();
    }

    void TearDown() override {
        TestDiagnosticsLogger::reset();
        backend_.reset();
    }

    std::shared_ptr<RecordingBackend> backend_;
};

TEST_F(DiagnosticsLoggerTest, Initialization) {
    EXPECT_FALSE(DiagnosticsLogger::is_initialized());

    DiagnosticsLogger::initialize({backend_});

    EXPECT_TRUE(DiagnosticsLogger::is_initialized());
}

TEST_F(DiagnosticsLoggerTest, DoubleInitializationThrows) {
    DiagnosticsLogger::initialize({backend_});

    EXPECT_THROW(DiagnosticsLogger::initialize({backend_}), std::runtime_error);
}

TEST_F(DiagnosticsLoggerTest, GetLoggerCreatesNew) {
    DiagnosticsLogger::initialize({backend_});

    auto logger1 = DiagnosticsLogger::get_logger("module1");
    EXPECT_NE(logger1, nullptr);

    auto logger2 = DiagnosticsLogger::get_logger("module2");
    EXPECT_NE(logger2, nullptr);

    EXPECT_NE(logger1, logger2);
}

TEST_F(DiagnosticsLoggerTest, GetLoggerReturnsSameInstance) {
    DiagnosticsLogger::initialize({backend_});

    auto logger1 = DiagnosticsLogger::get_logger("module1");
    auto logger2 = DiagnosticsLogger::get_logger("module1");

    EXPECT_EQ(logger1, logger2);
}

TEST_F(DiagnosticsLoggerTest, RemoveLogger) {
    DiagnosticsLogger::initialize({backend_});

    auto logger = DiagnosticsLogger::get_logger("module1");
    logger->info("", {{"value", 1}}, "Test");
    EXPECT_TRUE(logger->has_status());

    DiagnosticsLogger::remove_logger("module1");

    auto new_logger = DiagnosticsLogger::get_logger("module1");
    EXPECT_FALSE(new_logger->has_status());
}

TEST_F(DiagnosticsLoggerTest, PublishWithoutInitializationThrows) {
    EXPECT_THROW(DiagnosticsLogger::publish(), std::runtime_error);
}

TEST_F(DiagnosticsLoggerTest, PublishWithNoStatuses) {
    DiagnosticsLogger::initialize({backend_});

    EXPECT_NO_THROW(DiagnosticsLogger::publish());
}

TEST_F(DiagnosticsLoggerTest, PublishClearsLoggers) {
    DiagnosticsLogger::initialize({backend_});

    auto logger1 = DiagnosticsLogger::get_logger("module1");
    auto logger2 = DiagnosticsLogger::get_logger("module2");

    logger1->info("", {{"value", 1}}, "Message 1");
    logger2->warning("", {{"value", 2}}, "Message 2");

    EXPECT_TRUE(logger1->has_status());
    EXPECT_TRUE(logger2->has_status());

    EXPECT_NO_THROW(DiagnosticsLogger::publish());

    EXPECT_FALSE(logger1->has_status());
    EXPECT_FALSE(logger2->has_status());
}

TEST_F(DiagnosticsLoggerTest, MultipleModulesDifferentLevels) {
    DiagnosticsLogger::initialize({backend_});

    auto logger1 = DiagnosticsLogger::get_logger("sensors");
    auto logger2 = DiagnosticsLogger::get_logger("motors");
    auto logger3 = DiagnosticsLogger::get_logger("navigation");

    logger1->debug("", {{"temp", 25}}, "Sensor OK");
    logger2->warning("", {{"current", 5}}, "High current");
    logger3->error("", {{"error_code", 404}}, "Path not found");

    EXPECT_TRUE(logger1->has_status());
    EXPECT_TRUE(logger2->has_status());
    EXPECT_TRUE(logger3->has_status());

    EXPECT_NO_THROW(DiagnosticsLogger::publish());

    EXPECT_FALSE(logger1->has_status());
    EXPECT_FALSE(logger2->has_status());
    EXPECT_FALSE(logger3->has_status());
}

TEST_F(DiagnosticsLoggerTest, LoggerAccumulatesData) {
    DiagnosticsLogger::initialize({backend_});

    auto logger = DiagnosticsLogger::get_logger("test");

    logger->info("", {{"value1", 1}}, "Message 1");
    logger->warning("", {{"value2", 2}}, "Message 2");
    logger->error("", {{"value3", 3}}, "Message 3");

    auto statuses = logger->get_snapshots();

    ASSERT_EQ(statuses.size(), 1);

    EXPECT_EQ(statuses[0].level, DiagnosticLevel::ERROR);

    EXPECT_EQ(statuses[0].message, "Message 1 | Message 2 | Message 3");

    EXPECT_EQ(statuses[0].values.size(), 3);
}

TEST_F(DiagnosticsLoggerTest, EmptyLoggerDoesntPublish) {
    DiagnosticsLogger::initialize({backend_});

    auto logger = DiagnosticsLogger::get_logger("test");

    EXPECT_FALSE(logger->has_status());

    EXPECT_NO_THROW(DiagnosticsLogger::publish());
}

TEST_F(DiagnosticsLoggerTest, MixedEmptyAndNonEmptyLoggers) {
    DiagnosticsLogger::initialize({backend_});

    auto logger1 = DiagnosticsLogger::get_logger("module1");
    auto logger2 = DiagnosticsLogger::get_logger("module2");
    auto logger3 = DiagnosticsLogger::get_logger("module3");

    logger1->info("", {{"value", 1}}, "Has status");
    logger3->error("", {{"value", 3}}, "Also has status");

    EXPECT_TRUE(logger1->has_status());
    EXPECT_FALSE(logger2->has_status());
    EXPECT_TRUE(logger3->has_status());

    EXPECT_NO_THROW(DiagnosticsLogger::publish());

    EXPECT_FALSE(logger1->has_status());
    EXPECT_FALSE(logger2->has_status());
    EXPECT_FALSE(logger3->has_status());
}

TEST_F(DiagnosticsLoggerTest, PublishCycle) {
    DiagnosticsLogger::initialize({backend_});

    auto logger = DiagnosticsLogger::get_logger("test");

    logger->info("", {{"cycle", 1}}, "Cycle 1");
    EXPECT_TRUE(logger->has_status());
    DiagnosticsLogger::publish();
    EXPECT_FALSE(logger->has_status());

    logger->warning("", {{"cycle", 2}}, "Cycle 2");
    EXPECT_TRUE(logger->has_status());
    DiagnosticsLogger::publish();
    EXPECT_FALSE(logger->has_status());

    EXPECT_FALSE(logger->has_status());
    DiagnosticsLogger::publish();
    EXPECT_FALSE(logger->has_status());
}

TEST_F(DiagnosticsLoggerTest, LoggerPersistenceAcrossPublish) {
    DiagnosticsLogger::initialize({backend_});

    auto logger1 = DiagnosticsLogger::get_logger("module1");
    auto logger2 = DiagnosticsLogger::get_logger("module2");

    logger1->info({{"value", 1}});
    DiagnosticsLogger::publish();

    auto logger1_again = DiagnosticsLogger::get_logger("module1");
    auto logger2_again = DiagnosticsLogger::get_logger("module2");

    EXPECT_EQ(logger1, logger1_again);
    EXPECT_EQ(logger2, logger2_again);
}

TEST_F(DiagnosticsLoggerTest, RemoveNonExistentLogger) {
    DiagnosticsLogger::initialize({backend_});

    EXPECT_NO_THROW(DiagnosticsLogger::remove_logger("non_existent"));
}

TEST_F(DiagnosticsLoggerTest, MultipleSequentialPublishes) {
    DiagnosticsLogger::initialize({backend_});

    auto logger = DiagnosticsLogger::get_logger("test");

    for (int i = 0; i < 5; ++i) {
        logger->info("", {{"iteration", i}}, "Iteration " + std::to_string(i));
        EXPECT_TRUE(logger->has_status());
        EXPECT_NO_THROW(DiagnosticsLogger::publish());
        EXPECT_FALSE(logger->has_status());
    }
}

TEST_F(DiagnosticsLoggerTest, GetLoggerWhilePublishingIsSafe) {
    DiagnosticsLogger::initialize({backend_});
    for (int i = 0; i < 20; ++i) {
        DiagnosticsLogger::get_logger("existing_" + std::to_string(i))->info("", {{"i", i}}, "x");
    }
    std::atomic<bool> done{false};
    std::thread creator([&] {
        for (int i = 0; i < 20000; ++i) {
            DiagnosticsLogger::get_logger("new_" + std::to_string(i))->info("", {{"i", i}}, "x");
        }
        done.store(true);
    });
    int publishes = 0;
    while (!done.load()) {
        DiagnosticsLogger::publish();
        ++publishes;
    }
    creator.join();
    DiagnosticsLogger::publish();
    EXPECT_GT(publishes, 0);
    EXPECT_NE(DiagnosticsLogger::get_logger("new_19999"), nullptr);
}

class ReentrantBackend : public DiagnosticsBackend {
   public:
    void receive(const std::vector<DiagnosticStatusSnapshot> &) override {
        DiagnosticsLogger::get_logger("created_from_backend");
        receive_count.fetch_add(1);
    }

    std::atomic<int> receive_count{0};
};

TEST_F(DiagnosticsLoggerTest, BackendMayCallGetLoggerDuringPublish) {
    TestDiagnosticsLogger::reset();
    auto backend = std::make_shared<ReentrantBackend>();
    DiagnosticsLogger::initialize({backend});
    DiagnosticsLogger::get_logger("module")->info("", {{"i", 1}}, "x");

    auto finished = std::make_shared<std::promise<void>>();
    auto future = finished->get_future();
    std::thread([finished] {
        DiagnosticsLogger::publish();
        finished->set_value();
    }).detach();

    ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
        << "publish deadlocked with a backend that re-enters get_logger";
    EXPECT_EQ(backend->receive_count.load(), 1);
}

}  // namespace auto_battlebot
