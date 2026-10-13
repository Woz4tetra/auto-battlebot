#include <gtest/gtest.h>

#include <chrono>
#include <map>

#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {
class DiagnosticsModuleLoggerTest : public ::testing::Test {
   protected:
    void SetUp() override { logger = std::make_unique<DiagnosticsModuleLogger>("test_module"); }

    void TearDown() override { logger.reset(); }

    std::unique_ptr<DiagnosticsModuleLogger> logger;
};

// Test initial state
TEST_F(DiagnosticsModuleLoggerTest, InitialState) { EXPECT_FALSE(logger->has_status()); }

// Test debug logging
TEST_F(DiagnosticsModuleLoggerTest, DebugLogging) {
    logger->debug("subsection", {{"temp", 25}}, "Debug message");

    EXPECT_TRUE(logger->has_status());
    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::OK);
    EXPECT_EQ(statuses[0].subsection, "subsection");
    EXPECT_EQ(statuses[0].message, "Debug message");
}

// Test info logging
TEST_F(DiagnosticsModuleLoggerTest, InfoLogging) {
    logger->info("subsection", {{"value", 100}}, "Info message");

    EXPECT_TRUE(logger->has_status());
    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::OK);
    EXPECT_EQ(statuses[0].message, "Info message");
}

// Test warning logging
TEST_F(DiagnosticsModuleLoggerTest, WarningLogging) {
    logger->warning("subsection", {{"temp", 85}}, "Temperature high");

    EXPECT_TRUE(logger->has_status());
    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::WARN);
    EXPECT_EQ(statuses[0].message, "Temperature high");
}

// Test error logging
TEST_F(DiagnosticsModuleLoggerTest, ErrorLogging) {
    logger->error("subsection", {{"error_code", 123}}, "Hardware failure");

    EXPECT_TRUE(logger->has_status());
    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::ERROR);
    EXPECT_EQ(statuses[0].message, "Hardware failure");
}

// Test level escalation
TEST_F(DiagnosticsModuleLoggerTest, LevelEscalation) {
    logger->debug("sub", {}, "Debug");
    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::OK);
    logger->clear();

    logger->info("sub", {}, "Info");
    logger->warning("sub", {}, "Warning");
    statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::WARN);
    logger->clear();

    logger->debug("sub", {}, "Debug");
    logger->warning("sub", {}, "Warning");
    logger->error("sub", {}, "Error");
    statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::ERROR);
}

// Test message concatenation within a subsection
TEST_F(DiagnosticsModuleLoggerTest, MessageConcatenation) {
    logger->warning("sub", {}, "Low battery");
    logger->error("sub", {}, "GPS signal lost");

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].message, "Low battery | GPS signal lost");
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::ERROR);
}

// Test duplicate messages are allowed
TEST_F(DiagnosticsModuleLoggerTest, DuplicateMessagesAllowed) {
    logger->warning("sub", {}, "Duplicate message");
    logger->warning("sub", {}, "Duplicate message");
    logger->warning("sub", {}, "Duplicate message");

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].message, "Duplicate message | Duplicate message | Duplicate message");
}

// Test data accumulation
TEST_F(DiagnosticsModuleLoggerTest, DataAccumulation) {
    logger->info("sub", {{"temp", 25}});
    logger->info("sub", {{"voltage", 12.5}});

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].values.size(), 2);
}

// Test data overwriting
TEST_F(DiagnosticsModuleLoggerTest, DataOverwriting) {
    logger->info("sub", {{"temp", 25}});
    logger->info("sub", {{"temp", 30}});  // Should overwrite

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);

    // Find temp value
    ASSERT_TRUE(statuses[0].values.count("temp"));
    EXPECT_EQ(std::get<int>(statuses[0].values.at("temp")), 30);
}

// Test clear functionality
TEST_F(DiagnosticsModuleLoggerTest, ClearFunctionality) {
    logger->error("sub", {{"temp", 100}}, "Error message");
    EXPECT_TRUE(logger->has_status());

    logger->clear();
    EXPECT_FALSE(logger->has_status());
}

// Test logging with empty message
TEST_F(DiagnosticsModuleLoggerTest, EmptyMessage) {
    logger->info("sub", {{"value", 1}});

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].message, "");
    EXPECT_EQ(statuses[0].values.size(), 1);
}

// Test logging with empty data
TEST_F(DiagnosticsModuleLoggerTest, EmptyData) {
    logger->info("sub", {}, "Message only");

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].message, "Message only");
    EXPECT_EQ(statuses[0].values.size(), 0);
}

// Test logging with both empty data and message
TEST_F(DiagnosticsModuleLoggerTest, BothEmpty) {
    logger->info("sub", DiagnosticsData{});

    EXPECT_TRUE(logger->has_status());
}

// Test complex data with arrays
TEST_F(DiagnosticsModuleLoggerTest, ComplexDataWithArrays) {
    DiagnosticsData data = {{"temperatures", std::vector<int>{95, 94, 90}}, {"voltage", 12.5}};
    logger->error("sub", data, "Multiple issues");

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::ERROR);
    EXPECT_GE(statuses[0].values.size(), 4);  // 3 temps + 1 voltage
}

// Test multiple loggers don't interfere
TEST_F(DiagnosticsModuleLoggerTest, MultipleLoggers) {
    auto logger1 = std::make_unique<DiagnosticsModuleLogger>("module1");
    auto logger2 = std::make_unique<DiagnosticsModuleLogger>("module2");

    logger1->info("sub", {{"value", 1}}, "Logger 1");
    logger2->error("sub", {{"value", 2}}, "Logger 2");

    auto statuses1 = logger1->get_snapshots();
    auto statuses2 = logger2->get_snapshots();

    ASSERT_EQ(statuses1.size(), 1);
    ASSERT_EQ(statuses2.size(), 1);
    EXPECT_EQ(statuses1[0].subsection, "sub");
    EXPECT_EQ(statuses1[0].level, DiagnosticLevel::OK);
    EXPECT_EQ(logger1->get_name(), "module1");

    EXPECT_EQ(statuses2[0].subsection, "sub");
    EXPECT_EQ(statuses2[0].level, DiagnosticLevel::ERROR);
    EXPECT_EQ(logger2->get_name(), "module2");
}

// Test status after get_status (should still have status until cleared)
TEST_F(DiagnosticsModuleLoggerTest, StatusPersistsUntilClear) {
    logger->info("sub", {{"value", 1}}, "Test");

    EXPECT_TRUE(logger->has_status());
    auto statuses = logger->get_snapshots();
    EXPECT_TRUE(logger->has_status());  // Still has status

    logger->clear();
    EXPECT_FALSE(logger->has_status());
}

// Test mixed message types in same subsection
TEST_F(DiagnosticsModuleLoggerTest, MixedMessageTypes) {
    logger->debug("sub", {{"d1", 1}}, "Debug msg");
    logger->info("sub", {{"i1", 2}}, "Info msg");
    logger->warning("sub", {{"w1", 3}}, "Warning msg");

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 1);
    EXPECT_EQ(statuses[0].level, DiagnosticLevel::WARN);
    EXPECT_EQ(statuses[0].message, "Debug msg | Info msg | Warning msg");
    EXPECT_EQ(statuses[0].values.size(), 3);
}

// Test multiple subsections with separate messages
TEST_F(DiagnosticsModuleLoggerTest, MultipleSubsections) {
    logger->info("subsection1", {{"value1", 1}}, "Message 1");
    logger->warning("subsection2", {{"value2", 2}}, "Message 2");
    logger->error("subsection1", {{"value3", 3}}, "Message 3");

    auto statuses = logger->get_snapshots();
    ASSERT_EQ(statuses.size(), 2);

    // Find each subsection
    for (const auto &status : statuses) {
        if (status.subsection == "test_module:subsection1") {
            EXPECT_EQ(status.message, "Message 1 | Message 3");
            EXPECT_EQ(status.values.size(), 2);
        } else if (status.subsection == "test_module:subsection2") {
            EXPECT_EQ(status.message, "Message 2");
            EXPECT_EQ(status.values.size(), 1);
        }
    }
}

class StickyDiagnosticsModuleLoggerTest : public ::testing::Test {
   protected:
    std::chrono::steady_clock::time_point now{};
    DiagnosticsModuleLogger logger{"test_module", [this] { return now; }};

    void advance_ms(int ms) { now += std::chrono::milliseconds(ms); }
    std::map<std::string, DiagnosticStatusSnapshot> by_subsection() const {
        std::map<std::string, DiagnosticStatusSnapshot> out;
        for (auto &snap : logger.get_snapshots()) out[snap.subsection] = snap;
        return out;
    }
};

// A subsection logged only on change stays in the snapshots between changes, marked held.
TEST_F(StickyDiagnosticsModuleLoggerTest, UnloggedSubsectionIsHeldWithItsAge) {
    logger.debug("channels", {{"ch0", 992}});
    logger.debug("process", {{"linear", 0.5}});
    auto snaps = by_subsection();
    EXPECT_FALSE(snaps["channels"].held);
    EXPECT_DOUBLE_EQ(snaps["channels"].age_ms, 0.0);

    logger.clear();
    advance_ms(17);
    logger.debug("process", {{"linear", 0.25}});
    advance_ms(3);

    EXPECT_TRUE(logger.has_status());
    snaps = by_subsection();
    ASSERT_EQ(snaps.size(), 2U);
    EXPECT_TRUE(snaps["channels"].held);
    EXPECT_DOUBLE_EQ(snaps["channels"].age_ms, 20.0);
    EXPECT_EQ(std::get<int>(snaps["channels"].values.at("ch0")), 992);
    EXPECT_FALSE(snaps["process"].held);
    EXPECT_DOUBLE_EQ(snaps["process"].age_ms, 3.0);
    EXPECT_DOUBLE_EQ(std::get<double>(snaps["process"].values.at("linear")), 0.25);
}

// The first log of a new window replaces the held subsection instead of merging into it.
TEST_F(StickyDiagnosticsModuleLoggerTest, NewWindowReplacesHeldValuesMessageAndLevel) {
    logger.warning("link", {{"old_key", 1}}, "dongle not open");
    logger.clear();
    logger.info("link", {{"new_key", 2}});

    const auto snaps = by_subsection();
    const auto &link = snaps.at("link");
    EXPECT_EQ(link.level, DiagnosticLevel::OK);
    EXPECT_EQ(link.message, "");
    EXPECT_EQ(link.values.count("old_key"), 0U);
    EXPECT_EQ(link.values.count("new_key"), 1U);
}

// Each subsection keeps its own level, so a held warning does not raise a fresh subsection.
TEST_F(StickyDiagnosticsModuleLoggerTest, LevelIsPerSubsection) {
    logger.warning("link", {}, "dongle has not heard robot");
    logger.debug("process", {{"linear", 0.0}});

    auto snaps = by_subsection();
    EXPECT_EQ(snaps["link"].level, DiagnosticLevel::WARN);
    EXPECT_EQ(snaps["process"].level, DiagnosticLevel::OK);
}

// Only held subsections means nothing new to publish.
TEST_F(StickyDiagnosticsModuleLoggerTest, HeldOnlyHasNoStatus) {
    logger.debug("channels", {{"ch0", 992}});
    logger.clear();
    EXPECT_FALSE(logger.has_status());
    EXPECT_EQ(logger.get_snapshots().size(), 1U);
}

TEST_F(StickyDiagnosticsModuleLoggerTest, HeldSubsectionExpires) {
    logger.debug("initialized", {{"ok", 1}});
    logger.clear();
    advance_ms(9999);
    logger.debug("process", {{"linear", 0.0}});
    EXPECT_EQ(by_subsection().count("initialized"), 1U);

    logger.clear();
    advance_ms(1);
    logger.debug("process", {{"linear", 0.0}});
    EXPECT_EQ(by_subsection().count("initialized"), 0U);

    // Expired subsections are dropped at clear(), not only hidden.
    logger.clear();
    advance_ms(DiagnosticsModuleLogger::SECTION_EXPIRY.count() * 1000);
    logger.clear();
    EXPECT_TRUE(logger.get_snapshots().empty());
}

}  // namespace auto_battlebot
