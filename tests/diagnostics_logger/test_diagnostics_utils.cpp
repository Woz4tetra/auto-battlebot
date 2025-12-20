#include <gtest/gtest.h>
#include "diagnostics_logger/diagnostics_utils.hpp"
#include <diagnostic_msgs/DiagnosticStatus.hxx>

namespace auto_battlebot
{
    class DiagnosticsUtilsTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
        }

        void TearDown() override
        {
        }
    };

    // Test flattening simple key-value pairs
    TEST_F(DiagnosticsUtilsTest, FlattenSimpleKeyValue)
    {
        DiagnosticsData data = {
            {"temperature", 25},
            {"voltage", 12.5},
            {"status", std::string("ok")}};

        auto flattened = flatten_diagnostics_data(data);

        EXPECT_EQ(flattened.size(), 3);
        EXPECT_EQ(flattened["temperature"], "25");
        EXPECT_EQ(flattened["voltage"], "12.500000");
        EXPECT_EQ(flattened["status"], "ok");
    }

    // Test flattening integer vectors
    TEST_F(DiagnosticsUtilsTest, FlattenIntegerVector)
    {
        DiagnosticsData data = {
            {"temperatures", std::vector<int>{95, 94, 90}}};

        auto flattened = flatten_diagnostics_data(data);

        EXPECT_EQ(flattened.size(), 3);
        EXPECT_EQ(flattened["temperatures/0"], "95");
        EXPECT_EQ(flattened["temperatures/1"], "94");
        EXPECT_EQ(flattened["temperatures/2"], "90");
    }

    // Test flattening double vectors
    TEST_F(DiagnosticsUtilsTest, FlattenDoubleVector)
    {
        DiagnosticsData data = {
            {"voltages", std::vector<double>{12.5, 12.3, 12.1}}};

        auto flattened = flatten_diagnostics_data(data);

        EXPECT_EQ(flattened.size(), 3);
        EXPECT_EQ(flattened["voltages/0"], "12.500000");
        EXPECT_EQ(flattened["voltages/1"], "12.300000");
        EXPECT_EQ(flattened["voltages/2"], "12.100000");
    }

    // Test flattening string vectors
    TEST_F(DiagnosticsUtilsTest, FlattenStringVector)
    {
        DiagnosticsData data = {
            {"sensors", std::vector<std::string>{"lidar", "camera", "imu"}}};

        auto flattened = flatten_diagnostics_data(data);

        EXPECT_EQ(flattened.size(), 3);
        EXPECT_EQ(flattened["sensors/0"], "lidar");
        EXPECT_EQ(flattened["sensors/1"], "camera");
        EXPECT_EQ(flattened["sensors/2"], "imu");
    }

    // Test flattening mixed data types
    TEST_F(DiagnosticsUtilsTest, FlattenMixedTypes)
    {
        DiagnosticsData data = {
            {"temperature", 25},
            {"voltage", 12.5},
            {"status", std::string("ok")},
            {"readings", std::vector<int>{1, 2, 3}}};

        auto flattened = flatten_diagnostics_data(data);

        EXPECT_EQ(flattened.size(), 6);
        EXPECT_EQ(flattened["temperature"], "25");
        EXPECT_EQ(flattened["voltage"], "12.500000");
        EXPECT_EQ(flattened["status"], "ok");
        EXPECT_EQ(flattened["readings/0"], "1");
        EXPECT_EQ(flattened["readings/1"], "2");
        EXPECT_EQ(flattened["readings/2"], "3");
    }

    // Test flattening empty data
    TEST_F(DiagnosticsUtilsTest, FlattenEmptyData)
    {
        DiagnosticsData data = {};

        auto flattened = flatten_diagnostics_data(data);

        EXPECT_EQ(flattened.size(), 0);
    }

    // Test flattening with empty vector
    TEST_F(DiagnosticsUtilsTest, FlattenEmptyVector)
    {
        DiagnosticsData data = {
            {"empty_vector", std::vector<int>{}}};

        auto flattened = flatten_diagnostics_data(data);

        EXPECT_EQ(flattened.size(), 0);
    }

    // Test dict_to_diagnostics with simple data
    TEST_F(DiagnosticsUtilsTest, DictToDiagnosticsSimple)
    {
        DiagnosticsData data = {
            {"temperature", 25},
            {"voltage", 12.5}};

        auto status = dict_to_diagnostics(
            data,
            diagnostic_msgs::DiagnosticStatus::OK,
            "test_module",
            "Test message",
            "test_hardware");

        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::OK);
        EXPECT_EQ(status.name, "test_module");
        EXPECT_EQ(status.message, "Test message");
        EXPECT_EQ(status.hardware_id, "test_hardware");
        EXPECT_EQ(status.values.size(), 2);
    }

    // Test dict_to_diagnostics with arrays
    TEST_F(DiagnosticsUtilsTest, DictToDiagnosticsWithArrays)
    {
        DiagnosticsData data = {
            {"temperatures", std::vector<int>{95, 94, 90}},
            {"voltage", 12.5}};

        auto status = dict_to_diagnostics(data);

        EXPECT_EQ(status.values.size(), 4); // 3 temperatures + 1 voltage

        // Find and verify temperature values
        int temp_count = 0;
        for (const auto &kv : status.values)
        {
            if (kv.key.find("temperatures/") == 0)
            {
                temp_count++;
            }
        }
        EXPECT_EQ(temp_count, 3);
    }

    // Test dict_to_diagnostics with different levels
    TEST_F(DiagnosticsUtilsTest, DictToDiagnosticsLevels)
    {
        DiagnosticsData data = {{"value", 1}};

        auto ok_status = dict_to_diagnostics(data, diagnostic_msgs::DiagnosticStatus::OK);
        EXPECT_EQ(ok_status.level, diagnostic_msgs::DiagnosticStatus::OK);

        auto warn_status = dict_to_diagnostics(data, diagnostic_msgs::DiagnosticStatus::WARN);
        EXPECT_EQ(warn_status.level, diagnostic_msgs::DiagnosticStatus::WARN);

        auto error_status = dict_to_diagnostics(data, diagnostic_msgs::DiagnosticStatus::ERROR);
        EXPECT_EQ(error_status.level, diagnostic_msgs::DiagnosticStatus::ERROR);

        auto stale_status = dict_to_diagnostics(data, diagnostic_msgs::DiagnosticStatus::STALE);
        EXPECT_EQ(stale_status.level, diagnostic_msgs::DiagnosticStatus::STALE);
    }

    // Test dict_to_diagnostics with empty data
    TEST_F(DiagnosticsUtilsTest, DictToDiagnosticsEmpty)
    {
        DiagnosticsData data = {};

        auto status = dict_to_diagnostics(data, diagnostic_msgs::DiagnosticStatus::OK);

        EXPECT_EQ(status.values.size(), 0);
        EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::OK);
    }

    // Test custom separator
    TEST_F(DiagnosticsUtilsTest, FlattenWithCustomSeparator)
    {
        DiagnosticsData data = {
            {"readings", std::vector<int>{1, 2, 3}}};

        auto flattened = flatten_diagnostics_data(data, "", ".");

        EXPECT_EQ(flattened.size(), 3);
        EXPECT_EQ(flattened["readings.0"], "1");
        EXPECT_EQ(flattened["readings.1"], "2");
        EXPECT_EQ(flattened["readings.2"], "3");
    }

    // Test parent key parameter
    TEST_F(DiagnosticsUtilsTest, FlattenWithParentKey)
    {
        DiagnosticsData data = {
            {"temp", 25},
            {"voltage", 12.5}};

        auto flattened = flatten_diagnostics_data(data, "motor");

        EXPECT_EQ(flattened.size(), 2);
        EXPECT_EQ(flattened["motor/temp"], "25");
        EXPECT_EQ(flattened["motor/voltage"], "12.500000");
    }

} // namespace auto_battlebot
