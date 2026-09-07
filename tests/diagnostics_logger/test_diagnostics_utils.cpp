#include <gtest/gtest.h>

#include "diagnostics_logger/diagnostics_utils.hpp"

namespace auto_battlebot {
class DiagnosticsUtilsTest : public ::testing::Test {};

// Test flattening simple key-value pairs keeps the types
TEST_F(DiagnosticsUtilsTest, FlattenSimpleKeyValue) {
    DiagnosticsData data = {{"temperature", 25}, {"voltage", 12.5}, {"status", std::string("ok")}};

    auto flattened = flatten_diagnostics_data(data);

    EXPECT_EQ(flattened.size(), 3);
    EXPECT_EQ(std::get<int>(flattened["temperature"]), 25);
    EXPECT_DOUBLE_EQ(std::get<double>(flattened["voltage"]), 12.5);
    EXPECT_EQ(std::get<std::string>(flattened["status"]), "ok");
}

// Test flattening integer vectors
TEST_F(DiagnosticsUtilsTest, FlattenIntegerVector) {
    DiagnosticsData data = {{"temperatures", std::vector<int>{95, 94, 90}}};

    auto flattened = flatten_diagnostics_data(data);

    EXPECT_EQ(flattened.size(), 3);
    EXPECT_EQ(std::get<int>(flattened["temperatures/0"]), 95);
    EXPECT_EQ(std::get<int>(flattened["temperatures/1"]), 94);
    EXPECT_EQ(std::get<int>(flattened["temperatures/2"]), 90);
}

// Test flattening double vectors
TEST_F(DiagnosticsUtilsTest, FlattenDoubleVector) {
    DiagnosticsData data = {{"voltages", std::vector<double>{12.5, 12.3, 12.1}}};

    auto flattened = flatten_diagnostics_data(data);

    EXPECT_EQ(flattened.size(), 3);
    EXPECT_DOUBLE_EQ(std::get<double>(flattened["voltages/0"]), 12.5);
    EXPECT_DOUBLE_EQ(std::get<double>(flattened["voltages/1"]), 12.3);
    EXPECT_DOUBLE_EQ(std::get<double>(flattened["voltages/2"]), 12.1);
}

// Test flattening string vectors
TEST_F(DiagnosticsUtilsTest, FlattenStringVector) {
    DiagnosticsData data = {{"sensors", std::vector<std::string>{"lidar", "camera", "imu"}}};

    auto flattened = flatten_diagnostics_data(data);

    EXPECT_EQ(flattened.size(), 3);
    EXPECT_EQ(std::get<std::string>(flattened["sensors/0"]), "lidar");
    EXPECT_EQ(std::get<std::string>(flattened["sensors/1"]), "camera");
    EXPECT_EQ(std::get<std::string>(flattened["sensors/2"]), "imu");
}

// Test flattening nested maps
TEST_F(DiagnosticsUtilsTest, FlattenNested) {
    DiagnosticsData data = {
        {"motor", DiagnosticsData{{"temperature", std::vector<int>{95, 94}}, {"voltage", 12.5}}}};

    auto flattened = flatten_diagnostics_data(data);

    EXPECT_EQ(flattened.size(), 3);
    EXPECT_EQ(std::get<int>(flattened["motor/temperature/0"]), 95);
    EXPECT_EQ(std::get<int>(flattened["motor/temperature/1"]), 94);
    EXPECT_DOUBLE_EQ(std::get<double>(flattened["motor/voltage"]), 12.5);
}

// Test flattening empty data
TEST_F(DiagnosticsUtilsTest, FlattenEmptyData) {
    DiagnosticsData data = {};
    auto flattened = flatten_diagnostics_data(data);
    EXPECT_EQ(flattened.size(), 0);
}

// Test flattening empty vector
TEST_F(DiagnosticsUtilsTest, FlattenEmptyVector) {
    DiagnosticsData data = {{"empty_vector", std::vector<int>{}}};
    auto flattened = flatten_diagnostics_data(data);
    EXPECT_EQ(flattened.size(), 0);
}

// Test custom separator
TEST_F(DiagnosticsUtilsTest, FlattenWithCustomSeparator) {
    DiagnosticsData data = {{"readings", std::vector<int>{1, 2, 3}}};

    auto flattened = flatten_diagnostics_data(data, "", ".");

    EXPECT_EQ(flattened.size(), 3);
    EXPECT_EQ(std::get<int>(flattened["readings.0"]), 1);
    EXPECT_EQ(std::get<int>(flattened["readings.1"]), 2);
    EXPECT_EQ(std::get<int>(flattened["readings.2"]), 3);
}

// The UI is the only place values become text
TEST_F(DiagnosticsUtilsTest, ScalarToString) {
    EXPECT_EQ(diagnostic_scalar_to_string(DiagnosticScalar{25}), "25");
    EXPECT_EQ(diagnostic_scalar_to_string(DiagnosticScalar{12.5}), "12.500000");
    EXPECT_EQ(diagnostic_scalar_to_string(DiagnosticScalar{std::string("ok")}), "ok");
}

}  // namespace auto_battlebot
