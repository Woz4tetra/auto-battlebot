# Diagnostics Logger Unit Tests

Comprehensive unit tests for the C++ Diagnostics Logger implementation.

## Test Summary

**Total Tests: 47**

-   ✅ DiagnosticsUtilsTest: 13 tests
-   ✅ DiagnosticsModuleLoggerTest: 18 tests
-   ✅ DiagnosticsLoggerTest: 16 tests

**Status: All tests passing ✓**

## Test Files

### 1. test_diagnostics_utils.cpp (13 tests)

Tests for the utility functions that flatten and convert data structures.

**Coverage:**

-   ✅ Flattening simple key-value pairs
-   ✅ Flattening integer vectors
-   ✅ Flattening double vectors
-   ✅ Flattening string vectors
-   ✅ Flattening mixed data types
-   ✅ Flattening empty data
-   ✅ Flattening empty vectors
-   ✅ Converting data to DiagnosticStatus
-   ✅ Converting data with arrays
-   ✅ Different diagnostic levels (OK, WARN, ERROR, STALE)
-   ✅ Empty data conversion
-   ✅ Custom separator support
-   ✅ Parent key parameter support

### 2. test_diagnostics_module_logger.cpp (18 tests)

Tests for the individual module logger functionality.

**Coverage:**

-   ✅ Initial state validation
-   ✅ Debug level logging
-   ✅ Info level logging
-   ✅ Warning level logging
-   ✅ Error level logging
-   ✅ Level escalation (OK → WARN → ERROR)
-   ✅ Message concatenation with " | " separator
-   ✅ Duplicate message prevention
-   ✅ Data accumulation across multiple logs
-   ✅ Data overwriting for same keys
-   ✅ Clear functionality
-   ✅ Logging with empty message
-   ✅ Logging with empty data
-   ✅ Logging with both empty
-   ✅ Complex data with arrays
-   ✅ Multiple independent loggers
-   ✅ Status persistence until cleared
-   ✅ Mixed message types

### 3. test_diagnostics_logger.cpp (16 tests)

Tests for the singleton manager that coordinates all loggers.

**Coverage:**

-   ✅ Initialization process
-   ✅ Double initialization throws error
-   ✅ Creating new loggers
-   ✅ Logger instance reuse
-   ✅ Logger removal
-   ✅ Publishing without initialization throws
-   ✅ Publishing with no statuses
-   ✅ Publishing clears loggers
-   ✅ Multiple modules with different levels
-   ✅ Logger data accumulation
-   ✅ Empty logger handling
-   ✅ Mixed empty and non-empty loggers
-   ✅ Publish cycle behavior
-   ✅ Logger persistence across publishes
-   ✅ Removing non-existent logger
-   ✅ Multiple sequential publishes

## Key Test Features

### Data Type Testing

Tests cover all supported data types:

-   `int`
-   `double`
-   `std::string`
-   `std::vector<int>`
-   `std::vector<double>`
-   `std::vector<std::string>`

### Level Testing

Tests validate all diagnostic levels:

-   `OK` (0)
-   `WARN` (1)
-   `ERROR` (2)
-   `STALE` (3)

### Edge Cases

-   Empty data
-   Empty messages
-   Empty vectors
-   Duplicate messages
-   Multiple loggers
-   State persistence
-   Singleton behavior

## Running the Tests

### Build and Run All Tests

```bash
./scripts/build_and_test.sh
```

### Run Only Diagnostics Tests

```bash
cd build
./auto_battlebot_test --gtest_filter="Diagnostics*"
```

### Run Specific Test Suite

```bash
./auto_battlebot_test --gtest_filter="DiagnosticsUtilsTest.*"
./auto_battlebot_test --gtest_filter="DiagnosticsModuleLoggerTest.*"
./auto_battlebot_test --gtest_filter="DiagnosticsLoggerTest.*"
```

### Run Single Test

```bash
./auto_battlebot_test --gtest_filter="DiagnosticsLoggerTest.Initialization"
```

## Test Implementation Notes

### ROS Time Initialization

Tests that call `DiagnosticsLogger::publish()` require ROS time to be initialized:

```cpp
void SetUp() override {
    miniros::Time::init();
    // ... rest of setup
}
```

### Singleton Reset

The `DiagnosticsLogger` singleton needs to be reset between tests to ensure clean state.
A test-only subclass `TestDiagnosticsLogger` is used for this purpose:

```cpp
// In test header:
#include "test_diagnostics_logger.hpp"

void TearDown() override {
    TestDiagnosticsLogger::reset();
    // ... rest of teardown
}
```

**Important:** The `TestDiagnosticsLogger` class should ONLY be used in tests. The production
`DiagnosticsLogger` class does not have a reset method to prevent misuse.

### Mock Publisher

Tests use a mock publisher that doesn't actually publish to ROS:

```cpp
mock_publisher = std::make_shared<miniros::Publisher>();
DiagnosticsLogger::initialize("test_app", mock_publisher);
```

## Test Patterns

### Testing Level Escalation

```cpp
logger->debug({}, "Debug");    // Level: OK
logger->warning({}, "Warning"); // Level: WARN (escalated)
logger->error({}, "Error");     // Level: ERROR (escalated)
EXPECT_EQ(logger->get_status().level, DiagnosticStatus::ERROR);
```

### Testing Message Concatenation

```cpp
logger->warning({}, "Low battery");
logger->error({}, "GPS signal lost");
auto status = logger->get_status();
EXPECT_EQ(status.message, "Low battery | GPS signal lost");
```

### Testing Data Flattening

```cpp
DiagnosticsData data = {
    {"temperatures", std::vector<int>{95, 94, 90}},
    {"voltage", 12.5}
};
auto flattened = flatten_diagnostics_data(data);
EXPECT_EQ(flattened["temperatures/0"], "95");
EXPECT_EQ(flattened["temperatures/1"], "94");
EXPECT_EQ(flattened["temperatures/2"], "90");
EXPECT_EQ(flattened["voltage"], "12.500000");
```

## Coverage Gaps

The current tests do NOT cover:

-   Actual ROS message publishing (requires real ROS node)
-   Thread safety (would require threading tests)
-   Deep nested dictionary structures (current implementation only supports one level)
-   Very large data sets (performance testing)
-   Network failure scenarios
-   Publisher failure handling

## Future Test Improvements

1. **Integration Tests**: Test actual publishing to ROS topics
2. **Performance Tests**: Validate performance with large datasets
3. **Thread Safety Tests**: If multi-threading is added
4. **Mock Publisher**: Create a proper mock that captures published messages
5. **Coverage Reporting**: Add code coverage metrics
6. **Benchmark Tests**: Measure performance characteristics

## Contributing

When adding new features to the diagnostics logger:

1. Add corresponding unit tests
2. Ensure all existing tests still pass
3. Update this documentation
4. Follow existing test patterns
5. Test both success and failure cases
