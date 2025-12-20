# C++ Diagnostics Logger

A C++ implementation of the Python diagnostics logger system, using the miniroscpp library to publish ROS diagnostic messages.

## Overview

The diagnostics logger provides a singleton-based system for collecting and publishing diagnostic information from multiple modules in your application. It follows the same design as the Python implementation with these key classes:

-   **DiagnosticsLogger**: Singleton manager that coordinates all module loggers
-   **DiagnosticsModuleLogger**: Individual logger for a specific module
-   **DiagnosticsUtils**: Utility functions for data flattening and message conversion

## Features

-   **Singleton Pattern**: Single global instance manages all module loggers
-   **Module-based Logging**: Each module gets its own logger with independent status
-   **Level Tracking**: Automatically tracks the highest severity level (OK, WARN, ERROR, STALE)
-   **Message Deduplication**: Prevents duplicate messages within a publish cycle
-   **Message Concatenation**: Multiple messages are joined with " | "
-   **Data Flattening**: Nested structures and arrays are flattened with "/" separators
-   **Type Support**: Handles int, double, string, and vectors of these types

## File Structure

```
include/diagnostics_logger/
  ├── diagnostics_logger.hpp          # Main singleton class
  ├── diagnostics_module_logger.hpp   # Individual module logger
  └── diagnostics_utils.hpp           # Data conversion utilities

src/diagnostics_logger/
  ├── diagnostics_logger.cpp
  ├── diagnostics_module_logger.cpp
  └── diagnostics_utils.cpp

playground/
  └── example_diagnostics_usage.cpp   # Example usage
```

## Usage

### 1. Initialize the System

Initialize once at application startup:

```cpp
#include <miniros/ros.h>
#include <miniros/node_handle.h>
#include "diagnostics_logger/diagnostics_logger.hpp"

miniros::NodeHandle nh;
auto publisher = std::make_shared<miniros::Publisher>(
    nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10)
);

DiagnosticsLogger::initialize("my_app_name", publisher);
```

### 2. Get Module Loggers

Get or create loggers for different modules:

```cpp
auto sensor_logger = DiagnosticsLogger::get_logger("sensors");
auto motor_logger = DiagnosticsLogger::get_logger("motors");
```

### 3. Log Diagnostics

Log diagnostic information with various severity levels:

```cpp
// Simple messages
sensor_logger->info({}, "System initialized");
sensor_logger->warning({}, "Temperature high");
sensor_logger->error({}, "Sensor disconnected");

// With data
DiagnosticsData data = {
    {"temperature", 25.5},
    {"voltage", 12.0}
};
sensor_logger->debug(data, "Sensor readings");

// Arrays/vectors are automatically flattened
DiagnosticsData complex_data = {
    {"temperatures", std::vector<int>{95, 94, 90}},
    {"voltages", std::vector<double>{12.5, 12.3, 12.1}}
};
motor_logger->error(complex_data, "Multiple motor issues");
```

### 4. Publish Periodically

Call `publish()` from a single location (e.g., timer callback):

```cpp
// Option 1: Manual loop
while (ros::ok()) {
    // ... do work ...
    DiagnosticsLogger::publish();
    rate.sleep();
}

// Option 2: Timer callback
miniros::Timer timer = nh.createTimer(
    miniros::Duration(1.0),  // 1 Hz
    [](const miniros::TimerEvent&) {
        DiagnosticsLogger::publish();
    }
);
```

## Data Format Examples

### Simple Key-Value

```cpp
DiagnosticsData data = {
    {"temperature", 25.5},
    {"status", "ok"}
};
```

Results in:

```
temperature: 25.5
status: ok
```

### Arrays

```cpp
DiagnosticsData data = {
    {"temperatures", std::vector<int>{95, 94, 90}}
};
```

Results in:

```
temperatures/0: 95
temperatures/1: 94
temperatures/2: 90
```

### Multiple Messages

```cpp
logger->warning({}, "Low battery");
logger->error({}, "GPS signal lost");
```

Results in single diagnostic status with message:

```
"Low battery | GPS signal lost"
```

## API Reference

### DiagnosticsLogger

Static methods for managing the diagnostics system:

-   `static void initialize(const std::string& app_name, std::shared_ptr<miniros::Publisher> publisher)`
-   `static bool is_initialized()`
-   `static std::shared_ptr<DiagnosticsModuleLogger> get_logger(const std::string& name)`
-   `static void remove_logger(const std::string& name)`
-   `static void publish()`

### DiagnosticsModuleLogger

Instance methods for logging:

-   `void debug(const DiagnosticsData& data = {}, const std::string& message = "")`
-   `void info(const DiagnosticsData& data = {}, const std::string& message = "")`
-   `void warning(const DiagnosticsData& data = {}, const std::string& message = "")`
-   `void error(const DiagnosticsData& data = {}, const std::string& message = "")`
-   `void clear()`
-   `bool has_status() const`

### DiagnosticsData Type

```cpp
using DiagnosticsValue = std::variant<
    int,
    double,
    std::string,
    std::vector<int>,
    std::vector<double>,
    std::vector<std::string>
>;

using DiagnosticsData = std::map<std::string, DiagnosticsValue>;
```

## Implementation Notes

### Differences from Python Version

1. **Type System**: Uses `std::variant` instead of Python's dynamic typing
2. **Memory Management**: Uses `std::shared_ptr` for logger instances
3. **Thread Safety**: Not thread-safe by default (add mutexes if needed)
4. **Nested Dictionaries**: Current implementation supports one level of nesting via arrays

### Limitations

-   Does not support arbitrary nesting of dictionaries (only arrays within flat dictionaries)
-   No automatic header timestamp creation (uses `miniros::Time::now()`)
-   Not thread-safe without additional synchronization

### Future Enhancements

To support nested dictionaries, you would need to:

1. Create a recursive variant type
2. Modify `flatten_diagnostics_data()` to handle nested maps
3. Update the flattening logic in `diagnostics_utils.cpp`

## Building

The diagnostics logger is automatically included in the main `auto_battlebot` library. No special build steps are required.

## Dependencies

-   miniroscpp (ROS C++ client library)
-   diagnostic_msgs (ROS message definitions)
-   C++17 or later (for `std::variant`)

## Integration with Existing Code

The diagnostics logger integrates seamlessly with your existing miniroscpp-based application. Simply initialize it once and use loggers throughout your codebase. The singleton pattern ensures all loggers share the same publisher and coordinate their output.
