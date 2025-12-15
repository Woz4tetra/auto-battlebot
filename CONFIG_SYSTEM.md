# Automatic Configuration System

## Overview

The configuration system now uses automatic registration and field parsing. This eliminates manual boilerplate code when adding new configuration types or fields.

## Key Features

### 1. Automatic Type Registration

-   Configuration types are automatically registered at static initialization time
-   No need to manually update factory functions when adding new types

### 2. Automatic Field Parsing

-   Fields are parsed automatically based on struct layout
-   Adding a new field only requires updating the struct definition

### 3. Automatic Validation

-   All fields are validated based on the `PARSE_FIELD` macros
-   Unknown fields in config files are automatically detected and reported

## How to Add a New Configuration Type

### Example: Adding a new camera type

```cpp
// In include/rgbd_camera/config.hpp
struct RealSenseCameraConfiguration : public RgbdCameraConfiguration
{
    int fps = 30;
    int width = 640;
    int height = 480;
    std::string depth_preset = "High Accuracy";
    bool enable_ir = false;

    RealSenseCameraConfiguration()
    {
        type = "RealSenseCamera";
    }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD(fps)
        PARSE_FIELD(width)
        PARSE_FIELD(height)
        PARSE_FIELD_STRING(depth_preset)
        PARSE_FIELD_BOOL(enable_ir)
    )
    // clang-format on
};
```

```cpp
// In src/rgbd_camera/config.cpp
// Add ONE line to register the type:
REGISTER_CONFIG(RgbdCameraConfiguration, RealSenseCameraConfiguration, "RealSenseCamera")

// Update make_rgbd_camera to instantiate the interface:
std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config)
{
    // ... existing code ...
    else if (config.type == "RealSenseCamera")
    {
        auto &rs_config = static_cast<const RealSenseCameraConfiguration&>(config);
        return std::make_shared<RealSenseCamera>(rs_config);
    }
    // ...
}
```

That's it! The configuration parser will automatically:

-   Parse all fields defined in the struct
-   Use default values if fields are missing from the config file
-   Validate that no unknown fields are present
-   Detect the correct type and instantiate the right configuration object

## Available Field Parse Macros

### Basic Types

-   `PARSE_FIELD(field)` - Optional int field (uses default from struct)
-   `PARSE_FIELD_STRING(field)` - Optional string field
-   `PARSE_FIELD_DOUBLE(field)` - Optional double field
-   `PARSE_FIELD_BOOL(field)` - Optional bool field
-   `PARSE_FIELD_REQUIRED(field)` - Required int field (throws if missing)
-   `PARSE_FIELD_STRING_REQUIRED(field)` - Required string field
-   `PARSE_FIELD_DOUBLE_REQUIRED(field)` - Required double field
-   `PARSE_FIELD_BOOL_REQUIRED(field)` - Required bool field

### Enums

Enums are represented as strings in TOML files and **automatically converted using magic_enum** - no manual conversion functions needed!

-   `PARSE_ENUM(field, EnumType)` - Optional enum field with default
-   `PARSE_ENUM_REQUIRED(field, EnumType)` - Required enum field

**Example:**

```cpp
struct RobotConfiguration
{
    Label robot_label = Label::OPPONENT;  // default
    Group robot_group;  // required

    PARSE_CONFIG_FIELDS(
        PARSE_ENUM(robot_label, Label)
        PARSE_ENUM_REQUIRED(robot_group, Group)
    )
};
```

**TOML:**

```toml
[robot]
robot_label = "MR_STABS_MK1"
robot_group = "OURS"
```

**No manual string conversion functions required!** The enum values in TOML must match the C++ enum names exactly (e.g., `MR_STABS_MK1`, not `mr_stabs_mk1`). Invalid enum values will throw an `std::invalid_argument` exception with a helpful error message.

## Example Configuration Files

### NoopRgbdCamera (minimal config)

```toml
[rgbd_camera]
type = "NoopRgbdCamera"
```

### ZedRgbdCamera (with custom values)

```toml
[rgbd_camera]
type = "ZedRgbdCamera"
camera_fps = 60
camera_resolution = 1080
depth_mode = "ULTRA"
```

### ZedRgbdCamera (using defaults)

```toml
[rgbd_camera]
type = "ZedRgbdCamera"
# All fields will use defaults: fps=30, resolution=720, depth_mode="NEURAL"
```

## How It Works

1. **Registration Phase** (at program startup):

    - `REGISTER_CONFIG` creates a static `ConfigRegistrar` object
    - The registrar adds the type to the `ConfigFactory` singleton
    - Each type gets a creator function and a parser function

2. **Parsing Phase** (when loading config):

    - `parse_rgbd_camera_config()` is called with a `ConfigParser`
    - The parser reads the "type" field
    - The factory creates the correct config object based on type
    - The config object's `parse_fields()` method is called
    - Each `PARSE_FIELD` macro reads the corresponding field from the TOML
    - `validate_no_extra_fields()` ensures no typos or unknown fields

3. **Usage Phase** (when creating interfaces):
    - The parsed config object is passed to the factory function
    - The factory casts to the correct derived type
    - The interface is instantiated with the config

## Complete Example: Configuration with All Field Types

```cpp
// In include/my_module/config.hpp
struct MyModuleConfiguration
{
    std::string type;
    virtual ~MyModuleConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct AdvancedModuleConfiguration : public MyModuleConfiguration
{
    // Basic types
    int sample_rate = 100;
    double gain = 1.5;
    bool enable_logging = true;
    std::string output_path = "/tmp/output";

    // Enum types
    Label target_label = Label::OPPONENT;
    Group team = Group::OURS;

    AdvancedModuleConfiguration()
    {
        type = "AdvancedModule";
    }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD(sample_rate)
        PARSE_FIELD_DOUBLE(gain)
        PARSE_FIELD_BOOL(enable_logging)
        PARSE_FIELD_STRING(output_path)
        PARSE_ENUM(target_label, Label)
        PARSE_ENUM(team, Group)
    )
    // clang-format on
};// In src/my_module/config.cpp
REGISTER_CONFIG(MyModuleConfiguration, AdvancedModuleConfiguration, "AdvancedModule")
```

```toml
# In config file
[my_module]
type = "AdvancedModule"
sample_rate = 200
gain = 2.0
enable_logging = false
output_path = "/data/results"
target_label = "MR_STABS_MK1"
team = "OURS"
```

All fields are automatically:

-   ✅ Parsed from TOML
-   ✅ Type-checked at compile time
-   ✅ Validated (unknown fields rejected)
-   ✅ Converted (enums from strings)
-   ✅ Defaulted if not specified

## Benefits

-   **Less Boilerplate**: Add a struct + one registration line instead of manual parsing logic
-   **Type Safety**: Compile-time checking of field names and types
-   **Maintainability**: Changes to config structures are localized
-   **Extensibility**: Easy to add new types and fields without touching core code
-   **Validation**: Automatic detection of configuration errors
-   **Enum Support**: Automatic string-to-enum conversion with validation
