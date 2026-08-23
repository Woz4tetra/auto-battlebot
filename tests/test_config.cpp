#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "config/config.hpp"
#include "config/config_parser.hpp"
#include "config/profile_selection.hpp"
#include "field_filter/config.hpp"
#include "keypoint_model/config.hpp"
#include "mask_model/config.hpp"
#include "navigation/config.hpp"
#include "publisher/config.hpp"
#include "rgbd_camera/config.hpp"
#include "robot_filter/config.hpp"
#include "transmitter/config.hpp"

namespace auto_battlebot {

class ConfigTest : public ::testing::Test {
   protected:
    std::filesystem::path temp_dir;
    std::filesystem::path temp_config_file;

    void SetUp() override {
        // Create a temporary directory for test configs
        temp_dir = std::filesystem::temp_directory_path() / "auto_battlebot_test_configs";
        std::filesystem::create_directories(temp_dir);
        temp_config_file = temp_dir / "test_config.toml";
    }

    void TearDown() override {
        // Clean up temporary directory
        if (std::filesystem::exists(temp_dir)) {
            std::filesystem::remove_all(temp_dir);
        }
    }

    void write_config_file(const std::string &content) {
        std::ofstream file(temp_config_file);
        file << content;
        file.close();
    }

    // Write a named config file in temp_dir and return its path. Used for inheritance tests, which
    // pass temp_dir as the config root so `extends` targets resolve against it.
    std::filesystem::path write_named_config(const std::string &name, const std::string &content) {
        std::filesystem::path path = temp_dir / name;
        std::ofstream file(path);
        file << content;
        file.close();
        return path;
    }
};

// Full valid config with every section set to a Noop/Zed implementation. Inheritance tests use this
// as a base and layer small overrides on top.
static const char *kBaseConfig = R"(
[rgbd_camera]
type = "ZedRgbdCamera"
camera_fps = 60

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)";

// Test NoopRgbdCamera configuration
TEST_F(ConfigTest, NoopRgbdCameraConfiguration) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());
    ASSERT_NE(config.camera, nullptr);
    EXPECT_EQ(config.camera->type, "NoopRgbdCamera");

    // Verify it's the correct derived type
    auto *noop_config = dynamic_cast<NoopRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(noop_config, nullptr);
}

// Test ZedRgbdCamera configuration with default values
TEST_F(ConfigTest, ZedRgbdCameraConfigurationDefaults) {
    write_config_file(R"(
[rgbd_camera]
type = "ZedRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());
    ASSERT_NE(config.camera, nullptr);
    EXPECT_EQ(config.camera->type, "ZedRgbdCamera");

    auto *zed_config = dynamic_cast<ZedRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(zed_config, nullptr);

    // Check default values
    EXPECT_EQ(zed_config->camera_fps, 30);
    EXPECT_EQ(zed_config->camera_resolution, Resolution::RES_1280x720);
    EXPECT_EQ(zed_config->depth_mode, DepthMode::ZED_NEURAL_LIGHT);
}

// Test ZedRgbdCamera configuration with custom values
TEST_F(ConfigTest, ZedRgbdCameraConfigurationCustomValues) {
    write_config_file(R"(
[rgbd_camera]
type = "ZedRgbdCamera"
camera_fps = 60
camera_resolution = "RES_1920x1080"
depth_mode = "ZED_ULTRA"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());
    ASSERT_NE(config.camera, nullptr);

    auto *zed_config = dynamic_cast<ZedRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(zed_config, nullptr);

    // Check custom values
    EXPECT_EQ(zed_config->camera_fps, 60);
    EXPECT_EQ(zed_config->camera_resolution, Resolution::RES_1920x1080);
    EXPECT_EQ(zed_config->depth_mode, DepthMode::ZED_ULTRA);
}

// Test ZedRgbdCamera configuration with partial custom values
TEST_F(ConfigTest, ZedRgbdCameraConfigurationPartialCustom) {
    write_config_file(R"(
[rgbd_camera]
type = "ZedRgbdCamera"
camera_fps = 90

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());
    auto *zed_config = dynamic_cast<ZedRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(zed_config, nullptr);

    // Only camera_fps should be custom, others should be defaults
    EXPECT_EQ(zed_config->camera_fps, 90);
    EXPECT_EQ(zed_config->camera_resolution, Resolution::RES_1280x720);  // default
    EXPECT_EQ(zed_config->depth_mode, DepthMode::ZED_NEURAL_LIGHT);      // default
}

// Test unknown camera type
TEST_F(ConfigTest, UnknownCameraType) {
    write_config_file(R"(
[rgbd_camera]
type = "UnknownCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    EXPECT_THROW(
        {
            try {
                load_classes_from_config(temp_config_file.string());
            } catch (const std::invalid_argument &e) {
                EXPECT_STREQ(e.what(), "Unknown configuration type: UnknownCamera");
                throw;
            }
        },
        std::invalid_argument);
}

// Test extra unknown field in config
TEST_F(ConfigTest, UnknownFieldInConfig) {
    write_config_file(R"(
[rgbd_camera]
type = "ZedRgbdCamera"
camera_fps = 60
unknown_field = "should cause error"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    EXPECT_THROW(
        {
            try {
                load_classes_from_config(temp_config_file.string());
            } catch (const ConfigValidationError &e) {
                std::string error_msg = e.what();
                EXPECT_TRUE(error_msg.find("Unknown fields") != std::string::npos);
                EXPECT_TRUE(error_msg.find("unknown_field") != std::string::npos);
                throw;
            }
        },
        ConfigValidationError);
}

// Test missing required section
TEST_F(ConfigTest, MissingRequiredSection) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"
)");
    // Missing other required sections

    EXPECT_THROW({ load_classes_from_config(temp_config_file.string()); }, ConfigValidationError);
}

// Test all interface types can be loaded
TEST_F(ConfigTest, AllInterfacesLoaded) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());

    ASSERT_NE(config.camera, nullptr);
    EXPECT_EQ(config.camera->type, "NoopRgbdCamera");

    ASSERT_NE(config.field_model, nullptr);
    EXPECT_EQ(config.field_model->type, "NoopMaskModel");

    ASSERT_NE(config.robot_mask_model, nullptr);
    EXPECT_EQ(config.robot_mask_model->type, "NoopRobotBlobModel");

    ASSERT_NE(config.field_filter, nullptr);
    EXPECT_EQ(config.field_filter->type, "NoopFieldFilter");

    ASSERT_NE(config.keypoint_model, nullptr);
    EXPECT_EQ(config.keypoint_model->type, "NoopKeypointModel");

    ASSERT_NE(config.robot_filter, nullptr);
    EXPECT_EQ(config.robot_filter->type, "NoopRobotFilter");

    ASSERT_NE(config.navigation, nullptr);
    EXPECT_EQ(config.navigation->type, "NoopNavigation");

    ASSERT_NE(config.transmitter, nullptr);
    EXPECT_EQ(config.transmitter->type, "NoopTransmitter");
}

TEST_F(ConfigTest, SafestPointTargetConfiguration) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "SafestPointTarget"
retarget_improvement_m = 0.3

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());

    ASSERT_NE(config.target_selector, nullptr);
    EXPECT_EQ(config.target_selector->type, "SafestPointTarget");
    const auto &selector_config =
        dynamic_cast<const SafestPointTargetConfiguration &>(*config.target_selector);
    EXPECT_DOUBLE_EQ(selector_config.retarget_improvement_m, 0.3);
}

TEST_F(ConfigTest, OpenTxTransmitterRejectsUnknownFields) {
    // Regression test: when channel-control fields were renamed
    // (left_channel/right_channel -> linear_channel/angular_channel,
    // reverse_left_channel/reverse_right_channel -> reverse_linear_channel/reverse_angular_channel)
    // an old config that still uses the legacy keys must fail loudly rather than silently fall back
    // to the defaults.
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "OpenTxTransmitter"
reverse_right_channel = true

[publisher]
type = "NoopPublisher"
)");

    EXPECT_THROW(
        {
            try {
                load_classes_from_config(temp_config_file.string());
            } catch (const ConfigValidationError &e) {
                std::string msg = e.what();
                EXPECT_NE(msg.find("reverse_right_channel"), std::string::npos) << msg;
                throw;
            }
        },
        ConfigValidationError);
}

TEST_F(ConfigTest, HealthConfigurationDefaultsWhenMissing) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());
    EXPECT_FALSE(config.health.enable);
    EXPECT_FALSE(config.health.tegrastats_enable);
    EXPECT_FALSE(config.health.x86_tools_enable);
    EXPECT_EQ(config.health.sample_period_ms, 5000);
}

TEST_F(ConfigTest, HealthConfigurationParsesValues) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"

[health]
enable = true
tegrastats_enable = true
x86_tools_enable = true
sample_period_ms = 1200
)");

    auto config = load_classes_from_config(temp_config_file.string());
    EXPECT_TRUE(config.health.enable);
    EXPECT_TRUE(config.health.tegrastats_enable);
    EXPECT_TRUE(config.health.x86_tools_enable);
    EXPECT_EQ(config.health.sample_period_ms, 1200);
}

// A variant that `extends` a base inherits the base's sections and overrides only its own fields.
// This also proves the synthetic `extends` key is stripped before validate_no_extra_sections runs
// (otherwise loading would throw on an unknown [extends] section).
TEST_F(ConfigTest, ExtendsMergesBaseAndOverrides) {
    write_named_config("base.toml", kBaseConfig);
    auto child = write_named_config("child.toml", R"(
extends = "base"

[rgbd_camera]
camera_fps = 90
)");

    auto config = load_classes_from_config(child.string(), temp_dir);

    // Overridden scalar wins.
    auto *zed = dynamic_cast<ZedRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(zed, nullptr);
    EXPECT_EQ(zed->camera_fps, 90);

    // Sections only present in the base are inherited.
    ASSERT_NE(config.navigation, nullptr);
    EXPECT_EQ(config.navigation->type, "NoopNavigation");
    ASSERT_NE(config.publisher, nullptr);
    EXPECT_EQ(config.publisher->type, "NoopPublisher");
}

// When a variant changes a section's `type`, the whole section is replaced: stale base-only fields
// (here camera_fps) are dropped, so the new implementation loads without an "unknown fields" error.
TEST_F(ConfigTest, ExtendsTypeChangeReplacesSection) {
    write_named_config("base.toml", kBaseConfig);
    auto child = write_named_config("child.toml", R"(
extends = "base"

[rgbd_camera]
type = "NoopRgbdCamera"
)");

    auto config = load_classes_from_config(child.string(), temp_dir);

    auto *noop = dynamic_cast<NoopRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(noop, nullptr);
    EXPECT_EQ(config.camera->type, "NoopRgbdCamera");
}

// Extending a base that does not exist fails with a clear error.
TEST_F(ConfigTest, ExtendsMissingBaseThrows) {
    auto child = write_named_config("child.toml", R"(
extends = "does_not_exist"
)");

    EXPECT_THROW(
        {
            try {
                load_classes_from_config(child.string(), temp_dir);
            } catch (const ConfigValidationError &e) {
                std::string msg = e.what();
                EXPECT_NE(msg.find("does_not_exist"), std::string::npos) << msg;
                throw;
            }
        },
        ConfigValidationError);
}

// A cycle in the extends chain is detected rather than recursing forever.
TEST_F(ConfigTest, ExtendsCycleThrows) {
    write_named_config("a.toml", "extends = \"b\"\n");
    auto b = write_named_config("b.toml", "extends = \"a\"\n");

    EXPECT_THROW(
        {
            try {
                load_classes_from_config(b.string(), temp_dir);
            } catch (const ConfigValidationError &e) {
                std::string msg = e.what();
                EXPECT_NE(msg.find("Circular"), std::string::npos) << msg;
                throw;
            }
        },
        ConfigValidationError);
}

// Inheritance chains more than one level deep resolve correctly (grandchild -> child -> base).
TEST_F(ConfigTest, ExtendsMultiLevelChain) {
    write_named_config("base.toml", kBaseConfig);
    write_named_config("mid.toml", R"(
extends = "base"

[rgbd_camera]
camera_fps = 45
)");
    auto leaf = write_named_config("leaf.toml", R"(
extends = "mid"

[transmitter]
type = "NoopTransmitter"
)");

    auto config = load_classes_from_config(leaf.string(), temp_dir);

    auto *zed = dynamic_cast<ZedRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(zed, nullptr);
    EXPECT_EQ(zed->camera_fps, 45);  // inherited from mid, which overrode base's 60
}

// Test ConfigParser directly
TEST(ConfigParserTest, GetRequiredString) {
    toml::table table = toml::parse(R"(
key = "value"
)");
    ConfigParser parser(table, "test_section");

    EXPECT_EQ(parser.get_required_string("key"), "value");
}

TEST(ConfigParserTest, GetRequiredStringMissing) {
    toml::table table = toml::parse(R"(
other_key = "value"
)");
    ConfigParser parser(table, "test_section");

    EXPECT_THROW(
        {
            try {
                parser.get_required_string("key");
            } catch (const ConfigValidationError &e) {
                std::string error_msg = e.what();
                EXPECT_TRUE(error_msg.find("Missing required field 'key'") != std::string::npos);
                throw;
            }
        },
        ConfigValidationError);
}

TEST(ConfigParserTest, GetOptionalString) {
    toml::table table = toml::parse(R"(
key = "value"
)");
    ConfigParser parser(table, "test_section");

    EXPECT_EQ(parser.get_optional_string("key", "default"), "value");
    EXPECT_EQ(parser.get_optional_string("missing", "default"), "default");
}

TEST(ConfigParserTest, GetRequiredInt) {
    toml::table table = toml::parse(R"(
key = 42
)");
    ConfigParser parser(table, "test_section");

    EXPECT_EQ(parser.get_required_int("key"), 42);
}

TEST(ConfigParserTest, GetOptionalInt) {
    toml::table table = toml::parse(R"(
key = 100
)");
    ConfigParser parser(table, "test_section");

    EXPECT_EQ(parser.get_optional_int("key", 50), 100);
    EXPECT_EQ(parser.get_optional_int("missing", 50), 50);
}

TEST(ConfigParserTest, GetRequiredDouble) {
    toml::table table = toml::parse(R"(
key = 3.14159
)");
    ConfigParser parser(table, "test_section");

    EXPECT_DOUBLE_EQ(parser.get_required_double("key"), 3.14159);
}

TEST(ConfigParserTest, GetOptionalDouble) {
    toml::table table = toml::parse(R"(
key = 2.718
)");
    ConfigParser parser(table, "test_section");

    EXPECT_DOUBLE_EQ(parser.get_optional_double("key", 1.0), 2.718);
    EXPECT_DOUBLE_EQ(parser.get_optional_double("missing", 1.0), 1.0);
}

TEST(ConfigParserTest, GetRequiredBool) {
    toml::table table = toml::parse(R"(
key = true
)");
    ConfigParser parser(table, "test_section");

    EXPECT_TRUE(parser.get_required_bool("key"));
}

TEST(ConfigParserTest, GetOptionalBool) {
    toml::table table = toml::parse(R"(
key = false
)");
    ConfigParser parser(table, "test_section");

    EXPECT_FALSE(parser.get_optional_bool("key", true));
    EXPECT_TRUE(parser.get_optional_bool("missing", true));
}

TEST(ConfigParserTest, ValidateNoExtraFields) {
    toml::table table = toml::parse(R"(
key1 = "value1"
key2 = "value2"
extra = "should cause error"
)");
    ConfigParser parser(table, "test_section");

    parser.get_required_string("key1");
    parser.get_required_string("key2");
    // extra is not accessed

    EXPECT_THROW(
        {
            try {
                parser.validate_no_extra_fields();
            } catch (const ConfigValidationError &e) {
                std::string error_msg = e.what();
                EXPECT_TRUE(error_msg.find("Unknown fields") != std::string::npos);
                EXPECT_TRUE(error_msg.find("extra") != std::string::npos);
                throw;
            }
        },
        ConfigValidationError);
}

TEST(ConfigParserTest, ValidateNoExtraFieldsPass) {
    toml::table table = toml::parse(R"(
key1 = "value1"
key2 = "value2"
)");
    ConfigParser parser(table, "test_section");

    parser.get_required_string("key1");
    parser.get_required_string("key2");

    EXPECT_NO_THROW(parser.validate_no_extra_fields());
}

// Test enum parsing with PARSE_ENUM macro using magic_enum
TEST(ConfigParserTest, ParseEnumWithDefault) {
    toml::table table = toml::parse(R"(
robot_label = "MR_STABS_MK1"
)");
    ConfigParser parser(table, "test_section");

    Label robot_label = Label::OPPONENT;  // default value
    PARSE_ENUM(robot_label, Label)

    EXPECT_EQ(robot_label, Label::MR_STABS_MK1);
}

TEST(ConfigParserTest, ParseEnumWithDefaultNotSpecified) {
    toml::table table = toml::parse(R"(
other_field = "value"
)");
    ConfigParser parser(table, "test_section");

    Label robot_label = Label::HOUSE_BOT;  // default value
    PARSE_ENUM(robot_label, Label)

    EXPECT_EQ(robot_label, Label::HOUSE_BOT);  // Should keep default
}

TEST(ConfigParserTest, ParseEnumRequired) {
    toml::table table = toml::parse(R"(
robot_group = "OURS"
)");
    ConfigParser parser(table, "test_section");

    Group robot_group;
    PARSE_ENUM_REQUIRED(robot_group, Group)

    EXPECT_EQ(robot_group, Group::OURS);
}

TEST(ConfigParserTest, ParseEnumRequiredMissing) {
    toml::table table = toml::parse(R"(
other_field = "value"
)");
    ConfigParser parser(table, "test_section");

    EXPECT_THROW(
        {
            Group robot_group;
            PARSE_ENUM_REQUIRED(robot_group, Group)
        },
        ConfigValidationError);
}

TEST(ConfigParserTest, ParseEnumInvalidValue) {
    toml::table table = toml::parse(R"(
robot_label = "INVALID_LABEL"
)");
    ConfigParser parser(table, "test_section");

    EXPECT_THROW(
        {
            Label robot_label;
            PARSE_ENUM_REQUIRED(robot_label, Label)
        },
        std::invalid_argument);
}

// Test with Group enum using magic_enum
TEST(ConfigParserTest, ParseGroupEnum) {
    toml::table table = toml::parse(R"(
team = "THEIRS"
)");
    ConfigParser parser(table, "test_section");

    Group team = Group::OURS;  // default
    PARSE_ENUM(team, Group)

    EXPECT_EQ(team, Group::THEIRS);
}

// Test with DepthMode enum
TEST(ConfigParserTest, ParseDepthModeEnum) {
    toml::table table = toml::parse(R"(
depth_mode = "ZED_ULTRA"
)");
    ConfigParser parser(table, "test_section");

    DepthMode depth_mode = DepthMode::ZED_NEURAL;  // default
    PARSE_ENUM(depth_mode, DepthMode)

    EXPECT_EQ(depth_mode, DepthMode::ZED_ULTRA);
}

// Test NoopPublisher configuration
TEST_F(ConfigTest, NoopPublisherConfiguration) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());
    ASSERT_NE(config.publisher, nullptr);
    EXPECT_EQ(config.publisher->type, "NoopPublisher");

    // Verify it's the correct derived type
    auto *noop_config = dynamic_cast<NoopPublisherConfiguration *>(config.publisher.get());
    ASSERT_NE(noop_config, nullptr);
}

// Test unknown publisher type
TEST_F(ConfigTest, UnknownPublisherType) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "UnknownPublisher"
)");

    EXPECT_THROW(
        {
            try {
                load_classes_from_config(temp_config_file.string());
            } catch (const std::invalid_argument &e) {
                // The error message comes from the config parser, not the factory
                EXPECT_TRUE(std::string(e.what()).find("UnknownPublisher") != std::string::npos);
                throw;
            }
        },
        std::invalid_argument);
}

// Profile selection: regex filtering and selection/default resolution, all hermetic in a temp dir.
class ProfileSelectionTest : public ::testing::Test {
   protected:
    std::filesystem::path config_dir;

    void SetUp() override {
        config_dir = std::filesystem::temp_directory_path() / "auto_battlebot_profile_test";
        std::filesystem::remove_all(config_dir);
        std::filesystem::create_directories(config_dir / "playback");
    }

    void TearDown() override {
        if (std::filesystem::exists(config_dir)) {
            std::filesystem::remove_all(config_dir);
        }
    }

    void write_file(const std::filesystem::path &rel, const std::string &content) {
        std::filesystem::path path = config_dir / rel;
        std::filesystem::create_directories(path.parent_path());
        std::ofstream file(path);
        file << content;
    }

    // A minimal set of profile files plus base/aux files that a jetson regex should exclude.
    void write_profiles() {
        write_file("_common.toml", "");
        write_file("_jetson.toml", "");
        write_file("mrs_buff_mk3_jetson.toml", "");
        write_file("mr_stab_mk2_jetson.toml", "");
        write_file("mrs_buff_mk3_desktop.toml", "");
        write_file("playback/mrs_buff_mk3_playback.toml", "");
    }
};

TEST_F(ProfileSelectionTest, ListAvailableProfilesFiltersByRegex) {
    write_profiles();
    // The leading [^_] excludes the _jetson base, which ".*_jetson$" would otherwise match.
    auto profiles = list_available_profiles(config_dir, "[^_].*_jetson$");
    ASSERT_EQ(profiles.size(), 2u);
    // Sorted, path-relative ids without the .toml extension.
    EXPECT_EQ(profiles[0], "mr_stab_mk2_jetson");
    EXPECT_EQ(profiles[1], "mrs_buff_mk3_jetson");
}

TEST_F(ProfileSelectionTest, ListAvailableProfilesMatchesSubdirPaths) {
    write_profiles();
    auto profiles = list_available_profiles(config_dir, ".*playback$");
    ASSERT_EQ(profiles.size(), 1u);
    EXPECT_EQ(profiles[0], "playback/mrs_buff_mk3_playback");
}

TEST_F(ProfileSelectionTest, EmptyPatternMatchesNothing) {
    write_profiles();
    EXPECT_TRUE(list_available_profiles(config_dir, "").empty());
}

TEST_F(ProfileSelectionTest, SelectionRoundTrip) {
    ProfileSelectorConfig selector;
    selector.selection_file = config_dir / "state" / "selected_profile";
    EXPECT_FALSE(read_selection_file(selector).has_value());

    write_selection_file(selector, "mrs_buff_mk3_jetson");
    auto selected = read_selection_file(selector);
    ASSERT_TRUE(selected.has_value());
    EXPECT_EQ(*selected, "mrs_buff_mk3_jetson");
}

TEST_F(ProfileSelectionTest, ResolveUsesValidSelection) {
    write_profiles();
    write_file("default_profile", "mr_stab_mk2_jetson\n");
    ProfileSelectorConfig selector;
    selector.pattern = ".*_jetson$";
    selector.selection_file = config_dir / "state" / "selected_profile";
    write_selection_file(selector, "mrs_buff_mk3_jetson");

    EXPECT_EQ(resolve_active_profile(config_dir, selector), "mrs_buff_mk3_jetson");
}

TEST_F(ProfileSelectionTest, ResolveFallsBackWhenNoSelection) {
    write_profiles();
    write_file("default_profile", "mr_stab_mk2_jetson\n");
    ProfileSelectorConfig selector;
    selector.pattern = ".*_jetson$";
    selector.selection_file = config_dir / "state" / "selected_profile";  // never written

    EXPECT_EQ(resolve_active_profile(config_dir, selector), "mr_stab_mk2_jetson");
}

TEST_F(ProfileSelectionTest, ResolveFallsBackWhenSelectionNoLongerAvailable) {
    write_profiles();
    write_file("default_profile", "mr_stab_mk2_jetson\n");
    ProfileSelectorConfig selector;
    selector.pattern = ".*_jetson$";
    selector.selection_file = config_dir / "state" / "selected_profile";
    // A desktop profile no longer matches the jetson regex -> fall back to default.
    write_selection_file(selector, "mrs_buff_mk3_desktop");

    EXPECT_EQ(resolve_active_profile(config_dir, selector), "mr_stab_mk2_jetson");
}

TEST_F(ProfileSelectionTest, ResolveThrowsWhenDefaultInvalid) {
    write_profiles();
    write_file("default_profile", "does_not_exist\n");
    ProfileSelectorConfig selector;
    selector.pattern = ".*_jetson$";
    selector.selection_file = config_dir / "state" / "selected_profile";

    EXPECT_THROW(resolve_active_profile(config_dir, selector), ConfigValidationError);
}

TEST_F(ProfileSelectionTest, LoadSelectorParsesPatternAndSelectionFile) {
    write_file("profiles.toml", R"(
[profiles]
pattern = ".*_jetson$"
selection_file = "/tmp/auto_battlebot_profile_test_sel"
)");
    ProfileSelectorConfig selector = load_profile_selector(config_dir);
    EXPECT_EQ(selector.pattern, ".*_jetson$");
    EXPECT_EQ(selector.selection_file,
              std::filesystem::path("/tmp/auto_battlebot_profile_test_sel"));
}

TEST_F(ProfileSelectionTest, LoadSelectorMissingFileDegradesGracefully) {
    ProfileSelectorConfig selector = load_profile_selector(config_dir);  // no profiles.toml
    EXPECT_TRUE(selector.pattern.empty());
    EXPECT_FALSE(selector.selection_file.empty());  // default path
}

TEST_F(ConfigTest, RobotFilterKeypointHeightsParsePerLabelWithDefault) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "RobotFrontBackFilter"
default_frame_id = "OUR_ROBOT_1"
front_keypoints = ["OPPONENT_FRONT"]
back_keypoints = ["OPPONENT_BACK"]
keypoint_height_meters = 0.05

[robot_filter.keypoint_height_meters_per_label]
OPPONENT = 0.09

[robot_filter.label_mapping]
"OPPONENT" = ["THEIR_ROBOT_1"]

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)");

    auto config = load_classes_from_config(temp_config_file.string());
    auto *filter_config =
        dynamic_cast<RobotFrontBackFilterConfiguration *>(config.robot_filter.get());
    ASSERT_NE(filter_config, nullptr);

    const KeypointHeights &heights = filter_config->robot_keypoint_tracker_config.keypoint_heights;
    EXPECT_DOUBLE_EQ(heights.default_meters, 0.05);
    EXPECT_DOUBLE_EQ(heights.height_for(Label::OPPONENT), 0.09);
    // Labels without a per-label entry fall back to the default.
    EXPECT_DOUBLE_EQ(heights.height_for(Label::HOUSE_BOT), 0.05);
}

namespace {
/** Minimal full config with a parameterizable [robot_filter.motion_estimator] block. */
std::string config_with_motion_estimator(const std::string &motion_estimator_toml) {
    return R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "RobotFrontBackFilter"
default_frame_id = "OUR_ROBOT_1"
front_keypoints = ["OPPONENT_FRONT"]
back_keypoints = ["OPPONENT_BACK"]

[robot_filter.label_mapping]
"OPPONENT" = ["THEIR_ROBOT_1"]
)" + motion_estimator_toml +
           R"(
[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)";
}
}  // namespace

TEST_F(ConfigTest, RobotFilterMotionEstimatorDefaultsToDeadReckoning) {
    write_config_file(config_with_motion_estimator(""));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *filter_config =
        dynamic_cast<RobotFrontBackFilterConfiguration *>(config.robot_filter.get());
    ASSERT_NE(filter_config, nullptr);
    ASSERT_NE(filter_config->motion_estimator, nullptr);
    EXPECT_EQ(filter_config->motion_estimator->type, "DeadReckoningMotionEstimator");
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorParsesKalman) {
    write_config_file(config_with_motion_estimator(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
opponent_accel_psd = 12.5
max_coast_s = 0.3
gate_nis = 9.21
)"));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *filter_config =
        dynamic_cast<RobotFrontBackFilterConfiguration *>(config.robot_filter.get());
    ASSERT_NE(filter_config, nullptr);
    auto *kalman_config =
        dynamic_cast<KalmanMotionEstimatorConfiguration *>(filter_config->motion_estimator.get());
    ASSERT_NE(kalman_config, nullptr);
    EXPECT_EQ(kalman_config->our_robot_mode, OurRobotMode::DEAD_RECKONING);
    EXPECT_DOUBLE_EQ(kalman_config->opponent_accel_psd, 12.5);
    EXPECT_DOUBLE_EQ(kalman_config->max_coast_s, 0.3);
    EXPECT_DOUBLE_EQ(kalman_config->gate_nis, 9.21);
    // Unset fields keep their defaults.
    EXPECT_DOUBLE_EQ(kalman_config->blob_position_sigma_m, 0.10);
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorEkfModeRejectedUntilPlantModelExists) {
    write_config_file(config_with_motion_estimator(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
our_robot_mode = "EKF"
)"));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

namespace {
/** The jig-fitted plant table, stage A values (plant_stageA.toml, 2026-08-23). */
constexpr const char *kPlantTableToml = R"(
[robot_filter.motion_estimator.plant]
dz_lin_fwd = 0.0109768
dz_lin_rev = 0.0253396
dz_ang_l = 0.016061
dz_ang_r = 0.0240734
k_fwd = 4.88002
k_rev = 4.3546
k_ang = 31.7062
tau_lin_a = 0.14921
tau_lin_d = 0.123461
tau_ang_a = 0.173867
tau_ang_d = 0.0878998
delay_s = 0.0522094
c_sb = 2.70197
c_ad = 0.463423
c_drift = 0.0
c_drift_bias = 0.0
)";
}  // namespace

TEST_F(ConfigTest, RobotFilterMotionEstimatorParsesPlantTable) {
    write_config_file(config_with_motion_estimator(std::string(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
)") + kPlantTableToml +
                                                   R"(
[robot_filter.motion_estimator.plant.process_noise]
heading_random_walk = 0.5
)"));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *filter_config =
        dynamic_cast<RobotFrontBackFilterConfiguration *>(config.robot_filter.get());
    ASSERT_NE(filter_config, nullptr);
    auto *kalman_config =
        dynamic_cast<KalmanMotionEstimatorConfiguration *>(filter_config->motion_estimator.get());
    ASSERT_NE(kalman_config, nullptr);
    ASSERT_TRUE(kalman_config->plant.has_value());
    EXPECT_DOUBLE_EQ(kalman_config->plant->dz_lin_fwd, 0.0109768);
    EXPECT_DOUBLE_EQ(kalman_config->plant->k_ang, 31.7062);
    EXPECT_DOUBLE_EQ(kalman_config->plant->delay_s, 0.0522094);
    EXPECT_DOUBLE_EQ(kalman_config->plant->c_sb, 2.70197);
    EXPECT_DOUBLE_EQ(kalman_config->plant->c_drift, 0.0);
    // Overridden noise field takes, the rest keep the baked-in fit values.
    EXPECT_DOUBLE_EQ(kalman_config->plant_noise.heading_random_walk, 0.5);
    EXPECT_DOUBLE_EQ(kalman_config->plant_noise.q_along, JigPlantNoiseParams{}.q_along);
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorEkfModeAcceptedWithPlantTable) {
    write_config_file(config_with_motion_estimator(std::string(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
our_robot_mode = "EKF"
)") + kPlantTableToml));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *filter_config =
        dynamic_cast<RobotFrontBackFilterConfiguration *>(config.robot_filter.get());
    ASSERT_NE(filter_config, nullptr);
    auto *kalman_config =
        dynamic_cast<KalmanMotionEstimatorConfiguration *>(filter_config->motion_estimator.get());
    ASSERT_NE(kalman_config, nullptr);
    EXPECT_EQ(kalman_config->our_robot_mode, OurRobotMode::EKF);
    ASSERT_TRUE(kalman_config->plant.has_value());
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorPlantMissingFieldThrows) {
    // Every plant field is required; drop delay_s and the parse must refuse rather than
    // default a fitted physical parameter to zero.
    std::string table(kPlantTableToml);
    const auto at = table.find("delay_s");
    ASSERT_NE(at, std::string::npos);
    table.erase(at, table.find('\n', at) + 1 - at);
    write_config_file(config_with_motion_estimator(std::string(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
)") + table));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorPlantUnknownFieldThrows) {
    write_config_file(config_with_motion_estimator(std::string(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
)") + kPlantTableToml + "not_a_plant_field = 1.0\n"));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

namespace {
/** Minimal full config with a parameterizable [transmitter] block. */
std::string config_with_transmitter(const std::string &transmitter_toml) {
    return R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"

[keypoint_model]
type = "NoopKeypointModel"

[robot_filter]
type = "NoopRobotFilter"

[target_selector]
type = "NoopTarget"

[navigation]
type = "NoopNavigation"
)" + transmitter_toml +
           R"(
[publisher]
type = "NoopPublisher"
)";
}
}  // namespace

TEST_F(ConfigTest, PlaybackTransmitterBehaviorModeDefaultsToAttack) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"
)"));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *transmitter_config =
        dynamic_cast<PlaybackTransmitterConfiguration *>(config.transmitter.get());
    ASSERT_NE(transmitter_config, nullptr);
    EXPECT_EQ(transmitter_config->behavior_mode, BehaviorMode::ATTACK);
}

TEST_F(ConfigTest, PlaybackTransmitterBehaviorModeParsesRunAway) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"
behavior_mode = "RUN_AWAY"
)"));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *transmitter_config =
        dynamic_cast<PlaybackTransmitterConfiguration *>(config.transmitter.get());
    ASSERT_NE(transmitter_config, nullptr);
    EXPECT_EQ(transmitter_config->behavior_mode, BehaviorMode::RUN_AWAY);
}

TEST_F(ConfigTest, PlaybackTransmitterUnknownBehaviorModeThrows) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"
behavior_mode = "FLEE"
)"));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), std::invalid_argument);
}

}  // namespace auto_battlebot
