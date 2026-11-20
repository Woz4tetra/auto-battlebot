#include <gtest/gtest.h>

#include <algorithm>
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

namespace {
void expect_config_error_mentioning(const std::string &path, const std::string &needle) {
    try {
        load_classes_from_config(path);
        FAIL() << "expected ConfigValidationError";
    } catch (const ConfigValidationError &e) {
        EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
    }
}
}  // namespace

class ConfigTest : public ::testing::Test {
   protected:
    std::filesystem::path temp_dir;
    std::filesystem::path temp_config_file;

    void SetUp() override {
        temp_dir = std::filesystem::temp_directory_path() / "auto_battlebot_test_configs";
        std::filesystem::create_directories(temp_dir);
        temp_config_file = temp_dir / "test_config.toml";
    }

    void TearDown() override {
        if (std::filesystem::exists(temp_dir)) {
            std::filesystem::remove_all(temp_dir);
        }
    }

    void write_config_file(const std::string &content) {
        std::ofstream file(temp_config_file);
        file << content;
        file.close();
    }

    std::filesystem::path write_named_config(const std::string &name, const std::string &content) {
        std::filesystem::path path = temp_dir / name;
        std::ofstream file(path);
        file << content;
        file.close();
        return path;
    }
};

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

    auto *noop_config = dynamic_cast<NoopRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(noop_config, nullptr);
}

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

    EXPECT_EQ(zed_config->camera_fps, 30);
    EXPECT_EQ(zed_config->camera_resolution, Resolution::RES_1280x720);
    EXPECT_EQ(zed_config->depth_mode, DepthMode::ZED_NEURAL_LIGHT);
}

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

    EXPECT_EQ(zed_config->camera_fps, 60);
    EXPECT_EQ(zed_config->camera_resolution, Resolution::RES_1920x1080);
    EXPECT_EQ(zed_config->depth_mode, DepthMode::ZED_ULTRA);
}

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

    EXPECT_EQ(zed_config->camera_fps, 90);
    const ZedRgbdCameraConfiguration defaults;
    EXPECT_EQ(zed_config->camera_resolution, defaults.camera_resolution);
    EXPECT_EQ(zed_config->depth_mode, defaults.depth_mode);
}

TEST_F(ConfigTest, ZedOneRgbCameraConfiguration) {
    write_config_file(R"(
[rgbd_camera]
type = "ZedOneRgbCamera"
camera_resolution = "RES_960x600"
camera_fps = 120
video_recording = false

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
    auto *zed_one_config = dynamic_cast<ZedOneRgbCameraConfiguration *>(config.camera.get());
    ASSERT_NE(zed_one_config, nullptr);
    EXPECT_EQ(zed_one_config->camera_resolution, Resolution::RES_960x600);
    EXPECT_EQ(zed_one_config->camera_fps, 120);
    EXPECT_FALSE(zed_one_config->video_recording);
    EXPECT_EQ(zed_one_config->video_bitrate_kbps,
              ZedOneRgbCameraConfiguration{}.video_bitrate_kbps);
}

TEST_F(ConfigTest, ZedOneRgbCameraRejectsDepthMode) {
    write_config_file(R"(
[rgbd_camera]
type = "ZedOneRgbCamera"
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

    expect_config_error_mentioning(temp_config_file.string(), "depth_mode");
}

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

TEST_F(ConfigTest, MissingRequiredSection) {
    write_config_file(R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"
)");

    EXPECT_THROW({ load_classes_from_config(temp_config_file.string()); }, ConfigValidationError);
}

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

TEST_F(ConfigTest, OpenTxTransmitterRejectsLegacyLeftRightChannelKeys) {
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

TEST_F(ConfigTest, ExtendsMergesBaseAndOverrides) {
    write_named_config("base.toml", kBaseConfig);
    auto child = write_named_config("child.toml", R"(
extends = "base"

[rgbd_camera]
camera_fps = 90
)");

    auto config = load_classes_from_config(child.string(), temp_dir);

    auto *zed = dynamic_cast<ZedRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(zed, nullptr);
    EXPECT_EQ(zed->camera_fps, 90);

    ASSERT_NE(config.navigation, nullptr);
    EXPECT_EQ(config.navigation->type, "NoopNavigation");
    ASSERT_NE(config.publisher, nullptr);
    EXPECT_EQ(config.publisher->type, "NoopPublisher");
}

TEST_F(ConfigTest, ExtendsKeyIsStrippedBeforeSectionValidation) {
    write_named_config("base.toml", kBaseConfig);
    auto child = write_named_config("child.toml", "extends = \"base\"\n");

    EXPECT_NO_THROW(load_classes_from_config(child.string(), temp_dir));
}

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

TEST_F(ConfigTest, ExtendsMultiLevelChain) {
    write_named_config("base.toml", kBaseConfig);
    constexpr int kMidFps = 45;
    write_named_config("mid.toml", R"(
extends = "base"

[rgbd_camera]
camera_fps = )" + std::to_string(kMidFps) +
                                       "\n");
    auto leaf = write_named_config("leaf.toml", R"(
extends = "mid"

[transmitter]
type = "NoopTransmitter"
)");

    auto config = load_classes_from_config(leaf.string(), temp_dir);

    auto *zed = dynamic_cast<ZedRgbdCameraConfiguration *>(config.camera.get());
    ASSERT_NE(zed, nullptr);
    EXPECT_EQ(zed->camera_fps, kMidFps);
}

class ConfigFromTest : public ConfigTest {
   protected:
    std::filesystem::path write_robot_file(const std::string &content) {
        std::filesystem::create_directories(temp_dir / "robots");
        return write_named_config("robots/robot.toml", content);
    }

    std::string merge_error(const std::filesystem::path &path) {
        try {
            load_merged_config(path, temp_dir);
        } catch (const ConfigValidationError &e) {
            return e.what();
        }
        return "";
    }
};

TEST_F(ConfigFromTest, LoadsSectionWithSubtables) {
    write_robot_file(R"(
[plant]
k_fwd = 5.0

[plant.process_noise]
q_along = 1.0

[navigation]
type = "Unrelated"
)");
    auto child = write_named_config("child.toml", R"(
[plant]
from = "robots/robot"
)");

    toml::table merged = load_merged_config(child, temp_dir);

    EXPECT_EQ(merged["plant"]["k_fwd"].value<double>(), 5.0);
    EXPECT_EQ(merged["plant"]["process_noise"]["q_along"].value<double>(), 1.0);
    EXPECT_FALSE(merged["plant"]["from"]);
    EXPECT_FALSE(merged["navigation"]);
}

TEST_F(ConfigFromTest, ReplacesBaseSectionWholesale) {
    write_robot_file("[plant]\nk_fwd = 5.0\n");
    write_named_config("base.toml", R"(
[plant]
k_fwd = 1.0
c_sb = 2.0

[plant.process_noise]
q_along = 3.0
)");
    auto child = write_named_config("child.toml", R"(
extends = "base"

[plant]
from = "robots/robot.toml"
)");

    toml::table merged = load_merged_config(child, temp_dir);

    EXPECT_EQ(merged["plant"]["k_fwd"].value<double>(), 5.0);
    EXPECT_FALSE(merged["plant"]["c_sb"]);
    EXPECT_FALSE(merged["plant"]["process_noise"]);
}

TEST_F(ConfigFromTest, ReplacesNestedSectionOnly) {
    write_robot_file(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
max_coast_s = 0.2
)");
    write_named_config("base.toml", R"(
[robot_filter]
type = "RobotFrontBackFilter"

[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
opponent_max_coast_s = 0.1
)");
    auto child = write_named_config("child.toml", R"(
extends = "base"

[robot_filter.motion_estimator]
from = "robots/robot"
)");

    toml::table merged = load_merged_config(child, temp_dir);

    EXPECT_EQ(merged["robot_filter"]["type"].value<std::string>(), "RobotFrontBackFilter");
    EXPECT_EQ(merged["robot_filter"]["motion_estimator"]["max_coast_s"].value<double>(), 0.2);
    EXPECT_FALSE(merged["robot_filter"]["motion_estimator"]["opponent_max_coast_s"]);
}

TEST_F(ConfigFromTest, InlineOverlayMergesOntoLoadedSection) {
    write_robot_file("[plant]\nk_fwd = 5.0\nk_ang = 30.0\n");
    write_named_config("base.toml", "[plant]\nfrom = \"robots/robot\"\n");
    auto child = write_named_config("child.toml", R"(
extends = "base"

[plant]
k_fwd = 9.0
)");

    toml::table merged = load_merged_config(child, temp_dir);

    EXPECT_EQ(merged["plant"]["k_fwd"].value<double>(), 9.0);
    EXPECT_EQ(merged["plant"]["k_ang"].value<double>(), 30.0);
}

TEST_F(ConfigFromTest, ParsesIntoClassConfiguration) {
    write_robot_file(R"(
[transmitter]
type = "NoopTransmitter"
)");
    write_named_config("base.toml", kBaseConfig);
    auto child = write_named_config("child.toml", R"(
extends = "base"

[transmitter]
from = "robots/robot"
)");

    auto config = load_classes_from_config(child.string(), temp_dir);

    ASSERT_NE(config.transmitter, nullptr);
    EXPECT_EQ(config.transmitter->type, "NoopTransmitter");
}

TEST_F(ConfigFromTest, OtherKeysBesideFromThrow) {
    write_robot_file("[plant]\nk_fwd = 5.0\n");
    auto child = write_named_config("child.toml", R"(
[plant]
from = "robots/robot"
k_ang = 30.0
)");

    std::string msg = merge_error(child);
    EXPECT_NE(msg.find("[plant]"), std::string::npos) << msg;
    EXPECT_NE(msg.find("only key"), std::string::npos) << msg;
}

TEST_F(ConfigFromTest, SubtableBesideFromThrows) {
    write_robot_file("[plant]\nk_fwd = 5.0\n");
    auto child = write_named_config("child.toml", R"(
[plant]
from = "robots/robot"

[plant.process_noise]
q_along = 1.0
)");

    EXPECT_NE(merge_error(child).find("only key"), std::string::npos);
}

TEST_F(ConfigFromTest, MissingFileThrows) {
    auto child = write_named_config("child.toml", "[plant]\nfrom = \"robots/absent\"\n");

    std::string msg = merge_error(child);
    EXPECT_NE(msg.find("robots/absent"), std::string::npos) << msg;
    EXPECT_NE(msg.find("no file"), std::string::npos) << msg;
}

TEST_F(ConfigFromTest, MissingSectionInFileThrows) {
    write_robot_file("[navigation]\ntype = \"NoopNavigation\"\n");
    auto child = write_named_config("child.toml", "[plant]\nfrom = \"robots/robot\"\n");

    std::string msg = merge_error(child);
    EXPECT_NE(msg.find("has no [plant]"), std::string::npos) << msg;
}

TEST_F(ConfigFromTest, SectionFileWithExtendsThrows) {
    write_named_config("base.toml", "");
    write_robot_file("extends = \"base\"\n\n[plant]\nk_fwd = 5.0\n");
    auto child = write_named_config("child.toml", "[plant]\nfrom = \"robots/robot\"\n");

    EXPECT_NE(merge_error(child).find("`extends`"), std::string::npos);
}

TEST_F(ConfigFromTest, SectionFileWithFromThrows) {
    write_named_config("other.toml", "[plant]\nk_fwd = 5.0\n");
    write_robot_file("[plant]\nfrom = \"other\"\n");
    auto child = write_named_config("child.toml", "[plant]\nfrom = \"robots/robot\"\n");

    EXPECT_NE(merge_error(child).find("uses `from`"), std::string::npos);
}

TEST_F(ConfigFromTest, TopLevelFromThrows) {
    auto child = write_named_config("child.toml", "from = \"robots/robot\"\n");

    EXPECT_NE(merge_error(child).find("only valid inside a table"), std::string::npos);
}

TEST_F(ConfigFromTest, NonStringFromThrows) {
    auto child = write_named_config("child.toml", "[plant]\nfrom = 3\n");

    EXPECT_NE(merge_error(child).find("must be a string"), std::string::npos);
}

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

TEST(ConfigParserTest, ParseEnumWithDefault) {
    toml::table table = toml::parse(R"(
robot_label = "MR_STABS_MK1"
)");
    ConfigParser parser(table, "test_section");

    Label robot_label = Label::OPPONENT;
    PARSE_ENUM(robot_label, Label)

    EXPECT_EQ(robot_label, Label::MR_STABS_MK1);
}

TEST(ConfigParserTest, ParseEnumWithDefaultNotSpecified) {
    toml::table table = toml::parse(R"(
other_field = "value"
)");
    ConfigParser parser(table, "test_section");

    Label robot_label = Label::HOUSE_BOT;
    PARSE_ENUM(robot_label, Label)

    EXPECT_EQ(robot_label, Label::HOUSE_BOT);
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

TEST(ConfigParserTest, ParseGroupEnum) {
    toml::table table = toml::parse(R"(
team = "THEIRS"
)");
    ConfigParser parser(table, "test_section");

    Group team = Group::OURS;
    PARSE_ENUM(team, Group)

    EXPECT_EQ(team, Group::THEIRS);
}

TEST(ConfigParserTest, ParseDepthModeEnum) {
    toml::table table = toml::parse(R"(
depth_mode = "ZED_ULTRA"
)");
    ConfigParser parser(table, "test_section");

    DepthMode depth_mode = DepthMode::ZED_NEURAL;
    PARSE_ENUM(depth_mode, DepthMode)

    EXPECT_EQ(depth_mode, DepthMode::ZED_ULTRA);
}

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

    auto *noop_config = dynamic_cast<NoopPublisherConfiguration *>(config.publisher.get());
    ASSERT_NE(noop_config, nullptr);
}

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
                EXPECT_TRUE(std::string(e.what()).find("UnknownPublisher") != std::string::npos);
                throw;
            }
        },
        std::invalid_argument);
}

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
    const std::string kJetsonProfilesExcludingBasePattern = "[^_].*_jetson$";
    auto profiles = list_available_profiles(config_dir, kJetsonProfilesExcludingBasePattern);
    ASSERT_EQ(profiles.size(), 2u);
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
    const auto kNeverWrittenSelectionFile = config_dir / "state" / "selected_profile";
    selector.selection_file = kNeverWrittenSelectionFile;

    EXPECT_EQ(resolve_active_profile(config_dir, selector), "mr_stab_mk2_jetson");
}

TEST_F(ProfileSelectionTest, ResolveFallsBackWhenSelectionNoLongerAvailable) {
    write_profiles();
    write_file("default_profile", "mr_stab_mk2_jetson\n");
    ProfileSelectorConfig selector;
    selector.pattern = ".*_jetson$";
    selector.selection_file = config_dir / "state" / "selected_profile";
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
    ProfileSelectorConfig selector = load_profile_selector(config_dir);
    EXPECT_TRUE(selector.pattern.empty());
    EXPECT_FALSE(selector.selection_file.empty());
}

namespace {
constexpr const char *kPlantTableToml = R"(
[plant]
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

TEST_F(ConfigTest, RobotFilterKeypointHeightsParsePerLabelWithDefault) {
    write_config_file(std::string(R"(
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

[robot_filter.footprint_center_offset_per_label]
MR_STABS_MK2 = [0.043, -0.01]

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
)") + kPlantTableToml);

    auto config = load_classes_from_config(temp_config_file.string());
    auto *filter_config =
        dynamic_cast<RobotFrontBackFilterConfiguration *>(config.robot_filter.get());
    ASSERT_NE(filter_config, nullptr);

    const KeypointHeights &heights = filter_config->robot_keypoint_tracker_config.keypoint_heights;
    EXPECT_DOUBLE_EQ(heights.default_meters, 0.05);
    EXPECT_DOUBLE_EQ(heights.height_for(Label::OPPONENT), 0.09);
    EXPECT_DOUBLE_EQ(heights.height_for(Label::HOUSE_BOT), 0.05);

    const auto &offsets = filter_config->footprint_center_offset_per_label;
    ASSERT_EQ(offsets.size(), 1u);
    EXPECT_DOUBLE_EQ(offsets.at(Label::MR_STABS_MK2).x, 0.043);
    EXPECT_DOUBLE_EQ(offsets.at(Label::MR_STABS_MK2).y, -0.01);
}

namespace {
constexpr const char *kNoPlantTable = "";

std::string config_with_motion_estimator(const std::string &motion_estimator_toml,
                                         const std::string &plant_toml = kPlantTableToml) {
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
)" + plant_toml;
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
    EXPECT_DOUBLE_EQ(kalman_config->blob_position_sigma_m,
                     KalmanMotionEstimatorConfiguration{}.blob_position_sigma_m);
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorEkfModeRejectedUntilPlantModelExists) {
    write_config_file(config_with_motion_estimator(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
our_robot_mode = "EKF"
)",
                                                   kNoPlantTable));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST_F(ConfigTest, DeadReckoningMotionEstimatorWithoutPlantTableThrows) {
    write_config_file(config_with_motion_estimator(R"(
[robot_filter.motion_estimator]
type = "DeadReckoningMotionEstimator"
)",
                                                   kNoPlantTable));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST_F(ConfigTest, MotionEstimatorsWithoutPlantTableNameThePlantTableInTheError) {
    for (const char *type : {"DeadReckoningMotionEstimator", "KalmanMotionEstimator"}) {
        write_config_file(config_with_motion_estimator(
            std::string("\n[robot_filter.motion_estimator]\ntype = \"") + type + "\"\n",
            kNoPlantTable));
        try {
            load_classes_from_config(temp_config_file.string());
            FAIL() << type << " loaded without a [plant] table";
        } catch (const ConfigValidationError &error) {
            const std::string message = error.what();
            EXPECT_NE(message.find(type), std::string::npos) << message;
            EXPECT_NE(message.find("requires the top-level [plant] table"), std::string::npos)
                << message;
        }
    }
}

TEST_F(ConfigTest, KalmanMotionEstimatorDeadReckoningModeRequiresPlantTable) {
    write_config_file(config_with_motion_estimator(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
our_robot_mode = "DEAD_RECKONING"
)",
                                                   kNoPlantTable));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorParsesPlantTable) {
    write_config_file(config_with_motion_estimator(std::string(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
)") + kPlantTableToml +
                                                       R"(
[plant.process_noise]
heading_random_walk = 0.5
)",
                                                   kNoPlantTable));

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
    EXPECT_DOUBLE_EQ(kalman_config->plant_noise.heading_random_walk, 0.5);
    EXPECT_DOUBLE_EQ(kalman_config->plant_noise.q_along, JigPlantNoiseParams{}.q_along);
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorEkfModeAcceptedWithPlantTable) {
    write_config_file(config_with_motion_estimator(std::string(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
our_robot_mode = "EKF"
)") + kPlantTableToml,
                                                   kNoPlantTable));

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
    std::string table(kPlantTableToml);
    const auto at = table.find("delay_s");
    ASSERT_NE(at, std::string::npos);
    table.erase(at, table.find('\n', at) + 1 - at);
    write_config_file(config_with_motion_estimator(std::string(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
)") + table,
                                                   kNoPlantTable));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST_F(ConfigTest, RobotFilterMotionEstimatorPlantUnknownFieldThrows) {
    write_config_file(config_with_motion_estimator(std::string(R"(
[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
)") + kPlantTableToml + "not_a_plant_field = 1.0\n",
                                                   kNoPlantTable));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

namespace {
std::string config_with_noop_filter_and_motion_profile_nav(const std::string &plant_toml) {
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
type = "MotionProfileNavigation"

[transmitter]
type = "NoopTransmitter"

[publisher]
type = "NoopPublisher"
)" + plant_toml;
}
}  // namespace

TEST_F(ConfigTest, MotionProfileNavigationReadsTheSharedPlantTable) {
    write_config_file(config_with_noop_filter_and_motion_profile_nav(kPlantTableToml));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *nav = dynamic_cast<MotionProfileNavigationConfiguration *>(config.navigation.get());
    ASSERT_NE(nav, nullptr);
    EXPECT_DOUBLE_EQ(nav->plant.k_fwd, 4.88002);
    EXPECT_DOUBLE_EQ(nav->plant.tau_lin_d, 0.123461);
    EXPECT_DOUBLE_EQ(nav->plant.delay_s, 0.0522094);
    EXPECT_DOUBLE_EQ(nav->plant.c_ad, 0.463423);
    EXPECT_FALSE(nav->compensate_angular_droop);
}

TEST_F(ConfigTest, MotionProfileNavigationWithoutPlantTableThrows) {
    write_config_file(config_with_noop_filter_and_motion_profile_nav(kNoPlantTable));

    expect_config_error_mentioning(temp_config_file.string(), "MotionProfileNavigation");
}

namespace {
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

TEST_F(ConfigTest, RunnerBehaviorModeDefaultsToAttack) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"
)"));

    auto config = load_classes_from_config(temp_config_file.string());
    EXPECT_EQ(config.runner.default_behavior_mode, BehaviorMode::ATTACK);
}

TEST_F(ConfigTest, RunnerBehaviorModeParsesRunAway) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"

[runner]
default_behavior_mode = "RUN_AWAY"
)"));

    auto config = load_classes_from_config(temp_config_file.string());
    EXPECT_EQ(config.runner.default_behavior_mode, BehaviorMode::RUN_AWAY);
}

TEST_F(ConfigTest, RunnerAutoTrackingDefaultsOn) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"
)"));
    EXPECT_TRUE(
        load_classes_from_config(temp_config_file.string()).runner.auto_tracking_by_default);

    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"

[runner]
auto_tracking_by_default = false
)"));
    EXPECT_FALSE(
        load_classes_from_config(temp_config_file.string()).runner.auto_tracking_by_default);
}

TEST_F(ConfigTest, RunnerUnknownBehaviorModeThrows) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"

[runner]
default_behavior_mode = "FLEE"
)"));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), std::invalid_argument);
}

TEST_F(ConfigTest, TransmitterBehaviorModeKeyIsRejected) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "PlaybackTransmitter"
behavior_mode = "RUN_AWAY"
)"));

    EXPECT_ANY_THROW(load_classes_from_config(temp_config_file.string()));
}

TEST_F(ConfigTest, Esp32DiagnosticsTransmitterParsesItsFieldsAndTheRadioOnes) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "Esp32DiagnosticsOpenTxTransmitter"
drive_processor = "DifferentialDriveProcessor"
reverse_angular_channel = true
esp32_host = "192.168.4.2"
esp32_port = 8080
reconnect_period_s = 0.5
)"));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *transmitter_config =
        dynamic_cast<Esp32DiagnosticsOpenTxTransmitterConfiguration *>(config.transmitter.get());
    ASSERT_NE(transmitter_config, nullptr);
    EXPECT_EQ(transmitter_config->esp32_host, "192.168.4.2");
    EXPECT_EQ(transmitter_config->esp32_port, 8080);
    EXPECT_DOUBLE_EQ(transmitter_config->reconnect_period_s, 0.5);
    EXPECT_EQ(transmitter_config->drive_processor, "DifferentialDriveProcessor");
    EXPECT_TRUE(transmitter_config->reverse_angular_channel);
}

TEST_F(ConfigTest, Esp32DiagnosticsTransmitterDefaults) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "Esp32DiagnosticsOpenTxTransmitter"
)"));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *transmitter_config =
        dynamic_cast<Esp32DiagnosticsOpenTxTransmitterConfiguration *>(config.transmitter.get());
    ASSERT_NE(transmitter_config, nullptr);
    EXPECT_EQ(transmitter_config->esp32_host, "192.168.4.1");
    EXPECT_EQ(transmitter_config->esp32_port, 80);
    EXPECT_DOUBLE_EQ(transmitter_config->reconnect_period_s, 1.0);
}

TEST_F(ConfigTest, Esp32DiagnosticsTransmitterRejectsHostnameInHostField) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "Esp32DiagnosticsOpenTxTransmitter"
esp32_host = "robot.local"
)"));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST_F(ConfigTest, Esp32DiagnosticsTransmitterRejectsUnknownField) {
    write_config_file(config_with_transmitter(R"(
[transmitter]
type = "Esp32DiagnosticsOpenTxTransmitter"
esp32_hostname = "robot"
)"));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

namespace {
std::string config_with_keypoint_model(const std::string &keypoint_model_toml) {
    return R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

[robot_mask_model]
type = "NoopRobotBlobModel"

[field_filter]
type = "NoopFieldFilter"
)" + keypoint_model_toml +
           R"(
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
}

constexpr const char *kAprilTagModelToml = R"(
[keypoint_model]
type = "AprilTagKeypointModel"
tag_size_m = 0.064
robot_tag_ids = [41, 76]
roi_margin_px = 60
max_reprojection_error_px = 1.5
front_keypoint_m = [0.05, 0.0, 0.0]
back_keypoint_m = [-0.05, 0.0, 0.0]
rest_pitch_rad = 0.198

[[keypoint_model.tags]]
id = 41
translation_m = [0.035391, 1.9e-05, -0.012076]
rotation = [0.0, -0.983904, 0.178696, -1.0, -3e-06, -1.6e-05, 1.6e-05, -0.178696, -0.983904]

[[keypoint_model.tags]]
id = 76
translation_m = [0.035553, -1.1e-05, 0.014164]
rotation = [0.0, -0.986029, 0.166572, 1.0, -2e-06, -1.1e-05, 1.1e-05, 0.166572, 0.986029]
)";
}  // namespace

TEST_F(ConfigTest, AprilTagKeypointModelParses) {
    write_config_file(config_with_keypoint_model(kAprilTagModelToml));

    auto config = load_classes_from_config(temp_config_file.string());
    auto *model = dynamic_cast<AprilTagKeypointModelConfiguration *>(config.keypoint_model.get());
    ASSERT_NE(model, nullptr);
    EXPECT_DOUBLE_EQ(model->tag_size_m, 0.064);
    EXPECT_EQ(model->robot_tag_ids, (std::vector<int>{41, 76}));
    EXPECT_EQ(model->roi_margin_px, 60);
    EXPECT_DOUBLE_EQ(model->max_reprojection_error_px, 1.5);
    EXPECT_DOUBLE_EQ(model->front_keypoint_m[0], 0.05);
    EXPECT_DOUBLE_EQ(model->back_keypoint_m[0], -0.05);
    EXPECT_DOUBLE_EQ(model->rest_pitch_rad, 0.198);
    ASSERT_EQ(model->tags.size(), 2u);
    EXPECT_EQ(model->tags[1].id, 76);
    EXPECT_DOUBLE_EQ(model->tags[1].translation_m[2], 0.014164);
    EXPECT_DOUBLE_EQ(model->tags[1].rotation[8], 0.986029);
}

TEST_F(ConfigTest, AprilTagKeypointModelRejectsATagWithoutAMount) {
    std::string toml(kAprilTagModelToml);
    toml.replace(toml.find("robot_tag_ids = [41, 76]"), 24, "robot_tag_ids = [41, 77]");
    write_config_file(config_with_keypoint_model(toml));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST_F(ConfigTest, AprilTagKeypointModelRejectsANonRotation) {
    std::string toml(kAprilTagModelToml);
    toml.replace(toml.find("[0.0, -0.986029, 0.166572, 1.0, -2e-06, -1.1e-05"),
                 std::string("[0.0, -0.986029, 0.166572, 1.0, -2e-06, -1.1e-05").size(),
                 "[1.0, -2e-06, -1.1e-05, 0.0, -0.986029, 0.166572");
    write_config_file(config_with_keypoint_model(toml));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST_F(ConfigTest, AprilTagKeypointModelRejectsADroppedRotationElement) {
    std::string toml(kAprilTagModelToml);
    const std::string original = "[0.0, -0.986029, 0.166572, 1.0, -2e-06, -1.1e-05";
    toml.replace(toml.find(original), original.size(),
                 "[0.0, -0.986029, 0.0, 1.0, -2e-06, -1.1e-05");
    write_config_file(config_with_keypoint_model(toml));

    EXPECT_THROW(load_classes_from_config(temp_config_file.string()), ConfigValidationError);
}

TEST(FieldFilterConfigTest, FieldFilterAcceptsCommonHazardFields) {
    for (const std::string type : {"NoopFieldFilter", "FixedFieldFilter"}) {
        toml::table table = toml::parse("type = \"" + type +
                                        "\"\nhazards_file = \"hazards.toml\"\n"
                                        "hazard_static_margin_m = 0.11\n"
                                        "hazard_tracked_margin_m = 0.12\n"
                                        "hazard_tracked_hold_s = 0.13\n"
                                        "hazard_hard_margin_m = 0.14\n");
        ConfigParser parser(table, "field_filter");
        std::unique_ptr<FieldFilterConfiguration> config;
        ASSERT_NO_THROW(config = parse_field_filter_config(parser)) << type;
        EXPECT_EQ(config->hazards_file, "hazards.toml") << type;
        EXPECT_DOUBLE_EQ(config->hazard_static_margin_m, 0.11) << type;
        EXPECT_DOUBLE_EQ(config->hazard_hard_margin_m, 0.14) << type;
    }
}

TEST(FieldFilterConfigTest, UnknownFieldFilterKeyStillFails) {
    toml::table table = toml::parse("type = \"NoopFieldFilter\"\nnot_a_key = 1\n");
    ConfigParser parser(table, "field_filter");
    EXPECT_THROW(parse_field_filter_config(parser), ConfigValidationError);
}

}  // namespace auto_battlebot
