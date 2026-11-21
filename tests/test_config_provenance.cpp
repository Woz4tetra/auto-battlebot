#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "config/config.hpp"
#include "config/config_provenance.hpp"

namespace auto_battlebot {
namespace {
const char *kNoopSections = R"(
[rgbd_camera]
type = "NoopRgbdCamera"

[field_model]
type = "NoopMaskModel"

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

[plant]
dz_lin_fwd = 0.0
dz_lin_rev = 0.0
dz_ang_l = 0.0
dz_ang_r = 0.0
k_fwd = 1.0
k_rev = 1.0
k_ang = 1.0
tau_lin_a = 0.0
tau_lin_d = 0.0
tau_ang_a = 0.0
tau_ang_d = 0.0
delay_s = 0.0
c_sb = 0.0
c_ad = 0.0
c_drift = 0.0
c_drift_bias = 0.0
)";

class ConfigProvenanceTest : public ::testing::Test {
   protected:
    void SetUp() override {
        root_ = std::filesystem::temp_directory_path() / "auto_battlebot_test_provenance";
        std::filesystem::remove_all(root_);
        std::filesystem::create_directories(root_ / "robots");
    }

    void TearDown() override { std::filesystem::remove_all(root_); }

    std::filesystem::path write(const std::string &name, const std::string &content) {
        std::filesystem::path path = root_ / name;
        std::ofstream(path) << content;
        return path;
    }

    ResolvedConfig resolve(const std::filesystem::path &profile) {
        ResolvedConfig resolved;
        load_classes_from_config(profile, root_, &resolved);
        return resolved;
    }

    std::string source(const ResolvedConfig &resolved, const std::string &section,
                       const std::string &key) {
        const ResolvedField *field = resolved.find(section, key);
        return field ? field->source : "<absent>";
    }

    std::filesystem::path root_;
};

TEST_F(ConfigProvenanceTest, NamesTheFileAndLineThatSetEachValue) {
    write("base.toml", std::string(kNoopSections) + "\n[health]\nsample_period_ms = 900\n");
    auto profile = write("child.toml", "extends = \"base\"\n\n[health]\nenable = true\n");

    ResolvedConfig resolved = resolve(profile);

    EXPECT_EQ(source(resolved, "health", "sample_period_ms"), "base.toml:48");
    EXPECT_EQ(source(resolved, "health", "enable"), "child.toml:4");
    EXPECT_EQ(resolved.find("health", "sample_period_ms")->value, "900");
}

TEST_F(ConfigProvenanceTest, ListsOmittedFieldsAsDefaults) {
    auto profile = write("p.toml", std::string(kNoopSections) + "\n[health]\nenable = true\n");

    ResolvedConfig resolved = resolve(profile);

    const ResolvedField *tegrastats = resolved.find("health", "tegrastats_enable");
    ASSERT_NE(tegrastats, nullptr);
    EXPECT_EQ(tegrastats->source, kDefaultSource);
    EXPECT_EQ(tegrastats->value, HealthConfiguration{}.tegrastats_enable ? "true" : "false");
}

TEST_F(ConfigProvenanceTest, OmittedSectionsRecordNothing) {
    auto profile = write("p.toml", kNoopSections);

    ResolvedConfig resolved = resolve(profile);

    EXPECT_EQ(resolved.find("health", "enable"), nullptr);
}

TEST_F(ConfigProvenanceTest, FromSectionPointsAtTheSectionFile) {
    write("robots/robot.toml", "[transmitter]\ntype = \"NoopTransmitter\"\n");
    write("base.toml", kNoopSections);
    auto profile =
        write("child.toml", "extends = \"base\"\n\n[transmitter]\nfrom = \"robots/robot\"\n");

    ResolvedConfig resolved = resolve(profile);

    EXPECT_EQ(source(resolved, "transmitter", "type"), "robots/robot.toml:2");
}

TEST_F(ConfigProvenanceTest, TypeSwapTakesSourceFromTheOverlay) {
    write("base.toml", kNoopSections);
    auto profile =
        write("child.toml", "extends = \"base\"\n\n[navigation]\ntype = \"NoopNavigation\"\n");

    ResolvedConfig resolved = resolve(profile);

    EXPECT_EQ(source(resolved, "navigation", "type"), "child.toml:4");
}

TEST_F(ConfigProvenanceTest, FormattedOutputReloadsAsTheSameValues) {
    auto profile =
        write("p.toml", std::string(kNoopSections) + "\n[health]\nsample_period_ms = 700\n");

    const std::string text = format_resolved_toml(resolve(profile));
    toml::table reloaded = toml::parse(text);

    EXPECT_EQ(reloaded["health"]["sample_period_ms"].value<int64_t>(), 700);
    EXPECT_EQ(reloaded["rgbd_camera"]["type"].value<std::string>(), "NoopRgbdCamera");
    const size_t line_start = text.find("sample_period_ms = 700");
    ASSERT_NE(line_start, std::string::npos) << text;
    const std::string line = text.substr(line_start, text.find('\n', line_start) - line_start);
    EXPECT_NE(line.find("# p.toml:48"), std::string::npos) << line;
}

TEST(FormatTomlValue, PrintsShortestRoundTripDoubles) {
    EXPECT_EQ(format_toml_value(toml::value<double>(0.1)), "0.1");
    EXPECT_EQ(format_toml_value(toml::value<double>(2000.0)), "2000.0");
    toml::array array{0.25, 3.0};
    EXPECT_EQ(format_toml_value(array), "[0.25, 3.0]");
}
}  // namespace
}  // namespace auto_battlebot
