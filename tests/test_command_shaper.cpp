#include <gtest/gtest.h>
#include <toml++/toml.h>

#include <string>
#include <vector>

#include "command_shaper/config.hpp"
#include "command_shaper/identity_command_shaper.hpp"
#include "config/config_parser.hpp"

using namespace auto_battlebot;

namespace {

std::unique_ptr<CommandShaperConfiguration> load(const std::string &toml_text) {
    const toml::table table = toml::parse(toml_text);
    std::vector<std::string> parsed;
    return load_command_shaper_from_toml(table, parsed);
}

}  // namespace

TEST(CommandShaperTest, IdentityPassesInRangeCommandsThrough) {
    const IdentityCommandShaper shaper(1.0);
    const StickCommand stick =
        shaper.shape({.forward_mps = 0.3, .lateral_mps = 0.0, .yaw_rate_radps = -0.4});
    EXPECT_DOUBLE_EQ(stick.forward, 0.3);
    EXPECT_DOUBLE_EQ(stick.yaw, -0.4);
}

TEST(CommandShaperTest, IdentitySaturatesWithAngularPriority) {
    const IdentityCommandShaper shaper(1.0);
    const StickCommand stick =
        shaper.shape({.forward_mps = 1.0, .lateral_mps = 0.0, .yaw_rate_radps = 0.4});
    EXPECT_DOUBLE_EQ(stick.yaw, 0.4);
    EXPECT_NEAR(stick.forward, 0.6, 1e-12);
}

TEST(CommandShaperTest, SaturationLimitBelowOneShrinksLinearHeadroom) {
    const StickCommand stick = saturate_stick({.forward = -1.0, .lateral = 0.0, .yaw = 0.2}, 0.5);
    EXPECT_NEAR(stick.forward, -0.3, 1e-12);
    EXPECT_DOUBLE_EQ(stick.yaw, 0.2);
}

TEST(CommandShaperTest, MissingSectionDefaultsToFullSaturationLimit) {
    const auto config = load("");
    const auto &identity = dynamic_cast<const IdentityCommandShaperConfiguration &>(*config);
    EXPECT_DOUBLE_EQ(identity.velocity_saturation_limit, 1.0);
}

TEST(CommandShaperTest, SectionSetsSaturationLimit) {
    const auto config = load(R"(
[command_shaper]
type = "IdentityCommandShaper"
velocity_saturation_limit = 0.8
)");
    const auto &identity = dynamic_cast<const IdentityCommandShaperConfiguration &>(*config);
    EXPECT_DOUBLE_EQ(identity.velocity_saturation_limit, 0.8);
}

TEST(CommandShaperTest, UnknownFieldFailsAtParse) {
    EXPECT_THROW(load(R"(
[command_shaper]
type = "IdentityCommandShaper"
saturation = 0.8
)"),
                 ConfigValidationError);
}
