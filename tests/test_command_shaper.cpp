#include <gtest/gtest.h>
#include <toml++/toml.h>

#include <string>
#include <vector>

#include "command_shaper/config.hpp"
#include "command_shaper/plant_command_shaper.hpp"
#include "config/config_parser.hpp"
#include "plant/config.hpp"
#include "plant/plant_params.hpp"

using namespace auto_battlebot;

namespace {

JigPlantParams asymmetric_plant() {
    JigPlantParams plant;
    plant.k_fwd = 5.7302;
    plant.k_rev = 3.75736;
    plant.k_ang = 38.9057;
    return plant;
}

PlantConfiguration plant_config(const JigPlantParams &params) { return {params, {}}; }

std::unique_ptr<CommandShaperConfiguration> load(const std::string &toml_text,
                                                 const JigPlantParams &plant) {
    const toml::table table = toml::parse(toml_text);
    std::vector<std::string> parsed;
    return load_command_shaper_from_toml(table, parsed, plant_config(plant));
}

VelocityCommand body(double forward_mps, double yaw_rate_radps) {
    return {.forward_mps = forward_mps, .lateral_mps = 0.0, .yaw_rate_radps = yaw_rate_radps};
}

}  // namespace

TEST(CommandShaperTest, LinearDividesByTheGainForItsSign) {
    const JigPlantParams plant = asymmetric_plant();
    const StickCommand forward = plant_body_velocity_to_stick(body(2.0, 0.0), plant);
    const StickCommand reverse = plant_body_velocity_to_stick(body(-2.0, 0.0), plant);
    EXPECT_DOUBLE_EQ(forward.forward, 2.0 / plant.k_fwd);
    EXPECT_DOUBLE_EQ(reverse.forward, -2.0 / plant.k_rev);
}

TEST(CommandShaperTest, YawDividesByAngularGain) {
    const JigPlantParams plant = asymmetric_plant();
    EXPECT_DOUBLE_EQ(plant_body_velocity_to_stick(body(0.0, -10.0), plant).yaw,
                     -10.0 / plant.k_ang);
}

TEST(CommandShaperTest, FullSpeedTimesAStickShapesBackToThatStick) {
    const JigPlantParams plant = asymmetric_plant();
    const PlantCommandShaper shaper(plant, 0.0);
    for (double forward = -1.0; forward <= 1.0; forward += 0.05) {
        for (double yaw = -1.0; yaw <= 1.0; yaw += 0.05) {
            const StickCommand stick{.forward = forward, .lateral = 0.0, .yaw = yaw};
            const VelocityCommand body{
                .forward_mps = forward * (forward >= 0.0 ? plant.k_fwd : plant.k_rev),
                .lateral_mps = 0.0,
                .yaw_rate_radps = yaw * plant.k_ang};
            const StickCommand shaped = shaper.shape(body);
            EXPECT_NEAR(shaped.forward, forward, 1e-12) << forward << "," << yaw;
            EXPECT_NEAR(shaped.yaw, yaw, 1e-12) << forward << "," << yaw;
        }
    }
}

TEST(CommandShaperTest, SaturatesAfterConvertingWithAngularPriority) {
    const JigPlantParams plant = asymmetric_plant();
    const PlantCommandShaper shaper(plant, 1.0);
    const StickCommand stick = shaper.shape(body(plant.k_fwd, 0.4 * plant.k_ang));
    EXPECT_NEAR(stick.yaw, 0.4, 1e-12);
    EXPECT_NEAR(stick.forward, 0.6, 1e-12);
}

TEST(CommandShaperTest, LinearDeadzoneLiftsSmallCommandsBeforeSaturation) {
    const JigPlantParams plant = asymmetric_plant();
    const PlantCommandShaper shaper(plant, 1.0, 0.1);
    EXPECT_NEAR(shaper.shape(body(0.5 * plant.k_fwd, 0.0)).forward, 0.1 + 0.5 * 0.9, 1e-12);
    EXPECT_NEAR(shaper.shape(body(-0.5 * plant.k_rev, 0.0)).forward, -(0.1 + 0.5 * 0.9), 1e-12);
    EXPECT_DOUBLE_EQ(shaper.shape(body(0.0, 0.0)).forward, 0.0);
    EXPECT_DOUBLE_EQ(shaper.shape(body(0.0, 2.0)).yaw, 2.0 / plant.k_ang)
        << "the deadzone is linear only";
}

TEST(CommandShaperTest, SaturationLimitBelowOneShrinksLinearHeadroom) {
    const StickCommand stick = saturate_stick({.forward = -1.0, .lateral = 0.0, .yaw = 0.2}, 0.5);
    EXPECT_NEAR(stick.forward, -0.3, 1e-12);
    EXPECT_DOUBLE_EQ(stick.yaw, 0.2);
}

TEST(CommandShaperTest, MissingSectionBuildsAPlantShaperAtFullSaturation) {
    const auto config = load("", asymmetric_plant());
    const auto &plant_shaper = dynamic_cast<const PlantCommandShaperConfiguration &>(*config);
    EXPECT_DOUBLE_EQ(plant_shaper.velocity_saturation_limit, 1.0);
    EXPECT_DOUBLE_EQ(plant_shaper.plant.k_rev, asymmetric_plant().k_rev);
}

TEST(CommandShaperTest, SectionSetsSaturationLimit) {
    const auto config = load(R"(
[command_shaper]
type = "PlantCommandShaper"
velocity_saturation_limit = 0.8
)",
                             asymmetric_plant());
    const auto &plant_shaper = dynamic_cast<const PlantCommandShaperConfiguration &>(*config);
    EXPECT_DOUBLE_EQ(plant_shaper.velocity_saturation_limit, 0.8);
}

TEST(CommandShaperTest, UnknownFieldFailsAtParse) {
    EXPECT_THROW(load(R"(
[command_shaper]
type = "PlantCommandShaper"
saturation = 0.8
)",
                      asymmetric_plant()),
                 ConfigValidationError);
}

TEST(CommandShaperTest, NonPositiveGainFailsAtParse) {
    for (double JigPlantParams::*gain :
         {&JigPlantParams::k_fwd, &JigPlantParams::k_rev, &JigPlantParams::k_ang}) {
        JigPlantParams plant = asymmetric_plant();
        plant.*gain = 0.0;
        EXPECT_THROW(load("", plant), ConfigValidationError);
    }
}

TEST(CommandShaperTest, MissingPlantTableFailsAtParse) {
    const toml::table table = toml::parse("");
    std::vector<std::string> parsed;
    EXPECT_THROW(load_plant_from_toml(table, parsed), ConfigValidationError);
}
