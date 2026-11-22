#include <gtest/gtest.h>

#include <memory>

#include "config/config_parser.hpp"
#include "time/config.hpp"
#include "time/stepped_clock.hpp"

namespace auto_battlebot {

TEST(SteppedClockTest, FollowsFramesUntilTheLoopSteps) {
    SteppedClock clock;
    EXPECT_EQ(clock.now(), 0.0);
    clock.set(10.0);
    EXPECT_EQ(clock.now(), 10.0);
    clock.set(10.5);
    EXPECT_EQ(clock.now(), 10.5);

    clock.step_to(10.504);
    EXPECT_EQ(clock.now(), 10.504);
    clock.set(10.6);
    EXPECT_EQ(clock.now(), 10.504);
    clock.step_to(10.508);
    EXPECT_EQ(clock.now(), 10.508);
}

TEST(SteppedClockTest, FactoryBuildsSteppedClockFromToml) {
    const toml::table toml_data = toml::parse("[clock]\ntype = \"SteppedClock\"\n");
    std::vector<std::string> parsed;
    const auto config = load_clock_from_toml(toml_data, parsed);
    ASSERT_EQ(config->type, "SteppedClock");
    const auto clock = make_clock(*config);
    ASSERT_NE(std::dynamic_pointer_cast<SteppedClock>(clock), nullptr);
    clock->step_to(3.0);
    clock->set(4.0);
    EXPECT_EQ(clock->now(), 3.0);
}

}  // namespace auto_battlebot
