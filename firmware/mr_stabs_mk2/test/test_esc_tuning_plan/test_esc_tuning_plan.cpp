#include <esc_tuning_plan.h>
#include <gtest/gtest.h>

using namespace esc_tuning_plan;

TEST(EscTuningPlan, ExitRequestOrButtonEdgeExitsEvenWhenIdleReleaseIsDue) {
    EXPECT_EQ(next_action(true, false, false, 100000, 0, 60000), Action::EXIT);
    EXPECT_EQ(next_action(false, true, false, 100000, 0, 60000), Action::EXIT);
}

TEST(EscTuningPlan, ReleasesEscsOnceAfterTheIdleWindow) {
    EXPECT_EQ(next_action(false, false, false, 59999, 0, 60000), Action::NONE);
    EXPECT_EQ(next_action(false, false, false, 60000, 0, 60000), Action::RELEASE_ESCS);
    EXPECT_EQ(next_action(false, false, true, 200000, 0, 60000), Action::NONE);
}

TEST(EscTuningPlan, ActivityJustNowDefersTheRelease) {
    EXPECT_EQ(next_action(false, false, false, 100000, 99990, 60000), Action::NONE);
}

TEST(EscTuningPlan, IdleWindowSurvivesClockWrap) {
    EXPECT_EQ(next_action(false, false, false, 10, UINT32_MAX - 59989, 60000),
              Action::RELEASE_ESCS);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
