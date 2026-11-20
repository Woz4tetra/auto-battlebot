#include <armed_guard.h>
#include <gtest/gtest.h>

using namespace armed_guard;

TEST(ArmedGuard, EveryWifiDongleDumpAndTuningWriteIsRefusedWhileArmed) {
    for (Write write :
         {Write::WIFI_SETTINGS, Write::DONGLES, Write::CRASH_DUMP, Write::ESC_TUNING}) {
        EXPECT_NE(refusal(write, true), nullptr);
        EXPECT_EQ(refusal(write, false), nullptr);
    }
}

TEST(ArmedGuard, EscTuningEntryIsRefusedWhileArmedOrDriving) {
    EXPECT_EQ(enter_decision(10000, 0, 3000, true, false), EnterDecision::REFUSE_ARMED);
    EXPECT_EQ(enter_decision(10000, 0, 3000, false, true), EnterDecision::REFUSE_DRIVING);
    EXPECT_EQ(enter_decision(10000, 0, 3000, true, true), EnterDecision::REFUSE_ARMED);
    EXPECT_EQ(enter_decision(10000, 0, 3000, false, false), EnterDecision::ENTER);
}

TEST(ArmedGuard, EscTuningEntryWaitsForTheEscsToFinishStarting) {
    EXPECT_EQ(enter_decision(2999, 0, 3000, false, false), EnterDecision::WAIT_STARTUP);
    EXPECT_EQ(enter_decision(3000, 0, 3000, false, false), EnterDecision::ENTER);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
