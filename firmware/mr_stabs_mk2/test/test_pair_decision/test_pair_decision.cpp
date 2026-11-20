#include <gtest/gtest.h>
#include <pair_decision.h>

using namespace pair_decision;

TEST(PairDecision, ArmedRefusesBeforeAnyOtherCheck) {
    EXPECT_EQ(decide(true, 0x01, true, 8, 8), Decision::REFUSE_ARMED);
    EXPECT_EQ(decide(true, 0x02, false, 0, 8), Decision::REFUSE_ARMED);
}

TEST(PairDecision, DisarmedRefusesMulticastMacs) {
    EXPECT_EQ(decide(false, 0x01, false, 0, 8), Decision::REFUSE_BAD_MAC);
    EXPECT_EQ(decide(false, 0xFF, true, 8, 8), Decision::REFUSE_BAD_MAC);
}

TEST(PairDecision, DisarmedRefusesAKnownDongleBeforeCheckingCapacity) {
    EXPECT_EQ(decide(false, 0x02, true, 8, 8), Decision::REFUSE_EXISTS);
}

TEST(PairDecision, DisarmedRefusesWhenTheTableIsFull) {
    EXPECT_EQ(decide(false, 0x02, false, 8, 8), Decision::REFUSE_FULL);
}

TEST(PairDecision, DisarmedAddsAUnicastDongleWhileThereIsRoom) {
    EXPECT_EQ(decide(false, 0x02, false, 7, 8), Decision::ADD);
    EXPECT_EQ(decide(false, 0x02, false, 0, 8), Decision::ADD);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
