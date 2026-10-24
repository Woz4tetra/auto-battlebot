#include <esc_map.h>
#include <gtest/gtest.h>

using namespace esc_map;

TEST(EscMap, InsideStopThresholdSendsStop) {
    EXPECT_EQ(percent_to_dshot_3d(0.0f, 1.0f), DSHOT_STOP);
    EXPECT_EQ(percent_to_dshot_3d(0.99f, 1.0f), DSHOT_STOP);
    EXPECT_EQ(percent_to_dshot_3d(-0.99f, 1.0f), DSHOT_STOP);
    EXPECT_EQ(percent_to_dshot_3d(0.0f, 0.0f), DSHOT_STOP);
}

TEST(EscMap, SmallestCommandStartsAtTheFloor) {
    EXPECT_EQ(percent_to_dshot_3d(1.0f, 1.0f), DSHOT_3D_FWD_MIN + FLOOR_STEP + 9);
    EXPECT_EQ(percent_to_dshot_3d(0.01f, 0.0f), DSHOT_3D_FWD_MIN + FLOOR_STEP);
    EXPECT_EQ(percent_to_dshot_3d(-0.01f, 0.0f), DSHOT_3D_REV_MIN + FLOOR_STEP);
}

TEST(EscMap, FullScaleReachesTheTopStep) {
    EXPECT_EQ(percent_to_dshot_3d(100.0f, 1.0f), 2047);
    EXPECT_EQ(percent_to_dshot_3d(-100.0f, 1.0f), 1047);
    EXPECT_EQ(percent_to_dshot_3d(250.0f, 1.0f), 2047);
    EXPECT_EQ(percent_to_dshot_3d(-250.0f, 1.0f), 1047);
}

TEST(EscMap, MonotonicAndSymmetric) {
    uint16_t last = 0;
    for (int i = 1; i <= 1000; i++) {
        float p = i / 10.0f;
        uint16_t fwd = percent_to_dshot_3d(p, 0.0f);
        uint16_t rev = percent_to_dshot_3d(-p, 0.0f);
        EXPECT_GE(fwd, last);
        EXPECT_EQ(fwd - DSHOT_3D_FWD_MIN, rev - DSHOT_3D_REV_MIN);
        last = fwd;
    }
}

TEST(EscMap, DutyMatchesTheStepSent) {
    EXPECT_FLOAT_EQ(duty(0.5f, 1.0f), 0.0f);
    EXPECT_NEAR(duty(1.0f, 1.0f), (FLOOR_STEP + 9) / 999.0f, 1e-6f);
    EXPECT_NEAR(duty(-50.0f, 1.0f), -(FLOOR_STEP + 466) / 999.0f, 1e-6f);
    EXPECT_FLOAT_EQ(duty(100.0f, 1.0f), 1.0f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
