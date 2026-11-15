#include <gtest/gtest.h>
#include <motor_guard.h>

#include <cmath>
#include <deque>

using motor_guard::Config;
using motor_guard::MotorGuard;
using motor_guard::Sensors;
using motor_guard::State;

namespace {
const float kInaSamplePeriodS = 0.01f;
const float kBudgetUseTolerance_c = 3.0f;
const int kFirmwareExtraDelaySamples = 2;
const float kUnderPeakPercent = 32.0f;
const float kHeatLowRatio = 0.9f;
const float kHeatHighRatio = 1.2f;
const int kSubsteps = 10;
const float kPackVolts = 15.2f;
const float kTwoPi = 6.2831853f;
const float kSlowestNonzeroErpm = 919.0f;

// Two motors on a stiff pack. Current is the averaged ESC model the guard assumes,
// I = (d V - Ke w) / R, with R 15% below the guard's R_cold so the shunt gain has something to
// correct (g only corrects upward). A DShot stop freewheels: no current.
struct Plant {
    Config config;
    float r_ohm;
    float inertia_kg_m2 = 1.24e-6f;
    float friction_n_m_s_per_rad = 1e-6f;
    float speed[2] = {0.0f, 0.0f};
    bool locked[2] = {false, false};
    float blheli_restart_below_rad_s = 0.0f;
    float slow_duty_fraction = 1.0f;
    float avg_amps[2] = {0.0f, 0.0f};
    float avg_pack = 0.0f;

    explicit Plant(const Config &c) : config(c), r_ohm(c.r_cold_ohm / 1.15f) {}

    float motor_amps(int k, float duty) const {
        if (duty == 0.0f) return 0.0f;
        if (std::fabs(speed[k]) < blheli_restart_below_rad_s) duty *= slow_duty_fraction;
        return (duty * kPackVolts - config.ke_v_s_per_rad * speed[k]) / r_ohm;
    }

    void step(const float duty[2]) {
        const float h = kInaSamplePeriodS / kSubsteps;
        avg_amps[0] = avg_amps[1] = 0.0f;
        avg_pack = 0.0f;
        for (int s = 0; s < kSubsteps; s++) {
            float pack = config.logic_amps;
            for (int k = 0; k < 2; k++) {
                const float amps = motor_amps(k, duty[k]);
                avg_amps[k] += amps / kSubsteps;
                const float applied =
                    std::fabs(speed[k]) < blheli_restart_below_rad_s ? slow_duty_fraction : 1.0f;
                pack += duty[k] * applied * amps;
                if (locked[k]) {
                    speed[k] = 0.0f;
                } else {
                    speed[k] += h *
                                (config.ke_v_s_per_rad * amps - friction_n_m_s_per_rad * speed[k]) /
                                inertia_kg_m2;
                }
            }
            avg_pack += pack / kSubsteps;
        }
    }

    float erpm(int k) const {
        const float value = std::fabs(speed[k]) * 60.0f * config.pole_pairs / kTwoPi;
        return value < kSlowestNonzeroErpm ? 0.0f : std::floor(value);
    }
};

// Runs the guard against the plant the way main.cpp does: update with the previous sample's
// duty and the measured pack current, then scale this sample's command.
struct Sim {
    Config config;
    MotorGuard guard;
    Plant plant;
    int pack_amps_delay_samples;
    std::deque<float> pack_history;
    float duty[2] = {0.0f, 0.0f};
    bool shunt_ok = true;
    bool erpm_ok[2] = {true, true};
    float time_s = 0.0f;

    explicit Sim(const Config &c = Config(), int delay = 1)
        : config(c), guard(c), plant(c), pack_amps_delay_samples(delay) {}

    static float to_duty(float percent) {
        return std::fabs(percent) < 1.0f ? 0.0f : percent / 100.0f;
    }

    void tick(float left_percent, float right_percent) {
        pack_history.push_back(plant.avg_pack);
        while ((int)pack_history.size() > pack_amps_delay_samples) pack_history.pop_front();
        const float amps = shunt_ok ? pack_history.front() : NAN;
        const float volts = shunt_ok ? kPackVolts : NAN;
        guard.update(kInaSamplePeriodS, volts, amps, duty[0], duty[1],
                     erpm_ok[0] ? plant.erpm(0) : -1.0f, erpm_ok[1] ? plant.erpm(1) : -1.0f);
        guard.apply(left_percent, right_percent);
        duty[0] = to_duty(left_percent);
        duty[1] = to_duty(right_percent);
        plant.step(duty);
        time_s += kInaSamplePeriodS;
    }

    void run(float seconds, float left_percent, float right_percent) {
        const int ticks = (int)std::lround(seconds / kInaSamplePeriodS);
        for (int i = 0; i < ticks; i++) tick(left_percent, right_percent);
    }
};

float closed_form_time(const Config &c, float amps, float rise_c) {
    return -c.tau_s * std::log(1.0f - rise_c / (c.k_c_per_a2s * c.tau_s * amps * amps));
}
}  // namespace

TEST(MotorGuard, ColdAndLightlyLoadedPassesThroughBitForBit) {
    Sim sim;
    for (int i = 0; i < 500; i++) {
        const float command = 30.0f + 20.0f * std::sin(i * 0.05f);
        float left = command, right = -command * 0.7f;
        sim.tick(command, -command * 0.7f);
        sim.guard.apply(left, right);
        EXPECT_EQ(left, command);
        EXPECT_EQ(right, -command * 0.7f);
        ASSERT_EQ(sim.guard.scale(), 1.0f);
    }
    EXPECT_EQ(sim.guard.state(), State::NOMINAL);
    EXPECT_EQ(sim.guard.sensors(), Sensors::FUSED);
}

TEST(MotorGuard, SoftLimitTimeMatchesClosedForm) {
    Config config;
    config.alpha_per_c = 0.0f;
    Sim sim(config);
    sim.plant.locked[0] = sim.plant.locked[1] = true;
    const float percent = 30.0f;
    const float true_amps = percent / 100.0f * kPackVolts / sim.plant.r_ohm;
    while (sim.guard.rise_c(0) < config.rise_soft_c && sim.time_s < 60.0f) {
        sim.tick(percent, percent);
    }
    EXPECT_NEAR(sim.time_s, closed_form_time(config, true_amps, config.rise_soft_c),
                0.05f * closed_form_time(config, true_amps, config.rise_soft_c));
}

TEST(MotorGuard, DualStallAtFullCommandNeverPassesMaxRise) {
    Config config;
    Sim sim(config);
    sim.plant.locked[0] = sim.plant.locked[1] = true;
    float peak_rise = 0.0f;
    for (int i = 0; i < 6000; i++) {
        sim.tick(100.0f, 100.0f);
        peak_rise = std::fmax(peak_rise, sim.guard.rise_c(0));
    }
    EXPECT_LT(peak_rise, config.rise_max_c + 1.0f);
    EXPECT_GT(peak_rise, config.rise_max_c - kBudgetUseTolerance_c);
    // Held at I_cont the duty is under stall_duty, so the stall flag has cleared by now.
    EXPECT_EQ(sim.guard.state(), State::LIMITING);
    EXPECT_FALSE(sim.guard.tripped());
}

namespace {
Sim tripped_sim(float trip_rise_c) {
    Config config;
    config.trip_rise_c = trip_rise_c;
    Sim sim(config);
    sim.plant.locked[0] = sim.plant.locked[1] = true;
    while (!sim.guard.tripped() && sim.time_s < 60.0f) sim.tick(100.0f, 100.0f);
    return sim;
}
}  // namespace

TEST(MotorGuard, TripLatchesAndCutsDriveUntilCleared) {
    Sim sim = tripped_sim(5.0f);
    ASSERT_TRUE(sim.guard.tripped());
    EXPECT_GE(std::fmax(sim.guard.rise_c(0), sim.guard.rise_c(1)), 5.0f);
    EXPECT_EQ(sim.guard.state(), State::TRIPPED);
    float left = 80.0f, right = -60.0f;
    sim.guard.apply(left, right);
    EXPECT_EQ(left, 0.0f);
    EXPECT_EQ(right, 0.0f);

    sim.plant.locked[0] = sim.plant.locked[1] = false;
    sim.run(120.0f, 100.0f, 100.0f);
    EXPECT_LT(sim.guard.rise_c(0), 5.0f);
    EXPECT_TRUE(sim.guard.tripped());
    EXPECT_EQ(sim.duty[0], 0.0f);
    EXPECT_EQ(sim.duty[1], 0.0f);

    sim.guard.clear_trip();
    sim.tick(30.0f, 30.0f);
    EXPECT_FALSE(sim.guard.tripped());
    EXPECT_NE(sim.guard.state(), State::TRIPPED);
    EXPECT_GT(sim.duty[0], 0.0f);
}

TEST(MotorGuard, ClearedWhileHotTripsAgain) {
    Sim sim = tripped_sim(5.0f);
    ASSERT_TRUE(sim.guard.tripped());
    sim.guard.clear_trip();
    sim.tick(0.0f, 0.0f);
    EXPECT_TRUE(sim.guard.tripped());
}

TEST(MotorGuard, TripOutlastsSensorLoss) {
    Sim sim = tripped_sim(5.0f);
    ASSERT_TRUE(sim.guard.tripped());
    sim.shunt_ok = false;
    sim.erpm_ok[0] = sim.erpm_ok[1] = false;
    sim.run(1.0f, 100.0f, 100.0f);
    EXPECT_EQ(sim.guard.sensors(), Sensors::NONE);
    EXPECT_EQ(sim.guard.state(), State::TRIPPED);
    EXPECT_EQ(sim.duty[0], 0.0f);
}

TEST(MotorGuard, LimiterSettlesWithoutOvershootOrOscillation) {
    Config config;
    Sim sim(config, kFirmwareExtraDelaySamples);
    sim.plant.locked[0] = sim.plant.locked[1] = true;
    sim.run(1.0f, 100.0f, 100.0f);
    float max_amps = 0.0f, min_scale = 1.0f, max_scale = 0.0f;
    for (int i = 0; i < 200; i++) {
        sim.tick(100.0f, 100.0f);
        max_amps = std::fmax(max_amps, std::fabs(sim.plant.avg_amps[0]));
        min_scale = std::fmin(min_scale, sim.guard.scale());
        max_scale = std::fmax(max_scale, sim.guard.scale());
    }
    EXPECT_LT(max_amps, 1.1f * config.peak_amps);
    EXPECT_GT(max_amps, 0.8f * config.peak_amps);
    EXPECT_LT(max_scale - min_scale, 0.02f);
}

TEST(MotorGuard, SingleStallEstimatesEachMotor) {
    Sim sim;
    sim.plant.locked[0] = true;
    sim.run(2.0f, kUnderPeakPercent, kUnderPeakPercent);
    ASSERT_EQ(sim.guard.scale(), 1.0f);
    for (int k = 0; k < 2; k++) {
        const float truth = sim.plant.avg_amps[k];
        EXPECT_NEAR(sim.guard.amps_estimate(k), truth, 0.05f * std::fmax(std::fabs(truth), 1.0f))
            << "motor " << k;
    }
    EXPECT_TRUE(sim.guard.stalled(0));
    EXPECT_FALSE(sim.guard.stalled(1));
}

TEST(MotorGuard, ShuntOnlyEstimatesTheStalledMotor) {
    Sim sim;
    // The model bound d V / R_cold only holds with R_cold the lowest resistance the circuit has.
    sim.plant.r_ohm = sim.config.r_cold_ohm;
    sim.erpm_ok[0] = sim.erpm_ok[1] = false;
    sim.plant.locked[0] = true;
    sim.run(1.0f, 30.0f, 30.0f);
    EXPECT_EQ(sim.guard.sensors(), Sensors::SHUNT_ONLY);
    const float truth = sim.plant.avg_amps[0];
    EXPECT_NEAR(sim.guard.amps_estimate(0), truth, 0.10f * truth);
    EXPECT_GE(sim.guard.amps_estimate(1), sim.plant.avg_amps[1]);
}

TEST(MotorGuard, GainConvergesToResistanceRatioAndHolds) {
    Sim sim;
    sim.plant.locked[0] = true;
    sim.run(2.0f, 50.0f, 50.0f);
    const float ratio = sim.plant.r_ohm / sim.config.r_cold_ohm;
    const float expected = ratio / (1.0f + sim.config.alpha_per_c * sim.guard.rise_c(0));
    EXPECT_NEAR(1.0f / sim.guard.shunt_gain(), expected, 0.05f * expected);
    sim.plant.locked[0] = false;
    sim.run(0.05f, 0.0f, 0.0f);
    const float held = sim.guard.shunt_gain();
    sim.run(2.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(sim.guard.shunt_gain(), held);
}

TEST(MotorGuard, PluggingAndRegenTrackTrueCurrent) {
    Sim sim;
    sim.plant.locked[0] = true;
    sim.run(2.0f, 40.0f, 40.0f);
    sim.plant.locked[0] = false;
    sim.run(1.0f, 100.0f, 100.0f);
    ASSERT_GT(sim.guard.motor_shaft_speed_signed_rad_s(0), 1000.0f);
    // Full reverse at speed. eRPM never reads 0 at this sample rate, so the shunt tiebreak must
    // flip the sign. Until it does, the estimate errs high.
    int misses = 0;
    float true_heat = 0.0f, estimated_heat = 0.0f;
    for (int i = 0; i < 100; i++) {
        const float truth = sim.plant.motor_amps(0, sim.duty[0]);
        sim.tick(-100.0f, -100.0f);
        const float estimate = sim.guard.amps_estimate(0);
        true_heat += truth * truth;
        estimated_heat += estimate * estimate;
        if (std::fabs(estimate - truth) > 0.10f * std::fmax(std::fabs(truth), 2.0f)) {
            misses++;
            EXPECT_GT(std::fabs(estimate), std::fabs(truth)) << "tick " << i;
        }
    }
    EXPECT_LE(misses, 4);
    EXPECT_GT(estimated_heat, kHeatLowRatio * true_heat);
    EXPECT_LT(estimated_heat, kHeatHighRatio * true_heat);
    EXPECT_LT(sim.guard.motor_shaft_speed_signed_rad_s(0), -1000.0f);
}

TEST(MotorGuard, StallFlagNeedsHeldLockedRotor) {
    Sim sim;
    sim.plant.locked[0] = true;
    sim.run(0.08f, 40.0f, 40.0f);
    EXPECT_FALSE(sim.guard.stalled(0));
    sim.run(0.05f, 40.0f, 40.0f);
    EXPECT_TRUE(sim.guard.stalled(0));
    sim.plant.locked[0] = false;
    sim.run(0.05f, 40.0f, 40.0f);
    EXPECT_FALSE(sim.guard.stalled(0));
    Sim launch;
    for (int i = 0; i < 50; i++) {
        launch.tick(100.0f, 100.0f);
        ASSERT_FALSE(launch.guard.stalled(0));
        ASSERT_FALSE(launch.guard.stalled(1));
    }
}

TEST(MotorGuard, LaunchesFromRestDoNotStutter) {
    Sim sim(Config(), 2);
    for (int launch = 0; launch < 5; launch++) {
        for (float command : {100.0f, -100.0f}) {
            for (int i = 0; i < 100; i++) {
                sim.tick(command, command);
                ASSERT_EQ(sim.guard.scale(), 1.0f) << "launch " << launch << " tick " << i;
            }
            sim.run(0.5f, 0.0f, 0.0f);
        }
    }
    EXPECT_FLOAT_EQ(sim.guard.shunt_gain(), 1.0f);
}

TEST(MotorGuard, EscDutyClampAtLowSpeedDoesNotLimit) {
    // BLHeli restarts a reversed motor at a fraction of the commanded duty. The model alone
    // overestimates; the shunt bound must keep the guard from cutting power.
    Sim sim(Config(), 2);
    sim.plant.blheli_restart_below_rad_s = 800.0f;
    sim.plant.slow_duty_fraction = 0.3f;
    for (int reversal = 0; reversal < 4; reversal++) {
        const float command = reversal % 2 ? -100.0f : 100.0f;
        for (int i = 0; i < 150; i++) {
            sim.tick(command, command);
            ASSERT_EQ(sim.guard.scale(), 1.0f) << "reversal " << reversal << " tick " << i;
        }
    }
}

TEST(MotorGuard, HighResistanceErrsTowardLimitingEarly) {
    Config config;
    config.alpha_per_c = 0.0f;
    Sim sim(config);
    sim.plant.r_ohm = config.r_cold_ohm * 1.15f;
    sim.plant.locked[0] = sim.plant.locked[1] = true;
    const float percent = 36.0f;
    const float true_amps = percent / 100.0f * kPackVolts / sim.plant.r_ohm;
    while (sim.guard.rise_c(0) < config.rise_soft_c && sim.time_s < 60.0f) {
        sim.tick(percent, percent);
    }
    EXPECT_EQ(sim.guard.shunt_gain(), 1.0f);
    EXPECT_LT(sim.time_s, closed_form_time(config, true_amps, config.rise_soft_c));
}

TEST(MotorGuard, CoolingFollowsTau) {
    Sim sim;
    sim.guard.restore(sim.config.rise_max_c, sim.config.rise_max_c);
    const float target = sim.config.rise_max_c / 2.0f;
    while (sim.guard.rise_c(0) > target && sim.time_s < 300.0f) sim.tick(0.0f, 0.0f);
    const float expected = sim.config.tau_s * std::log(2.0f);
    EXPECT_NEAR(sim.time_s, expected, 0.02f * expected);
}

TEST(MotorGuard, RecoveryIsRateLimited) {
    Sim sim;
    sim.plant.locked[0] = sim.plant.locked[1] = true;
    sim.run(20.0f, 100.0f, 100.0f);
    ASSERT_LT(sim.guard.scale(), 0.5f);
    sim.plant.locked[0] = sim.plant.locked[1] = false;
    float previous = std::log(sim.guard.scale());
    for (int i = 0; i < 500; i++) {
        sim.tick(100.0f, 100.0f);
        const float now = std::log(sim.guard.scale());
        ASSERT_LE(now - previous, sim.config.recover_per_s * kInaSamplePeriodS + 1e-5f);
        previous = now;
    }
    EXPECT_EQ(sim.guard.scale(), 1.0f);
}

TEST(MotorGuard, LosingOneEncoderMidStallKeepsTheHeat) {
    Sim sim;
    sim.plant.locked[0] = sim.plant.locked[1] = true;
    sim.run(5.0f, 100.0f, 100.0f);
    const float rise_before = sim.guard.rise_c(0);
    sim.erpm_ok[0] = false;
    sim.tick(100.0f, 100.0f);
    EXPECT_EQ(sim.guard.sensors(), Sensors::SHUNT_ONLY);
    EXPECT_NEAR(sim.guard.rise_c(0), rise_before, 0.5f);
    sim.run(2.0f, 100.0f, 100.0f);
    EXPECT_NEAR(sim.guard.amps_estimate(0), sim.plant.avg_amps[0], 0.15f * sim.plant.avg_amps[0]);
    EXPECT_NEAR(sim.guard.amps_estimate(1), sim.plant.avg_amps[1], 0.10f * sim.plant.avg_amps[1]);
    EXPECT_EQ(sim.guard.state(), State::LIMITING);
}

TEST(MotorGuard, ErpmOnlyFreeRunNeverLimits) {
    Sim sim;
    sim.shunt_ok = false;
    sim.run(10.0f, 100.0f, 100.0f);
    EXPECT_EQ(sim.guard.sensors(), Sensors::ERPM_ONLY);
    EXPECT_EQ(sim.guard.scale(), 1.0f);
}

TEST(MotorGuard, ErpmOnlyDualStallStillLimits) {
    float time_to_soft[2] = {0.0f, 0.0f};
    for (int mode = 0; mode < 2; mode++) {
        Sim sim;
        sim.shunt_ok = mode == 1;
        sim.plant.locked[0] = sim.plant.locked[1] = true;
        while (sim.guard.rise_c(0) < sim.config.rise_soft_c && sim.time_s < 60.0f) {
            sim.tick(100.0f, 100.0f);
        }
        EXPECT_EQ(sim.guard.state(), State::LIMITING) << "mode " << mode;
        time_to_soft[mode] = sim.time_s;
    }
    EXPECT_NEAR(time_to_soft[0], time_to_soft[1], 0.3f * time_to_soft[1]);
}

TEST(MotorGuard, NoSensorsIsOffAndCools) {
    Sim sim;
    sim.guard.restore(40.0f, 40.0f);
    sim.shunt_ok = false;
    sim.erpm_ok[0] = sim.erpm_ok[1] = false;
    sim.plant.locked[0] = true;
    sim.run(5.0f, 100.0f, 100.0f);
    EXPECT_EQ(sim.guard.state(), State::OFF);
    EXPECT_EQ(sim.guard.scale(), 1.0f);
    EXPECT_LT(sim.guard.rise_c(0), 40.0f);
}

TEST(MotorGuard, LongGapIsClampedToMaxDt) {
    MotorGuard clamped{Config()}, reference{Config()};
    clamped.restore(30.0f, 30.0f);
    reference.restore(30.0f, 30.0f);
    clamped.update(2.0f, kPackVolts, 0.3f, 0.0f, 0.0f, 0.0f, 0.0f);
    reference.update(Config().max_dt_s, kPackVolts, 0.3f, 0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(clamped.rise_c(0), reference.rise_c(0));
}

TEST(MotorGuard, StoppedMotorsDrawNothing) {
    MotorGuard guard{Config()};
    guard.update(kInaSamplePeriodS, kPackVolts, 0.3f, 0.0f, 0.0f, 20000.0f, 20000.0f);
    EXPECT_EQ(guard.amps_estimate(0), 0.0f);
    EXPECT_EQ(guard.amps_estimate(1), 0.0f);
}

TEST(MotorGuard, RestoreClamps) {
    MotorGuard guard{Config()};
    guard.restore(-5.0f, 1e6f);
    EXPECT_EQ(guard.rise_c(0), 0.0f);
    EXPECT_EQ(guard.rise_c(1), 2.0f * Config().rise_max_c);
    guard.restore(NAN, 10.0f);
    EXPECT_EQ(guard.rise_c(0), 0.0f);
    EXPECT_EQ(guard.rise_c(1), 10.0f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
