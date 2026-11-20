#include <gtest/gtest.h>
#include <ina238_sampler.h>

#include <cmath>
#include <functional>
#include <vector>

using namespace ina238_id;

struct Bus {
    bool bus_ok = true;
    bool shunt_ok = true;
    uint16_t bus_raw = 0;
    uint16_t shunt_raw = 0;
    std::vector<uint8_t> reads;
    bool operator()(uint8_t reg, uint16_t *value) {
        reads.push_back(reg);
        if (reg == Sampler::BUS_REGISTER) {
            *value = bus_raw;
            return bus_ok;
        }
        *value = shunt_raw;
        return shunt_ok;
    }
};

TEST(Ina238Sampler, MissingChipReadsNaNAndNeverTouchesTheBus) {
    Sampler sampler;
    sampler.begin(false, 0);
    Bus bus;
    EXPECT_EQ(sampler.update(5000000, std::ref(bus)), SampleResult::SKIPPED);
    EXPECT_TRUE(std::isnan(sampler.volts()));
    EXPECT_TRUE(std::isnan(sampler.amps()));
    EXPECT_TRUE(bus.reads.empty());
}

TEST(Ina238Sampler, FirstUpdateSamplesImmediately) {
    Sampler sampler;
    sampler.begin(true, 123456);
    Bus bus;
    bus.bus_raw = 4096;
    bus.shunt_raw = 200;
    EXPECT_EQ(sampler.update(123456, std::ref(bus)), SampleResult::READ_OK);
    EXPECT_FLOAT_EQ(sampler.volts(), 4096 * 3.125e-3f);
    EXPECT_FLOAT_EQ(sampler.amps(), 200 * (5e-6f / 0.0005f));
}

TEST(Ina238Sampler, SignedRawValuesGoNegative) {
    Sampler sampler;
    sampler.begin(true, 0);
    Bus bus;
    bus.shunt_raw = 0xFF38;
    sampler.update(0, std::ref(bus));
    EXPECT_FLOAT_EQ(sampler.amps(), -200 * (5e-6f / 0.0005f));
}

TEST(Ina238Sampler, BusReadFailureGivesNaNAndBacksOffToTheRetryInterval) {
    Sampler sampler;
    sampler.begin(true, 0);
    Bus bus;
    bus.bus_raw = 4096;
    bus.shunt_raw = 200;
    EXPECT_EQ(sampler.update(0, std::ref(bus)), SampleResult::READ_OK);
    bus.bus_ok = false;
    EXPECT_EQ(sampler.update(10000, std::ref(bus)), SampleResult::READ_FAILED);
    EXPECT_TRUE(std::isnan(sampler.volts()));
    EXPECT_TRUE(std::isnan(sampler.amps()));
    bus.bus_ok = true;
    EXPECT_EQ(sampler.update(1009999, std::ref(bus)), SampleResult::SKIPPED);
    EXPECT_EQ(sampler.update(1010000, std::ref(bus)), SampleResult::READ_OK);
}

TEST(Ina238Sampler, ShuntFailureKeepsVoltsAndNaNsAmps) {
    Sampler sampler;
    sampler.begin(true, 0);
    Bus bus;
    bus.bus_raw = 100;
    bus.shunt_ok = false;
    EXPECT_EQ(sampler.update(0, std::ref(bus)), SampleResult::READ_OK);
    EXPECT_FLOAT_EQ(sampler.volts(), 100 * 3.125e-3f);
    EXPECT_TRUE(std::isnan(sampler.amps()));
}

TEST(Ina238Sampler, SamplesNoFasterThanTheSampleInterval) {
    Sampler sampler;
    sampler.begin(true, 0);
    Bus bus;
    EXPECT_EQ(sampler.update(0, std::ref(bus)), SampleResult::READ_OK);
    EXPECT_EQ(sampler.update(9999, std::ref(bus)), SampleResult::SKIPPED);
    EXPECT_EQ(sampler.update(10000, std::ref(bus)), SampleResult::READ_OK);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
