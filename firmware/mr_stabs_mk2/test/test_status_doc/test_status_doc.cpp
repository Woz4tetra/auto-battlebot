#include <gtest/gtest.h>
#include <status_doc.h>

#include <string>

namespace {

struct FakeScan {
    bool scanned = true;
    uint32_t scanned_at_ms = 900;
    uint8_t count = 2;
    uint8_t addresses[4] = {40, 64, 0, 0};
};

struct FakeLines {
    bool sda_high = true;
    bool scl_high = false;
};

struct FakeImu {
    bool initialized = true;
    uint32_t begin_attempts = 3;
    uint32_t begin_failures = 1;
    uint8_t last_chip_id = 0xA0;
    uint8_t chip_id_read_error = 0;
    uint32_t samples_lost_after_begin = 2;
    uint32_t samples = 50;
    uint32_t last_sample_ms = 990;
    uint32_t details_ms = 0;
    uint8_t details_error = 255;
    uint8_t operation_mode = 8;
    uint8_t sys_status = 5;
    uint8_t self_test = 15;
    uint8_t sys_error = 0;
    uint8_t calibration = 255;
    uint32_t gyro_agree = 11;
    uint32_t gyro_disagree = 4;
};

struct FakeIna {
    bool present = true;
    uint16_t device_id = 0x2381;
    uint8_t last_error = 0;
    uint32_t reads = 7;
    uint32_t read_failures = 0;
    uint32_t last_ok_ms = 0;
};

struct FakeStatus {
    FakeImu imu;
    FakeIna ina;
    FakeScan scan;
    FakeLines lines;
    uint8_t sda_pin = 41;
    uint8_t scl_pin = 40;
    uint32_t snapshot_taken_ms = 1000;
    float yaw_rate = 12.34f;
    float heading_rate = -5.06f;
    bool gyro_sign_suspect = true;
};

JsonDocument build(const FakeStatus &s, bool scan_pending) {
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    status_doc::fill_sensors(root, s, scan_pending);
    return doc;
}

}  // namespace

TEST(StatusDoc, BusSectionCarriesPinsLinesAndFoundAddresses) {
    JsonDocument doc = build(FakeStatus{}, true);
    JsonObject bus = doc["bus"];
    EXPECT_EQ(bus["sda_pin"].as<int>(), 41);
    EXPECT_EQ(bus["scl_pin"].as<int>(), 40);
    EXPECT_TRUE(bus["sda_high"].as<bool>());
    EXPECT_FALSE(bus["scl_high"].as<bool>());
    EXPECT_TRUE(bus["scan_pending"].as<bool>());
    EXPECT_EQ(bus["scan_age_ms"].as<int>(), 100);
    ASSERT_EQ(bus["found"].size(), 2u);
    EXPECT_EQ(bus["found"][0].as<int>(), 40);
    EXPECT_EQ(bus["found"][1].as<int>(), 64);
}

TEST(StatusDoc, AgesAreNullUntilTheEventHappened) {
    FakeStatus s;
    s.scan.scanned = false;
    s.imu.samples = 0;
    JsonDocument doc = build(s, false);
    EXPECT_TRUE(doc["bus"]["scan_age_ms"].isNull());
    EXPECT_TRUE(doc["imu"]["sample_age_ms"].isNull());
    EXPECT_TRUE(doc["imu"]["details_age_ms"].isNull());
    EXPECT_TRUE(doc["ina"]["last_ok_age_ms"].isNull());
    EXPECT_FALSE(doc["bus"]["scan_age_ms"].is<int>());
}

TEST(StatusDoc, ImuSectionRoundsRatesToOneDecimalAndKeepsCounters) {
    JsonDocument doc = build(FakeStatus{}, false);
    JsonObject imu = doc["imu"];
    EXPECT_NEAR(imu["yaw_rate"].as<float>(), 12.3f, 1e-4);
    EXPECT_NEAR(imu["heading_rate"].as<float>(), -5.1f, 1e-4);
    EXPECT_EQ(imu["chip_id"].as<int>(), 0xA0);
    EXPECT_EQ(imu["lost_count"].as<int>(), 2);
    EXPECT_EQ(imu["sample_age_ms"].as<int>(), 10);
    EXPECT_EQ(imu["details_error"].as<int>(), 255);
    EXPECT_TRUE(imu["gyro_sign_suspect"].as<bool>());
}

TEST(StatusDoc, InaSectionCarriesDeviceId) {
    JsonDocument doc = build(FakeStatus{}, false);
    EXPECT_EQ(doc["ina"]["device_id"].as<int>(), 0x2381);
    EXPECT_EQ(doc["ina"]["reads"].as<int>(), 7);
}

TEST(StatusDoc, NestedSectionsAppendedAfterwardsDoNotWipeEarlierKeys) {
    JsonDocument doc = build(FakeStatus{}, false);
    JsonObject root = doc.as<JsonObject>();
    root["espnow"].to<JsonObject>()["enabled"] = true;
    root["boot"].to<JsonObject>()["has_dump"] = false;
    std::string out;
    serializeJson(doc, out);
    JsonDocument parsed;
    ASSERT_FALSE(deserializeJson(parsed, out));
    EXPECT_EQ(parsed["snapshot_ms"].as<int>(), 1000);
    EXPECT_EQ(parsed["bus"]["sda_pin"].as<int>(), 41);
    EXPECT_TRUE(parsed["espnow"]["enabled"].as<bool>());
    EXPECT_FALSE(parsed["boot"]["has_dump"].as<bool>());
    EXPECT_LT(out.find("\"ina\""), out.find("\"espnow\""));
    EXPECT_LT(out.find("\"espnow\""), out.find("\"boot\""));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
