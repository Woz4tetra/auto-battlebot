#include <gtest/gtest.h>
#include <wifi_types.h>

using namespace wifi_manager;

TEST(WifiSnapshot, CarriesExactlyTheElevenKnownFieldsAndNoPassword) {
    snapshot_t snapshot = {};
    auto &[mode, state, forced_ap, selected, count, networks, joining_ssid, ip, rssi,
           connect_ms_left, stations] = snapshot;
    (void)mode;
    (void)state;
    (void)forced_ap;
    (void)selected;
    (void)count;
    (void)networks;
    (void)joining_ssid;
    (void)ip;
    (void)rssi;
    (void)connect_ms_left;
    (void)stations;
    SUCCEED();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
