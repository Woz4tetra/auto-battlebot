#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#include "host/host_services.hpp"

namespace auto_battlebot {
namespace {

struct FakeSystemctl {
    std::mutex mutex;
    std::vector<std::string> calls;
    std::chrono::milliseconds delay{0};
    int rc = 0;

    HostServices::SystemctlRunner runner() {
        return [this](const std::string& verb, const std::string& unit) {
            std::this_thread::sleep_for(delay);
            std::lock_guard<std::mutex> lock(mutex);
            calls.push_back(verb + " " + unit);
            return rc;
        };
    }
    size_t count() {
        std::lock_guard<std::mutex> lock(mutex);
        return calls.size();
    }
};

class HostServicesTest : public ::testing::Test {
   protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("host_services_test_" + std::to_string(::getpid()));
        std::filesystem::remove_all(dir_);
    }
    void TearDown() override { std::filesystem::remove_all(dir_); }

    HostServices::Options options(FakeSystemctl& fake) {
        HostServices::Options o;
        o.state_file = dir_ / "wifi_access";
        o.run_systemctl = fake.runner();
        o.refresh_period = std::chrono::milliseconds(50);
        return o;
    }

    static bool wait_for(const std::function<bool()>& done) {
        for (int i = 0; i < 200; ++i) {
            if (done()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    std::filesystem::path dir_;
};

TEST_F(HostServicesTest, SetWifiAccessDoesNotWaitForSystemctl) {
    FakeSystemctl fake;
    fake.delay = std::chrono::milliseconds(200);
    HostServices host(options(fake), nullptr);
    const auto start = std::chrono::steady_clock::now();
    host.set_wifi_access(true);
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(50));
    ASSERT_TRUE(wait_for([&] { return host.wifi_access(); }));
    EXPECT_EQ(fake.calls.front(), "start auto-battlebot-dashboard-wifi.service");
}

TEST_F(HostServicesTest, SettingPersistsAndIsAppliedAtStartup) {
    {
        FakeSystemctl fake;
        HostServices host(options(fake), nullptr);
        host.set_wifi_access(true);
        ASSERT_TRUE(wait_for([&] { return host.wifi_access(); }));
    }
    std::ifstream in(dir_ / "wifi_access");
    std::string word;
    in >> word;
    EXPECT_EQ(word, "on");

    FakeSystemctl fake;
    HostServices host(options(fake), nullptr);
    ASSERT_TRUE(wait_for([&] { return host.wifi_access(); }));
    EXPECT_EQ(fake.calls.front(), "start auto-battlebot-dashboard-wifi.service");
}

TEST_F(HostServicesTest, FailedSystemctlKeepsTheOldSetting) {
    FakeSystemctl fake;
    fake.rc = 1;
    HostServices host(options(fake), nullptr);
    host.set_wifi_access(true);
    ASSERT_TRUE(wait_for([&] { return fake.count() == 1; }));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(host.wifi_access());
    EXPECT_FALSE(std::filesystem::exists(dir_ / "wifi_access"));
}

TEST_F(HostServicesTest, SavedOffRunsNothingAtStartup) {
    std::filesystem::create_directories(dir_);
    std::ofstream(dir_ / "wifi_access") << "off\n";
    FakeSystemctl fake;
    HostServices host(options(fake), nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(fake.count(), 0u);
    EXPECT_FALSE(host.wifi_access());
}

TEST(HostServicesNetworkTest, ReadsHostname) {
    EXPECT_FALSE(HostServices::read_network().hostname.empty());
}

}  // namespace
}  // namespace auto_battlebot
