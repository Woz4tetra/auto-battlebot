#pragma once

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "remote/messages.hpp"

namespace auto_battlebot {

namespace remote {
class StatusBus;
}

class HostServices {
   public:
    using SystemctlRunner = std::function<int(const std::string& verb, const std::string& unit)>;

    struct Options {
        std::filesystem::path state_file = default_state_file();
        std::string wifi_unit = "dashboard-wifi.service";
        std::chrono::milliseconds refresh_period{5000};
        SystemctlRunner run_systemctl = run_systemctl_command;
    };

    HostServices(Options options, std::shared_ptr<remote::StatusBus> status_bus);
    ~HostServices();

    HostServices(const HostServices&) = delete;
    HostServices& operator=(const HostServices&) = delete;

    void set_wifi_access(bool enabled);

    bool wifi_access() const;

    remote::NetworkMessage network() const;

    static std::filesystem::path default_state_file();

    static std::vector<std::string> systemctl_arguments(const std::string& verb,
                                                        const std::string& unit);
    static int run_systemctl_command(const std::string& verb, const std::string& unit);

    static remote::NetworkMessage read_network();

   private:
    void run();
    bool apply_wifi_access(bool enabled);
    std::optional<bool> read_state_file() const;
    void write_state_file(bool enabled) const;
    void publish_network();

    Options options_;
    std::shared_ptr<remote::StatusBus> status_bus_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<bool> requested_;
    bool wifi_access_ = false;
    remote::NetworkMessage network_;
    bool stop_ = false;
    std::thread thread_;
};

}  // namespace auto_battlebot
