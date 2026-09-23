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

#include "remote/messages.hpp"

namespace auto_battlebot {

namespace remote {
class StatusBus;
}

/**
 * Host-side jobs the loop must never wait on: opening the dashboard to Wi-Fi and reading the
 * network addresses the System tab shows.
 *
 * One worker thread. set_wifi_access() queues the request and returns; the worker runs
 * `systemctl start|stop auto-battlebot-dashboard-wifi.service`, which a polkit rule from
 * install/install_dashboard_network.sh lets the app user do for that unit only. The setting
 * persists in `state_file` and is applied again at startup, so a phone reaches the box after a
 * reboot without the iPad. The worker publishes /status/network after each change and every
 * `refresh_period`.
 */
class HostServices {
   public:
    /** Runs `systemctl <verb> <unit>`; returns the exit code. Replaced in tests. */
    using SystemctlRunner = std::function<int(const std::string& verb, const std::string& unit)>;

    struct Options {
        std::filesystem::path state_file = default_state_file();
        std::string wifi_unit = "auto-battlebot-dashboard-wifi.service";
        std::chrono::milliseconds refresh_period{5000};
        SystemctlRunner run_systemctl = run_systemctl_command;
    };

    HostServices(Options options, std::shared_ptr<remote::StatusBus> status_bus);
    ~HostServices();

    HostServices(const HostServices&) = delete;
    HostServices& operator=(const HostServices&) = delete;

    /** Never blocks on the subprocess. */
    void set_wifi_access(bool enabled);

    /** The last applied setting. */
    bool wifi_access() const;

    remote::NetworkMessage network() const;

    /** $HOME/.local/state/auto_battlebot/wifi_access. Under $HOME, not config/, because a deploy
     *  overwrites the repo tree. */
    static std::filesystem::path default_state_file();

    /** `systemctl --no-ask-password <verb> <unit>`, so a missing polkit rule fails instead of
     *  prompting on a terminal. */
    static int run_systemctl_command(const std::string& verb, const std::string& unit);

    /** Reads interface addresses with getifaddrs(): the first IPv4 link-local address is the
     *  cable, and the first interface with /sys/class/net/<name>/wireless is the Wi-Fi. */
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
