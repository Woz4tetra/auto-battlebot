#include "host/host_services.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <spawn.h>
#include <spdlog/spdlog.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>

#include "remote/status_bus.hpp"

namespace auto_battlebot {

HostServices::HostServices(Options options, std::shared_ptr<remote::StatusBus> status_bus)
    : options_(std::move(options)), status_bus_(std::move(status_bus)) {
    // Apply a saved "on" at startup. The unit is not enabled at boot, so "off" needs no action.
    if (read_state_file().value_or(false)) requested_ = true;
    thread_ = std::thread([this] { run(); });
}

HostServices::~HostServices() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void HostServices::set_wifi_access(bool enabled) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        requested_ = enabled;
    }
    cv_.notify_all();
}

bool HostServices::wifi_access() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return wifi_access_;
}

remote::NetworkMessage HostServices::network() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return network_;
}

std::filesystem::path HostServices::default_state_file() {
    const char* home = std::getenv("HOME");
    std::filesystem::path base = home ? home : "/tmp";
    return base / ".local" / "state" / "auto_battlebot" / "wifi_access";
}

int HostServices::run_systemctl_command(const std::string& verb, const std::string& unit) {
    // posix_spawn rather than std::system: no shell, and no fork of a process holding CUDA and
    // camera threads.
    std::string arg0 = "systemctl", arg1 = "--no-ask-password", arg2 = verb, arg3 = unit;
    char* argv[] = {arg0.data(), arg1.data(), arg2.data(), arg3.data(), nullptr};
    pid_t pid = 0;
    if (::posix_spawnp(&pid, "systemctl", nullptr, nullptr, argv, environ) != 0) return -1;
    int status = 0;
    if (::waitpid(pid, &status, 0) < 0) return -1;
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

remote::NetworkMessage HostServices::read_network() {
    remote::NetworkMessage out;
    char host[256] = {};
    if (::gethostname(host, sizeof(host) - 1) == 0) out.hostname = host;

    ifaddrs* addrs = nullptr;
    if (::getifaddrs(&addrs) != 0) return out;
    for (ifaddrs* a = addrs; a != nullptr; a = a->ifa_next) {
        if (!a->ifa_addr || a->ifa_addr->sa_family != AF_INET) continue;
        if (a->ifa_flags & IFF_LOOPBACK) continue;
        const auto* sin = reinterpret_cast<const sockaddr_in*>(a->ifa_addr);
        char text[INET_ADDRSTRLEN] = {};
        ::inet_ntop(AF_INET, &sin->sin_addr, text, sizeof(text));
        const uint32_t ip = ntohl(sin->sin_addr.s_addr);
        const bool link_local = (ip >> 16) == 0xA9FE;  // 169.254.0.0/16
        const std::string name = a->ifa_name;
        const bool wireless = std::filesystem::exists("/sys/class/net/" + name + "/wireless");
        if (link_local && out.cable_address.empty()) {
            out.cable_address = text;
        } else if (wireless && !out.wifi_address) {
            out.wifi_interface = name;
            out.wifi_address = text;
        }
    }
    ::freeifaddrs(addrs);
    return out;
}

std::optional<bool> HostServices::read_state_file() const {
    std::ifstream in(options_.state_file);
    std::string word;
    if (!(in >> word)) return std::nullopt;
    if (word == "on") return true;
    if (word == "off") return false;
    return std::nullopt;
}

void HostServices::write_state_file(bool enabled) const {
    std::error_code ec;
    std::filesystem::create_directories(options_.state_file.parent_path(), ec);
    std::ofstream out(options_.state_file, std::ios::trunc);
    out << (enabled ? "on" : "off") << "\n";
    if (!out) spdlog::warn("Could not save Wi-Fi access to {}", options_.state_file.string());
}

bool HostServices::apply_wifi_access(bool enabled) {
    const std::string verb = enabled ? "start" : "stop";
    const int rc = options_.run_systemctl(verb, options_.wifi_unit);
    if (rc != 0) {
        spdlog::warn(
            "systemctl {} {} failed (rc={}). Run install/install_dashboard_network.sh to install "
            "the unit and its polkit rule.",
            verb, options_.wifi_unit, rc);
        return false;
    }
    spdlog::info("Dashboard Wi-Fi access {}", enabled ? "on" : "off");
    return true;
}

void HostServices::publish_network() {
    remote::NetworkMessage message = read_network();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        message.wifi_access = wifi_access_;
        network_ = message;
    }
    if (status_bus_) status_bus_->publish(message);
}

void HostServices::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_) {
        if (requested_) {
            const bool enabled = *requested_;
            requested_.reset();
            lock.unlock();
            const bool ok = apply_wifi_access(enabled);
            if (ok) write_state_file(enabled);
            lock.lock();
            if (ok) wifi_access_ = enabled;
        }
        lock.unlock();
        publish_network();
        lock.lock();
        cv_.wait_for(lock, options_.refresh_period, [this] { return stop_ || requested_; });
    }
}

}  // namespace auto_battlebot
