#include "health/command_chain.hpp"

#include <espnow_protocol.h>
#include <spdlog/fmt/fmt.h>

#include <cstdint>
#include <set>

namespace auto_battlebot {
namespace {

std::string format_rate(const std::optional<double> &rate) {
    if (!rate || *rate <= 0.0) return "0";
    return fmt::format("{:g}", *rate);
}

}  // namespace

std::optional<CommandChain> resolve_command_chain(const TransmitterConfiguration &transmitter) {
    if (const auto *espnow = dynamic_cast<const EspNowTransmitterConfiguration *>(&transmitter)) {
        return CommandChain{
            {"transmitter", espnow->type},
            {"drive_processor", espnow->drive_processor},
            {"max_motor_rpm_per_s", format_rate(espnow->max_motor_rpm_per_s)},
            {"protocol_version", std::to_string(espnow_protocol::PROTOCOL_VERSION)},
        };
    }
    if (const auto *opentx = dynamic_cast<const OpenTxTransmitterConfiguration *>(&transmitter)) {
        return CommandChain{
            {"transmitter", opentx->type},
            {"drive_processor", opentx->drive_processor},
            {"max_motor_rpm_per_s", format_rate(opentx->max_motor_rpm_per_s)},
            {"protocol_version", "none"},
        };
    }
    return std::nullopt;
}

std::string format_command_chain(const CommandChain &chain) {
    std::string text;
    for (const auto &[key, value] : chain) text += key + "=" + value + "\n";
    return text;
}

std::string command_chain_hash(const CommandChain &chain) {
    uint64_t hash = 0xcbf29ce484222325ULL;
    for (unsigned char c : format_command_chain(chain)) {
        hash ^= c;
        hash *= 0x100000001b3ULL;
    }
    return fmt::format("{:016x}", hash);
}

std::vector<std::string> command_chain_differences(const CommandChain &bundle,
                                                   const CommandChain &profile) {
    std::set<std::string> keys;
    for (const auto &[key, value] : bundle) keys.insert(key);
    for (const auto &[key, value] : profile) keys.insert(key);
    std::vector<std::string> differences;
    for (const std::string &key : keys) {
        const auto in_bundle = bundle.find(key);
        const auto in_profile = profile.find(key);
        const std::string bundle_value =
            in_bundle == bundle.end() ? "(missing)" : in_bundle->second;
        const std::string profile_value =
            in_profile == profile.end() ? "(missing)" : in_profile->second;
        if (bundle_value != profile_value) {
            differences.push_back(key + ": bundle " + bundle_value + ", profile " + profile_value);
        }
    }
    return differences;
}

}  // namespace auto_battlebot
