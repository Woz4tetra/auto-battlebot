#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "rl_policy/action_decoder_interface.hpp"
#include "rl_policy/network_runtime_interface.hpp"
#include "rl_policy/policy_bundle.hpp"

namespace auto_battlebot::rl_policy {

struct SelftestPairs {
    uint32_t count = 0;
    uint32_t observation_size = 0;
    uint32_t action_size = 0;
    std::vector<float> observations;
    std::vector<float> actions;
};

inline constexpr uint32_t kSelftestVersion = 1;

SelftestPairs read_selftest(const std::filesystem::path &path);

struct SelftestResult {
    int pairs = 0;
    std::optional<int> worst_pair;
    std::optional<int> worst_index;
    double worst_error = 0.0;
    double tolerance = 0.0;
    bool passed = false;

    std::string summary() const;
};

SelftestResult run_selftest(NetworkRuntimeInterface &runtime, const ActionDecoderInterface &decoder,
                            const SelftestSpec &spec);

}  // namespace auto_battlebot::rl_policy
