#pragma once

#include <cstdint>
#include <filesystem>
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

/** Throws PolicyBundleError when the file is missing, truncated, or not a version-1 self-test. */
SelftestPairs read_selftest(const std::filesystem::path &path);

struct SelftestResult {
    int pairs = 0;
    /** Pair and action index of the largest disagreement; -1 when there were no pairs. */
    int worst_pair = -1;
    int worst_index = -1;
    double worst_error = 0.0;
    double tolerance = 0.0;
    bool passed = false;

    std::string summary() const;
};

/**
 * Run every self-test observation through `runtime` and `decoder` on this box and compare with
 * the PyTorch actions. The runtime is reset before each pair, so a recurrent network sees each
 * observation from a zero state, and again at the end. Throws PolicyBundleError when the file's
 * sizes disagree with the runtime's input or the decoder's action size; a numeric disagreement
 * is reported in the result, not thrown.
 */
SelftestResult run_selftest(NetworkRuntimeInterface &runtime, const ActionDecoderInterface &decoder,
                            const SelftestSpec &spec);

}  // namespace auto_battlebot::rl_policy
