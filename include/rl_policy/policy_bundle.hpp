#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "enums/action_decoder_kind.hpp"
#include "enums/network_runtime_kind.hpp"
#include "enums/policy_execution.hpp"
#include "enums/policy_kind.hpp"
#include "rl_policy/tensor.hpp"

namespace auto_battlebot::rl_policy {

inline constexpr int kPolicyBundleFormatVersion = 1;

class PolicyBundleError : public std::runtime_error {
   public:
    explicit PolicyBundleError(const std::string &message) : std::runtime_error(message) {}
};

struct NetworkSpec {
    std::string name;
    NetworkRuntimeKind runtime = NetworkRuntimeKind::RTNEURAL;
    std::filesystem::path file;
    std::vector<TensorSpec> inputs;
    std::vector<TensorSpec> outputs;
};

struct DecoderSpec {
    ActionDecoderKind kind = ActionDecoderKind::SQUASHED;
    std::vector<double> scale;
};

struct SelftestSpec {
    std::filesystem::path file;
    double tolerance = 1e-4;
};

struct NetworkBundle {
    DecoderSpec decoder;
    std::vector<NetworkSpec> networks;
    SelftestSpec selftest;

    const NetworkSpec &actor() const { return networks.front(); }
};

struct ConstantBundle {
    double linear = 0.0;
    double angular = 0.0;
};

struct PolicyBundle {
    std::filesystem::path directory;
    int format_version = 0;
    std::string run_id;
    std::string git_hash;
    PolicyKind policy_kind = PolicyKind::NETWORK;
    int obs_spec_version = 0;
    PolicyExecution execution = PolicyExecution::INLINE;
    int policy_period_control_ticks = 1;
    std::optional<std::map<std::string, std::string>> command_chain;
    std::variant<NetworkBundle, ConstantBundle> policy;
};

PolicyBundle load_policy_bundle(const std::filesystem::path &directory);

}  // namespace auto_battlebot::rl_policy
