#pragma once

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "enums/action_decoder_kind.hpp"
#include "enums/network_runtime_kind.hpp"
#include "enums/policy_execution.hpp"
#include "enums/policy_kind.hpp"
#include "rl_policy/tensor.hpp"

namespace auto_battlebot::rl_policy {

/** The manifest format this loader reads. auto_battlebot/rl/export/bundle.py FORMAT_VERSION. */
inline constexpr int kPolicyBundleFormatVersion = 1;

/** A bundle that cannot be used. The message names the bundle and the mismatch. */
class PolicyBundleError : public std::runtime_error {
   public:
    explicit PolicyBundleError(const std::string &message) : std::runtime_error(message) {}
};

/** One `[[networks]]` entry. */
struct NetworkSpec {
    std::string name;
    NetworkRuntimeKind runtime = NetworkRuntimeKind::RTNEURAL;
    /** Absolute path to the network file. */
    std::filesystem::path file;
    std::vector<TensorSpec> inputs;
    std::vector<TensorSpec> outputs;
};

/** The `[decoder]` table. */
struct DecoderSpec {
    ActionDecoderKind kind = ActionDecoderKind::SQUASHED;
    /** Multiplies each decoded action dimension. */
    std::vector<double> scale;
};

/** The `[selftest]` table. */
struct SelftestSpec {
    /** Absolute path to selftest.bin. */
    std::filesystem::path file;
    double tolerance = 1e-4;
};

/**
 * A model bundle directory, parsed and checked (plan section 12):
 *
 *     manifest.toml         policy kind, observation spec, decoder, execution, networks
 *     actor.rtneural.json   the network (NETWORK bundles)
 *     selftest.bin          observation and decoded-action pairs from PyTorch (NETWORK bundles)
 *
 * NETWORK bundles are written by auto_battlebot/rl/export/bundle.py. A CONSTANT bundle is written
 * by hand for the bench and the lockstep sim before a trained bundle exists. It has no networks,
 * decoder or self-test, only the action every tick:
 *
 *     format_version = 1
 *     run_id = "constant_forward"
 *     policy_kind = "CONSTANT"
 *     obs_spec_version = 1
 *     execution = "INLINE"
 *     policy_period_ticks = 5
 *
 *     [constant]
 *     action = [0.3, 0.0]   # linear, angular, normalized [-1, 1]
 */
struct PolicyBundle {
    std::filesystem::path directory;
    int format_version = 0;
    std::string run_id;
    std::string git_hash;
    PolicyKind policy_kind = PolicyKind::NETWORK;
    int obs_spec_version = 0;
    PolicyExecution execution = PolicyExecution::INLINE;
    /** Control-loop ticks between policy steps. The loop runs at 250 Hz, so 5 is 50 Hz. */
    int policy_period_ticks = 1;
    /** NETWORK bundles only. */
    std::optional<DecoderSpec> decoder;
    /** NETWORK bundles only; the first is the actor. */
    std::vector<NetworkSpec> networks;
    /** NETWORK bundles only. */
    std::optional<SelftestSpec> selftest;
    /** CONSTANT bundles only: [linear, angular]. */
    std::optional<std::vector<double>> constant_action;
};

/**
 * Parse `<directory>/manifest.toml` and check everything that needs no network library: the
 * format version, every enum field against its known values, required fields and tables, that
 * the files it names exist, and that the decoder scale matches the actor's output size. Throws
 * PolicyBundleError naming the problem.
 */
PolicyBundle load_policy_bundle(const std::filesystem::path &directory);

}  // namespace auto_battlebot::rl_policy
