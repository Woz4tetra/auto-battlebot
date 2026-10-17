#include "rl_policy/selftest.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>

namespace auto_battlebot::rl_policy {
namespace {

static_assert(std::endian::native == std::endian::little,
              "selftest.bin is little-endian and read with memcpy");

constexpr size_t kHeaderBytes = 20;

uint32_t read_u32(const std::vector<char> &bytes, size_t offset) {
    uint32_t value = 0;
    std::memcpy(&value, bytes.data() + offset, sizeof(value));
    return value;
}

}  // namespace

SelftestPairs read_selftest(const std::filesystem::path &path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw PolicyBundleError("self-test " + path.string() + ": cannot open the file");
    const std::vector<char> bytes((std::istreambuf_iterator<char>(file)),
                                  std::istreambuf_iterator<char>());
    const auto fail = [&](const std::string &message) {
        throw PolicyBundleError("self-test " + path.string() + ": " + message);
    };
    if (bytes.size() < kHeaderBytes || std::memcmp(bytes.data(), "RLST", 4) != 0) {
        fail("not a self-test file (no RLST header)");
    }
    const uint32_t version = read_u32(bytes, 4);
    if (version != kSelftestVersion) {
        fail("version " + std::to_string(version) + ", this build reads " +
             std::to_string(kSelftestVersion));
    }
    SelftestPairs pairs;
    pairs.count = read_u32(bytes, 8);
    pairs.observation_size = read_u32(bytes, 12);
    pairs.action_size = read_u32(bytes, 16);
    const size_t observation_floats = size_t{pairs.count} * pairs.observation_size;
    const size_t action_floats = size_t{pairs.count} * pairs.action_size;
    const size_t expected = kHeaderBytes + 4 * (observation_floats + action_floats);
    if (bytes.size() != expected) {
        fail(std::to_string(bytes.size()) + " bytes; the header's " + std::to_string(pairs.count) +
             " pairs of " + std::to_string(pairs.observation_size) + " + " +
             std::to_string(pairs.action_size) + " floats need " + std::to_string(expected));
    }
    pairs.observations.resize(observation_floats);
    pairs.actions.resize(action_floats);
    std::memcpy(pairs.observations.data(), bytes.data() + kHeaderBytes, 4 * observation_floats);
    std::memcpy(pairs.actions.data(), bytes.data() + kHeaderBytes + 4 * observation_floats,
                4 * action_floats);
    return pairs;
}

std::string SelftestResult::summary() const {
    std::ostringstream text;
    text << pairs << " pairs, worst error " << worst_error;
    if (worst_pair >= 0) text << " at pair " << worst_pair << " action " << worst_index;
    text << " (tolerance " << tolerance << ")";
    return text.str();
}

SelftestResult run_selftest(NetworkRuntimeInterface &runtime, const ActionDecoderInterface &decoder,
                            const SelftestSpec &spec) {
    const SelftestPairs pairs = read_selftest(spec.file);
    TensorSet &inputs = runtime.inputs();
    const TensorSet &outputs = runtime.outputs();
    if (inputs.size() != 1 || outputs.size() != 1) {
        throw PolicyBundleError("self-test " + spec.file.string() +
                                ": needs a network with one input and one output tensor");
    }
    Tensor &input = inputs[0];
    if (pairs.observation_size != input.data.size()) {
        throw PolicyBundleError("self-test " + spec.file.string() + ": observations have " +
                                std::to_string(pairs.observation_size) +
                                " values; the network input is " + input.spec.to_string());
    }
    if (pairs.action_size != decoder.action_size()) {
        throw PolicyBundleError("self-test " + spec.file.string() + ": actions have " +
                                std::to_string(pairs.action_size) + " values; the decoder makes " +
                                std::to_string(decoder.action_size()));
    }

    SelftestResult result;
    result.tolerance = spec.tolerance;
    result.pairs = static_cast<int>(pairs.count);
    std::vector<double> action(pairs.action_size);
    for (uint32_t pair = 0; pair < pairs.count; ++pair) {
        runtime.reset();
        const float *observation =
            pairs.observations.data() + size_t{pair} * pairs.observation_size;
        std::copy(observation, observation + pairs.observation_size, input.data.begin());
        runtime.run();
        decoder.decode(outputs[0].data, action);
        for (uint32_t i = 0; i < pairs.action_size; ++i) {
            const double expected = pairs.actions[size_t{pair} * pairs.action_size + i];
            const double error = std::abs(action[i] - expected);
            // A NaN is the worst there is and stays the worst once found.
            const bool worse =
                result.worst_pair < 0 || (!std::isnan(result.worst_error) &&
                                          (std::isnan(error) || error > result.worst_error));
            if (worse) {
                result.worst_error = error;
                result.worst_pair = static_cast<int>(pair);
                result.worst_index = static_cast<int>(i);
            }
        }
    }
    runtime.reset();
    result.passed = result.worst_error <= spec.tolerance;
    return result;
}

}  // namespace auto_battlebot::rl_policy
