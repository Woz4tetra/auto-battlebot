#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace auto_battlebot::rl_test {

struct DenseLayer {
    std::vector<std::vector<double>> kernel_in_out;
    std::vector<double> bias;
    std::string activation;
};

inline std::filesystem::path bundles_dir_beside_source() {
    return std::filesystem::path(__FILE__).parent_path() / "bundles";
}

inline std::filesystem::path tiny_mlp_bundle() { return bundles_dir_beside_source() / "tiny_mlp"; }

inline std::filesystem::path constant_bundle() {
    return bundles_dir_beside_source() / "constant_forward";
}

inline std::filesystem::path fresh_dir(const std::string &name) {
    const char *tmp = std::getenv("TMPDIR");
    const std::filesystem::path dir =
        std::filesystem::path(tmp != nullptr ? tmp : "/tmp") / ("rl_policy_test_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

inline std::filesystem::path copy_bundle(const std::filesystem::path &source,
                                         const std::string &name) {
    const std::filesystem::path dir = fresh_dir(name);
    for (const auto &entry : std::filesystem::directory_iterator(source)) {
        std::filesystem::copy_file(entry.path(), dir / entry.path().filename());
    }
    return dir;
}

inline std::string read_text(const std::filesystem::path &path) {
    std::ifstream file(path);
    std::stringstream text;
    text << file.rdbuf();
    return text.str();
}

inline void write_text(const std::filesystem::path &path, const std::string &text) {
    std::ofstream file(path);
    file << text;
}

inline void replace_in_file(const std::filesystem::path &path, const std::string &from,
                            const std::string &to) {
    std::string text = read_text(path);
    const size_t at = text.find(from);
    if (at != std::string::npos) text.replace(at, from.size(), to);
    write_text(path, text);
}

inline std::string json_numbers(const std::vector<double> &values) {
    std::ostringstream text;
    text.precision(17);
    text << "[";
    for (size_t i = 0; i < values.size(); ++i) text << (i > 0 ? ", " : "") << values[i];
    text << "]";
    return text.str();
}

inline void write_dense_json(const std::filesystem::path &path, int input_size,
                             const std::vector<DenseLayer> &layers) {
    std::ostringstream json;
    json << R"({"in_shape": [null, null, )" << input_size << R"(], "layers": [)";
    for (size_t l = 0; l < layers.size(); ++l) {
        const DenseLayer &layer = layers[l];
        json << (l > 0 ? ", " : "") << R"({"type": "dense", "activation": ")" << layer.activation
             << R"(", "shape": [null, null, )" << layer.bias.size() << R"(], "weights": [[)";
        for (size_t i = 0; i < layer.kernel_in_out.size(); ++i) {
            json << (i > 0 ? ", " : "") << json_numbers(layer.kernel_in_out[i]);
        }
        json << "], " << json_numbers(layer.bias) << "]}";
    }
    json << "]}";
    write_text(path, json.str());
}

constexpr char kSelftestMagic[4] = {'R', 'L', 'S', 'T'};
constexpr uint32_t kSelftestVersion = 1;

struct SelftestHeader {
    uint32_t version = kSelftestVersion;
    uint32_t pair_count = 0;
    uint32_t obs_size = 0;
    uint32_t action_size = 0;
};

inline void write_selftest(const std::filesystem::path &path, uint32_t count, uint32_t obs_size,
                           uint32_t action_size, const std::vector<float> &observations,
                           const std::vector<float> &actions) {
    std::ofstream file(path, std::ios::binary);
    file.write(kSelftestMagic, sizeof(kSelftestMagic));
    const SelftestHeader header{
        .pair_count = count, .obs_size = obs_size, .action_size = action_size};
    file.write(reinterpret_cast<const char *>(&header), sizeof(header));
    file.write(reinterpret_cast<const char *>(observations.data()),
               static_cast<std::streamsize>(observations.size() * sizeof(float)));
    file.write(reinterpret_cast<const char *>(actions.data()),
               static_cast<std::streamsize>(actions.size() * sizeof(float)));
}

inline std::string network_manifest(int obs_spec_version, int obs_size,
                                    const std::string &input_name = "obs.vector",
                                    int policy_period_ticks = 5) {
    std::ostringstream text;
    text << "format_version = 1\n"
         << "run_id = \"test_bundle\"\n"
         << "git_hash = \"test\"\n"
         << "policy_kind = \"NETWORK\"\n"
         << "obs_spec_version = " << obs_spec_version << "\n"
         << "execution = \"INLINE\"\n"
         << "policy_period_ticks = " << policy_period_ticks << "\n\n"
         << "[decoder]\nkind = \"SQUASHED\"\nscale = [1, 1]\n\n"
         << "[[networks]]\nname = \"actor\"\nruntime = \"RTNEURAL\"\n"
         << "file = \"actor.rtneural.json\"\n"
         << "inputs = [{ name = \"" << input_name << "\", shape = [" << obs_size << "] }]\n"
         << "outputs = [{ name = \"action\", shape = [2] }]\n\n"
         << "[selftest]\nfile = \"selftest.bin\"\ntolerance = 0.0001\n";
    return text.str();
}

inline std::filesystem::path write_linear_bundle(const std::string &name, int obs_spec_version,
                                                 int obs_size, double gain = 0.5,
                                                 double last_angular_selftest_error = 0.0,
                                                 const std::string &input_name = "obs.vector",
                                                 int linear_obs_index = 0,
                                                 int angular_obs_index = 1) {
    const std::filesystem::path dir = fresh_dir(name);
    DenseLayer layer;
    layer.kernel_in_out.assign(static_cast<size_t>(obs_size), std::vector<double>(2, 0.0));
    layer.kernel_in_out[static_cast<size_t>(linear_obs_index)][0] = gain;
    layer.kernel_in_out[static_cast<size_t>(angular_obs_index)][1] = gain;
    layer.bias = {0.0, 0.0};
    write_dense_json(dir / "actor.rtneural.json", obs_size, {layer});

    const uint32_t pairs = 4;
    std::vector<float> observations(pairs * static_cast<size_t>(obs_size), 0.0f);
    std::vector<float> actions(pairs * 2, 0.0f);
    for (uint32_t p = 0; p < pairs; ++p) {
        const float a = 0.25f * static_cast<float>(p) - 0.4f;
        const float b = 0.6f - 0.3f * static_cast<float>(p);
        observations[p * obs_size + linear_obs_index] = a;
        observations[p * obs_size + angular_obs_index] = b;
        actions[p * 2 + 0] = static_cast<float>(std::tanh(gain * a));
        actions[p * 2 + 1] = static_cast<float>(std::tanh(gain * b));
    }
    actions[(pairs - 1) * 2 + 1] += static_cast<float>(last_angular_selftest_error);
    write_selftest(dir / "selftest.bin", pairs, static_cast<uint32_t>(obs_size), 2, observations,
                   actions);
    write_text(dir / "manifest.toml", network_manifest(obs_spec_version, obs_size, input_name));
    return dir;
}

}  // namespace auto_battlebot::rl_test
