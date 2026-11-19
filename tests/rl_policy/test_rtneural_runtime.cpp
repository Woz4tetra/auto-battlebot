
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "rl_policy/allocation_counter.hpp"
#include "rl_policy/bundle_writer.hpp"
#include "rl_policy/runtimes/rtneural_runtime.hpp"

namespace auto_battlebot::rl_policy {
namespace {

using rl_test::DenseLayer;

NetworkSpec spec_for(const std::filesystem::path &file, int inputs, int outputs) {
    NetworkSpec spec;
    spec.name = "actor";
    spec.runtime = NetworkRuntimeKind::RTNEURAL;
    spec.file = file;
    spec.inputs = {TensorSpec{"obs.vector", {inputs}}};
    spec.outputs = {TensorSpec{"action", {outputs}}};
    return spec;
}

std::vector<DenseLayer> tiny_layers(const std::string &output_activation) {
    DenseLayer hidden;
    hidden.kernel_in_out = {
        {0.5, -1.0, 0.25, 0.0}, {1.0, 0.5, -0.5, 2.0}, {-0.25, 0.75, 1.0, -1.0}};
    hidden.bias = {0.1, -0.2, 0.0, 0.3};
    hidden.activation = "elu";
    DenseLayer output;
    output.kernel_in_out = {{1.0, -0.5}, {0.5, 0.25}, {-1.0, 1.0}, {0.2, -0.3}};
    output.bias = {0.05, -0.1};
    output.activation = output_activation;
    return {hidden, output};
}

std::vector<float> run_once(RtNeuralRuntime &runtime, const std::vector<float> &input) {
    std::ranges::copy(input, runtime.inputs()[0].values().begin());
    runtime.run();
    const auto output = runtime.outputs()[0].values();
    return std::vector<float>(output.begin(), output.end());
}

constexpr double kOutputPreActivation0 = -2.2575816785144247;
constexpr double kOutputPreActivation1 = 2.0712390705126515;

TEST(RtNeuralRuntimeTest, DenseEluTanhMatchesHandComputedValues) {
    const auto dir = rl_test::fresh_dir("rtneural_tanh");
    rl_test::write_dense_json(dir / "net.json", 3, tiny_layers("tanh"));
    RtNeuralRuntime runtime;
    runtime.load(spec_for(dir / "net.json", 3, 2));

    const auto out = run_once(runtime, {0.3f, -0.7f, 1.2f});
    ASSERT_EQ(out.size(), 2u);
    EXPECT_NEAR(out[0], std::tanh(kOutputPreActivation0), 1e-5);
    EXPECT_NEAR(out[1], std::tanh(kOutputPreActivation1), 1e-5);
}

TEST(RtNeuralRuntimeTest, AnEmptyActivationMeansNone) {
    const auto dir = rl_test::fresh_dir("rtneural_linear");
    rl_test::write_dense_json(dir / "net.json", 3, tiny_layers(""));
    RtNeuralRuntime runtime;
    runtime.load(spec_for(dir / "net.json", 3, 2));

    const auto out = run_once(runtime, {0.3f, -0.7f, 1.2f});
    EXPECT_NEAR(out[0], -2.2575816785144247, 1e-5);
    EXPECT_NEAR(out[1], 2.0712390705126515, 1e-5);
}

TEST(RtNeuralRuntimeTest, LoadsTheExportedTinyMlp) {
    RtNeuralRuntime runtime;
    runtime.load(spec_for(rl_test::tiny_mlp_bundle() / "actor.rtneural.json", 4, 2));
    EXPECT_EQ(runtime.inputs()[0].spec, (TensorSpec{"obs.vector", {4}}));
    EXPECT_EQ(runtime.outputs()[0].spec, (TensorSpec{"action", {2}}));
}

std::string load_error(const NetworkSpec &spec) {
    try {
        RtNeuralRuntime runtime;
        runtime.load(spec);
    } catch (const PolicyBundleError &e) {
        return e.what();
    }
    return "";
}

TEST(RtNeuralRuntimeTest, ModelSizesMustMatchTheManifest) {
    const auto dir = rl_test::fresh_dir("rtneural_sizes");
    rl_test::write_dense_json(dir / "net.json", 3, tiny_layers("tanh"));
    const std::string inputs = load_error(spec_for(dir / "net.json", 5, 2));
    EXPECT_NE(inputs.find("the model takes 3 inputs; the manifest says obs.vector [5]"),
              std::string::npos)
        << inputs;
    const std::string outputs = load_error(spec_for(dir / "net.json", 3, 4));
    EXPECT_NE(outputs.find("the model gives 2 outputs; the manifest says action [4]"),
              std::string::npos)
        << outputs;
}

TEST(RtNeuralRuntimeTest, AnActivationRtNeuralCannotBuildIsNamed) {
    // RTNeural's parser would add a null layer for it and crash on the first forward pass.
    const auto dir = rl_test::fresh_dir("rtneural_gelu");
    rl_test::write_dense_json(dir / "net.json", 3, tiny_layers("gelu"));
    const std::string error = load_error(spec_for(dir / "net.json", 3, 2));
    EXPECT_NE(error.find("layers[1] activation 'gelu'"), std::string::npos) << error;
}

TEST(RtNeuralRuntimeTest, ALayerTypeRtNeuralCannotBuildIsNamed) {
    const auto dir = rl_test::fresh_dir("rtneural_layer_type");
    rl_test::write_dense_json(dir / "net.json", 3, tiny_layers("tanh"));
    rl_test::replace_in_file(dir / "net.json", R"("type": "dense", "activation": "tanh")",
                             R"("type": "transformer", "activation": "tanh")");
    const std::string error = load_error(spec_for(dir / "net.json", 3, 2));
    EXPECT_NE(error.find("layers[1] type 'transformer'"), std::string::npos) << error;
}

TEST(RtNeuralRuntimeTest, ATransposedKernelIsNamed) {
    auto layers = tiny_layers("tanh");
    const std::vector<std::vector<double>> kOutInKernel = {{1.0, 0.5, -1.0, 0.2},
                                                           {-0.5, 0.25, 1.0, -0.3}};
    layers[1].kernel_in_out = kOutInKernel;
    const auto dir = rl_test::fresh_dir("rtneural_transposed");
    rl_test::write_dense_json(dir / "net.json", 3, layers);
    const std::string error = load_error(spec_for(dir / "net.json", 3, 2));
    EXPECT_NE(error.find("layers[1] Dense kernel has 2 rows; the layer's input has 4"),
              std::string::npos)
        << error;
}

TEST(RtNeuralRuntimeTest, AFileThatIsNotJsonIsNamed) {
    const auto dir = rl_test::fresh_dir("rtneural_garbage");
    rl_test::write_text(dir / "net.json", "not json");
    const std::string error = load_error(spec_for(dir / "net.json", 3, 2));
    EXPECT_NE(error.find("RTNeural could not load it"), std::string::npos) << error;
}

TEST(RtNeuralRuntimeTest, RunDoesNotAllocate) {
    const auto dir = rl_test::fresh_dir("rtneural_alloc");
    rl_test::write_dense_json(dir / "net.json", 3, tiny_layers("tanh"));
    RtNeuralRuntime runtime;
    runtime.load(spec_for(dir / "net.json", 3, 2));
    const std::vector<float> input = {0.3f, -0.7f, 1.2f};
    std::ranges::copy(input, runtime.inputs()[0].values().begin());

    std::vector<int> *probe = nullptr;
    ASSERT_GT(rl_test::allocations_during([&] { probe = new std::vector<int>(4); }), 0)
        << "the counter saw nothing, so a zero below proves nothing";
    delete probe;

    const long allocations = rl_test::allocations_during([&] {
        for (int i = 0; i < 100; ++i) runtime.run();
        runtime.reset();
    });
    EXPECT_EQ(allocations, 0);
}

TEST(RtNeuralRuntimeTest, FullSizeActorInferenceTime) {
    std::mt19937 rng(7);
    std::normal_distribution<double> weight(0.0, 0.08);
    const std::vector<int> sizes = {138, 256, 256, 128, 2};
    std::vector<DenseLayer> layers;
    for (size_t l = 0; l + 1 < sizes.size(); ++l) {
        DenseLayer layer;
        layer.kernel_in_out.assign(static_cast<size_t>(sizes[l]),
                                   std::vector<double>(static_cast<size_t>(sizes[l + 1])));
        for (auto &row : layer.kernel_in_out) {
            for (auto &w : row) w = weight(rng);
        }
        layer.bias.assign(static_cast<size_t>(sizes[l + 1]), 0.01);
        layer.activation = l + 2 < sizes.size() ? "elu" : "";
        layers.push_back(std::move(layer));
    }
    const auto dir = rl_test::fresh_dir("rtneural_full_size");
    rl_test::write_dense_json(dir / "net.json", 138, layers);
    RtNeuralRuntime runtime;
    runtime.load(spec_for(dir / "net.json", 138, 2));
    for (auto &value : runtime.inputs()[0].values()) value = static_cast<float>(weight(rng) * 5.0);

    constexpr int kWarmup = 200;
    constexpr int kRuns = 5000;
    for (int i = 0; i < kWarmup; ++i) runtime.run();
    std::vector<double> times_us;
    times_us.reserve(kRuns);
    for (int i = 0; i < kRuns; ++i) {
        const auto start = std::chrono::steady_clock::now();
        runtime.run();
        times_us.push_back(
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start)
                .count());
    }
    std::sort(times_us.begin(), times_us.end());
    const double median = times_us[kRuns / 2];
    const double p99 = times_us[kRuns * 99 / 100];
    std::cout << "[ RtNeural ] 138-256-256-128-2 ELU: median " << median << " us, p99 " << p99
              << " us over " << kRuns << " runs\n";
    RecordProperty("median_us", std::to_string(median));
    RecordProperty("p99_us", std::to_string(p99));
    EXPECT_TRUE(std::isfinite(runtime.outputs()[0].values()[0]));
    // A loose ceiling so a loaded dev box does not fail it; the printed number is the result.
    EXPECT_LT(median, 2000.0);
}

}  // namespace
}  // namespace auto_battlebot::rl_policy
