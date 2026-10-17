// NetworkPolicy: the self-test gate at load, act() through runtime and decoder, and reset().
// ConstantPolicy rides along: it is the other policy kind built now.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "rl_policy/bundle_writer.hpp"
#include "rl_policy/policies/constant_policy.hpp"
#include "rl_policy/policies/network_policy.hpp"
#include "rl_policy/runtimes/rtneural_runtime.hpp"

namespace auto_battlebot::rl_policy {
namespace {

using rl_test::tiny_mlp_bundle;

TensorSet observation_of(const std::vector<float> &values) {
    TensorSet set({TensorSpec{"obs.vector", {static_cast<int>(values.size())}}});
    set[0].data = values;
    return set;
}

TEST(NetworkPolicyTest, TheExportedBundlePassesItsSelftestOnThisBox) {
    NetworkPolicy policy;
    policy.load(load_policy_bundle(tiny_mlp_bundle()));
    const auto result = policy.selftest_result();
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->passed) << result->summary();
    EXPECT_EQ(result->pairs, 16);
    EXPECT_LT(result->worst_error, 1e-4);
    EXPECT_EQ(policy.input_specs(), (std::vector<TensorSpec>{TensorSpec{"obs.vector", {4}}}));
}

TEST(NetworkPolicyTest, ActReproducesThePyTorchActions) {
    NetworkPolicy policy;
    const PolicyBundle bundle = load_policy_bundle(tiny_mlp_bundle());
    policy.load(bundle);
    const SelftestPairs pairs = read_selftest(bundle.selftest->file);
    for (uint32_t p = 0; p < pairs.count; ++p) {
        const std::vector<float> obs(pairs.observations.begin() + p * 4,
                                     pairs.observations.begin() + (p + 1) * 4);
        const ActionCommand action = policy.act(observation_of(obs));
        EXPECT_NEAR(action.linear, pairs.actions[p * 2], 1e-4) << "pair " << p;
        EXPECT_NEAR(action.angular, pairs.actions[p * 2 + 1], 1e-4) << "pair " << p;
    }
}

TEST(NetworkPolicyTest, AFailedSelftestRefusesToLoadAndNamesTheWorstPair) {
    // The linear test bundle's last pair has its angular action off by 0.01.
    const auto dir = rl_test::write_linear_bundle("selftest_fail", 0, 4, 0.5, 0.01);
    NetworkPolicy policy;
    try {
        policy.load(load_policy_bundle(dir));
        FAIL() << "expected the self-test to fail";
    } catch (const PolicyBundleError &e) {
        const std::string message = e.what();
        EXPECT_NE(message.find("self-test failed"), std::string::npos) << message;
        EXPECT_NE(message.find("at pair 3 action 1"), std::string::npos) << message;
        EXPECT_NE(message.find("worst error 0.01"), std::string::npos) << message;
    }
}

TEST(NetworkPolicyTest, ASelftestOfTheWrongSizeIsNamed) {
    const auto dir = rl_test::write_linear_bundle("selftest_size", 0, 4);
    rl_test::write_selftest(dir / "selftest.bin", 1, 3, 2, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f});
    NetworkPolicy policy;
    try {
        policy.load(load_policy_bundle(dir));
        FAIL() << "expected a size mismatch";
    } catch (const PolicyBundleError &e) {
        EXPECT_NE(std::string(e.what()).find("observations have 3 values; the network input is "
                                             "obs.vector [4]"),
                  std::string::npos)
            << e.what();
    }
}

/** A recurrent actor: GRU(2 -> 2) then Dense(2 -> 2). Its output depends on the steps before,
 * which is what reset() has to clear. */
std::filesystem::path write_gru_bundle() {
    const auto dir = rl_test::fresh_dir("gru_bundle");
    rl_test::write_text(
        dir / "actor.rtneural.json",
        R"({"in_shape": [null, null, 2], "layers": [)"
        R"({"type": "gru", "activation": "", "shape": [null, null, 2], "weights": [)"
        R"([[0.5, -0.4, 0.3, 0.2, 0.6, -0.7], [-0.3, 0.8, 0.1, -0.5, 0.4, 0.9]],)"
        R"([[0.7, 0.2, -0.6, 0.4, -0.3, 0.5], [0.1, -0.9, 0.3, 0.6, 0.2, -0.4]],)"
        R"([[0.05, -0.05, 0.1, 0.0, 0.02, -0.03], [0.0, 0.04, -0.02, 0.01, 0.0, 0.06]]]},)"
        R"({"type": "dense", "activation": "", "shape": [null, null, 2], "weights": [)"
        R"([[1.2, -0.8], [0.6, 1.1]], [0.1, -0.2]]}]})");

    // Expected actions from RTNeural itself, from a zero state, as the self-test runs them.
    RtNeuralRuntime runtime;
    NetworkSpec spec;
    spec.name = "actor";
    spec.file = dir / "actor.rtneural.json";
    spec.inputs = {TensorSpec{"obs.vector", {2}}};
    spec.outputs = {TensorSpec{"action", {2}}};
    runtime.load(spec);
    const std::vector<float> observations = {0.8f, -0.3f, -0.5f, 0.9f};
    std::vector<float> actions;
    for (int p = 0; p < 2; ++p) {
        runtime.reset();
        runtime.inputs()[0].data = {observations[2 * p], observations[2 * p + 1]};
        runtime.run();
        for (float raw : runtime.outputs()[0].data) actions.push_back(std::tanh(raw));
    }
    rl_test::write_selftest(dir / "selftest.bin", 2, 2, 2, observations, actions);
    rl_test::write_text(dir / "manifest.toml", rl_test::network_manifest(0, 2));
    return dir;
}

TEST(NetworkPolicyTest, ResetClearsRecurrentState) {
    NetworkPolicy policy;
    policy.load(load_policy_bundle(write_gru_bundle()));
    const TensorSet observation = observation_of({0.8f, -0.3f});

    const ActionCommand first = policy.act(observation);
    const ActionCommand second = policy.act(observation);
    // Same input, different output: the GRU carried state from the first step.
    EXPECT_GT(std::abs(second.linear - first.linear) + std::abs(second.angular - first.angular),
              1e-3);

    policy.reset();
    const ActionCommand after_reset = policy.act(observation);
    EXPECT_FLOAT_EQ(after_reset.linear, first.linear);
    EXPECT_FLOAT_EQ(after_reset.angular, first.angular);
}

TEST(NetworkPolicyTest, LoadLeavesTheStateReset) {
    // The self-test runs pairs through the network; none of that may leak into the first act().
    NetworkPolicy policy;
    const auto dir = write_gru_bundle();
    policy.load(load_policy_bundle(dir));
    const ActionCommand first = policy.act(observation_of({0.8f, -0.3f}));
    const SelftestPairs pairs = read_selftest(dir / "selftest.bin");
    EXPECT_NEAR(first.linear, pairs.actions[0], 1e-6);
    EXPECT_NEAR(first.angular, pairs.actions[1], 1e-6);
}

TEST(ConstantPolicyTest, ReturnsTheManifestActionAndReadsNothing) {
    ConstantPolicy policy;
    policy.load(load_policy_bundle(rl_test::constant_bundle()));
    EXPECT_TRUE(policy.input_specs().empty());
    EXPECT_FALSE(policy.selftest_result().has_value());
    const ActionCommand action = policy.act(TensorSet{});
    EXPECT_DOUBLE_EQ(action.linear, 0.3);
    EXPECT_DOUBLE_EQ(action.angular, -0.1);
}

TEST(PolicyRegistryTest, MakesOnePolicyPerKind) {
    EXPECT_NE(dynamic_cast<NetworkPolicy *>(make_policy(PolicyKind::NETWORK).get()), nullptr);
    EXPECT_NE(dynamic_cast<ConstantPolicy *>(make_policy(PolicyKind::CONSTANT).get()), nullptr);
}

}  // namespace
}  // namespace auto_battlebot::rl_policy
