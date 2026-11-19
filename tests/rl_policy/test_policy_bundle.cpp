
#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "rl_policy/bundle_writer.hpp"
#include "rl_policy/policy_bundle.hpp"

namespace auto_battlebot::rl_policy {
namespace {

using rl_test::constant_bundle;
using rl_test::copy_bundle;
using rl_test::replace_in_file;
using rl_test::tiny_mlp_bundle;

std::string load_error(const std::filesystem::path &dir) {
    try {
        load_policy_bundle(dir);
    } catch (const PolicyBundleError &e) {
        return e.what();
    }
    return "";
}

std::filesystem::path broken_tiny_mlp(const std::string &name, const std::string &from,
                                      const std::string &to) {
    const auto dir = copy_bundle(tiny_mlp_bundle(), name);
    replace_in_file(dir / "manifest.toml", from, to);
    return dir;
}

TEST(PolicyBundleTest, LoadsTheExportedTinyMlp) {
    const PolicyBundle bundle = load_policy_bundle(tiny_mlp_bundle());
    EXPECT_EQ(bundle.format_version, 1);
    EXPECT_EQ(bundle.run_id, "tiny_mlp");
    EXPECT_EQ(bundle.policy_kind, PolicyKind::NETWORK);
    EXPECT_EQ(bundle.obs_spec_version, 0);
    EXPECT_EQ(bundle.execution, PolicyExecution::INLINE);
    EXPECT_EQ(bundle.policy_period_control_ticks, 5);
    const auto &network_bundle = std::get<NetworkBundle>(bundle.policy);
    EXPECT_EQ(network_bundle.decoder.kind, ActionDecoderKind::SQUASHED);
    EXPECT_EQ(network_bundle.decoder.scale, (std::vector<double>{1.0, 1.0}));
    ASSERT_EQ(network_bundle.networks.size(), 1u);
    const NetworkSpec &actor = network_bundle.actor();
    EXPECT_EQ(actor.name, "actor");
    EXPECT_EQ(actor.runtime, NetworkRuntimeKind::RTNEURAL);
    EXPECT_EQ(actor.file, tiny_mlp_bundle() / "actor.rtneural.json");
    ASSERT_EQ(actor.inputs.size(), 1u);
    EXPECT_EQ(actor.inputs.front(), (TensorSpec{"obs.vector", {4}}));
    ASSERT_EQ(actor.outputs.size(), 1u);
    EXPECT_EQ(actor.outputs.front(), (TensorSpec{"action", {2}}));
    EXPECT_DOUBLE_EQ(network_bundle.selftest.tolerance, 1e-4);
}

TEST(PolicyBundleTest, LoadsAConstantBundle) {
    const PolicyBundle bundle = load_policy_bundle(constant_bundle());
    EXPECT_EQ(bundle.policy_kind, PolicyKind::CONSTANT);
    EXPECT_EQ(bundle.obs_spec_version, 1);
    const auto &constant = std::get<ConstantBundle>(bundle.policy);
    EXPECT_DOUBLE_EQ(constant.linear, 0.3);
    EXPECT_DOUBLE_EQ(constant.angular, -0.1);
}

TEST(PolicyBundleTest, AWrongFormatVersionNamesBoth) {
    const auto dir = broken_tiny_mlp("format_version", "format_version = 1", "format_version = 2");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("format_version 2, this build reads 1"), std::string::npos) << error;
}

TEST(PolicyBundleTest, ANewerFormatFailsOnItsVersionNotOnAChangedKey) {
    const auto dir = broken_tiny_mlp("newer_format", "format_version = 1", "format_version = 2");
    replace_in_file(dir / "manifest.toml", "policy_period_ticks = 5", "policy_period_tick = 5");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("format_version 2, this build reads 1"), std::string::npos) << error;
}

TEST(PolicyBundleTest, AnUnknownPolicyKindListsTheValidValues) {
    const auto dir =
        broken_tiny_mlp("policy_kind", R"(policy_kind = "NETWORK")", R"(policy_kind = "PLANNER")");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("policy_kind 'PLANNER'"), std::string::npos) << error;
    EXPECT_NE(error.find("NETWORK, CONSTANT"), std::string::npos) << error;
}

TEST(PolicyBundleTest, AnUnknownRuntimeListsTheValidValues) {
    const auto dir =
        broken_tiny_mlp("runtime", R"(runtime = "RTNEURAL")", R"(runtime = "ONNXRUNTIME")");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("runtime 'ONNXRUNTIME'"), std::string::npos) << error;
    EXPECT_NE(error.find("valid values: RTNEURAL"), std::string::npos) << error;
}

TEST(PolicyBundleTest, AnUnknownDecoderOrExecutionFails) {
    const auto decoder =
        broken_tiny_mlp("decoder_kind", R"(kind = "SQUASHED")", R"(kind = "CLIPPED")");
    EXPECT_NE(load_error(decoder).find("decoder.kind 'CLIPPED'"), std::string::npos);
    const auto execution =
        broken_tiny_mlp("execution", R"(execution = "INLINE")", R"(execution = "WORKER")");
    EXPECT_NE(load_error(execution).find("execution 'WORKER'"), std::string::npos);
}

TEST(PolicyBundleTest, AMissingNetworkFileIsNamed) {
    const auto dir = copy_bundle(tiny_mlp_bundle(), "missing_network");
    std::filesystem::remove(dir / "actor.rtneural.json");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("networks[0].file names a missing file"), std::string::npos) << error;
}

TEST(PolicyBundleTest, AMissingSelftestFileIsNamed) {
    const auto dir = copy_bundle(tiny_mlp_bundle(), "missing_selftest");
    std::filesystem::remove(dir / "selftest.bin");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("selftest.file names a missing file"), std::string::npos) << error;
}

TEST(PolicyBundleTest, AMissingManifestIsNamed) {
    const auto dir = rl_test::fresh_dir("no_manifest");
    EXPECT_NE(load_error(dir).find("no manifest.toml"), std::string::npos);
}

TEST(PolicyBundleTest, ADecoderScaleOfTheWrongSizeIsAShapeMismatch) {
    const auto dir = broken_tiny_mlp("scale_size", "scale = [1, 1]", "scale = [1, 1, 1]");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("decoder.scale has 3 values but the actor's output action [2] has 2"),
              std::string::npos)
        << error;
}

TEST(PolicyBundleTest, AnActionThatIsNotLinearAndAngularFails) {
    const auto dir = broken_tiny_mlp("action_size", R"(name = "action", shape = [2])",
                                     R"(name = "action", shape = [3])");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("action [3] is not [linear, angular]"), std::string::npos) << error;
}

TEST(PolicyBundleTest, AMisspelledKeyFailsInsteadOfBeingIgnored) {
    const auto dir = broken_tiny_mlp("typo", "policy_period_ticks = 5", "policy_period_tick = 5");
    const std::string error = load_error(dir);
    EXPECT_NE(error.find("unknown key 'policy_period_tick'"), std::string::npos) << error;
}

TEST(PolicyBundleTest, AConstantBundleMustBeNormalizedLinearAndAngular) {
    const auto dir = copy_bundle(constant_bundle(), "constant_bad");
    replace_in_file(dir / "manifest.toml", "action = [0.3, -0.1]", "action = [1.5, 0.0]");
    EXPECT_NE(load_error(dir).find("[-1, 1]"), std::string::npos);
    replace_in_file(dir / "manifest.toml", "action = [1.5, 0.0]", "action = [0.5]");
    EXPECT_NE(load_error(dir).find("constant.action has 1 values"), std::string::npos);
}

}  // namespace
}  // namespace auto_battlebot::rl_policy
