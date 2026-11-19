#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "rl_policy/bundle_writer.hpp"
#include "rl_policy/selftest.hpp"

namespace auto_battlebot::rl_policy {
namespace {

class RecordingRuntime : public NetworkRuntimeInterface {
   public:
    void load(const NetworkSpec &) override {}
    TensorSet &inputs() override { return inputs_; }
    const TensorSet &outputs() const override { return outputs_; }
    void run() override { events.push_back("run"); }
    void reset() override { events.push_back("reset"); }

    std::vector<std::string> events;

   private:
    TensorSet inputs_{{TensorSpec{"obs.vector", {2}}}};
    TensorSet outputs_{{TensorSpec{"action", {2}}}};
};

std::filesystem::path zero_selftest(const std::string &name, uint32_t pairs) {
    const auto dir = rl_test::fresh_dir(name);
    rl_test::write_selftest(dir / "selftest.bin", pairs, 2, 2, std::vector<float>(2 * pairs, 0.0f),
                            std::vector<float>(2 * pairs));
    return dir / "selftest.bin";
}

TEST(SelftestTest, TruncatedFileThrows) {
    const auto path = zero_selftest("selftest_truncated", 3);
    std::filesystem::resize_file(path, std::filesystem::file_size(path) - 4);
    EXPECT_THROW(read_selftest(path), PolicyBundleError);
}

TEST(SelftestTest, MissingFileThrows) {
    EXPECT_THROW(read_selftest(rl_test::fresh_dir("selftest_missing") / "selftest.bin"),
                 PolicyBundleError);
}

TEST(SelftestTest, WrongVersionThrows) {
    const auto path = zero_selftest("selftest_version", 1);
    std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
    file.seekp(4);
    const uint32_t version = kSelftestVersion + 1;
    file.write(reinterpret_cast<const char *>(&version), sizeof(version));
    file.close();
    EXPECT_THROW(read_selftest(path), PolicyBundleError);
}

TEST(SelftestTest, ResetsRuntimeBetweenPairs) {
    RecordingRuntime runtime;
    const auto decoder = make_action_decoder(DecoderSpec{ActionDecoderKind::SQUASHED, {1.0, 1.0}});
    const SelftestResult result =
        run_selftest(runtime, *decoder, SelftestSpec{zero_selftest("selftest_resets", 3), 1e-4});
    EXPECT_TRUE(result.passed);
    EXPECT_EQ(runtime.events,
              (std::vector<std::string>{"reset", "run", "reset", "run", "reset", "run", "reset"}));
}

TEST(SelftestTest, NaNIsReportedAsWorst) {
    const auto dir = rl_test::fresh_dir("selftest_nan");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    rl_test::write_selftest(dir / "selftest.bin", 3, 2, 2, std::vector<float>(6, 0.0f),
                            std::vector<float>{0.1f, 0.1f, nan, 0.0f, 0.5f, 0.5f});
    RecordingRuntime runtime;
    const auto decoder = make_action_decoder(DecoderSpec{ActionDecoderKind::SQUASHED, {1.0, 1.0}});
    const SelftestResult result =
        run_selftest(runtime, *decoder, SelftestSpec{dir / "selftest.bin", 1e-4});
    EXPECT_FALSE(result.passed);
    EXPECT_TRUE(std::isnan(result.worst_error));
    ASSERT_TRUE(result.worst_pair);
    EXPECT_EQ(*result.worst_pair, 1);
}

}  // namespace
}  // namespace auto_battlebot::rl_policy
