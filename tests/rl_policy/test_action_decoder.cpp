#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>

#include "rl_policy/action_decoder_interface.hpp"

namespace auto_battlebot::rl_policy {
namespace {

std::unique_ptr<ActionDecoderInterface> squashed_decoder() {
    return make_action_decoder(DecoderSpec{ActionDecoderKind::SQUASHED, {1.0, 2.0}});
}

TEST(SquashedDecoderTest, ScalesTheTanhOfEachRawValue) {
    const auto decoder = squashed_decoder();
    const std::array<float, 2> raw = {0.5f, -1.0f};
    std::array<double, 2> action{};
    decoder->decode(raw, action);
    EXPECT_NEAR(action[0], std::tanh(0.5), 1e-6);
    EXPECT_NEAR(action[1], 2.0 * std::tanh(-1.0), 1e-6);
}

TEST(SquashedDecoderTest, ARawOrActionSpanOfTheWrongSizeThrows) {
    const auto decoder = squashed_decoder();
    ASSERT_EQ(decoder->action_size(), 2u);
    std::array<float, 3> long_raw{};
    std::array<float, 2> raw{};
    std::array<double, 2> action{};
    std::array<double, 1> short_action{};
    EXPECT_THROW(decoder->decode(long_raw, action), std::invalid_argument);
    EXPECT_THROW(decoder->decode(raw, short_action), std::invalid_argument);
}

}  // namespace
}  // namespace auto_battlebot::rl_policy
