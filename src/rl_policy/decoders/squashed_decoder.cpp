#include "rl_policy/decoders/squashed_decoder.hpp"

#include <cmath>
#include <utility>

namespace auto_battlebot::rl_policy {

SquashedDecoder::SquashedDecoder(std::vector<double> scale) : scale_(std::move(scale)) {}

void SquashedDecoder::decode(std::span<const float> raw, std::span<double> action) const {
    for (size_t i = 0; i < scale_.size(); ++i) {
        action[i] = scale_[i] * std::tanh(static_cast<double>(raw[i]));
    }
}

}  // namespace auto_battlebot::rl_policy
