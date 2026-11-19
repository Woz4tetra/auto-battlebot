#include "rl_policy/decoders/squashed_decoder.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace auto_battlebot::rl_policy {

SquashedDecoder::SquashedDecoder(std::vector<double> scale) : scale_(std::move(scale)) {}

void SquashedDecoder::decode(std::span<const float> raw, std::span<double> action) const {
    if (raw.size() != scale_.size() || action.size() != scale_.size()) {
        throw std::invalid_argument("SquashedDecoder expects " + std::to_string(scale_.size()) +
                                    " raw values and actions, got " + std::to_string(raw.size()) +
                                    " and " + std::to_string(action.size()));
    }
    for (size_t i = 0; i < scale_.size(); ++i) {
        action[i] = scale_[i] * std::tanh(static_cast<double>(raw[i]));
    }
}

}  // namespace auto_battlebot::rl_policy
