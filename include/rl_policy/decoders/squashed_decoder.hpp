#pragma once

#include <vector>

#include "rl_policy/action_decoder_interface.hpp"

namespace auto_battlebot::rl_policy {

/** `action[i] = scale[i] * tanh(raw[i])`: the mean of a tanh-squashed Gaussian policy, which is
 * what deployment runs (plan section 2). */
class SquashedDecoder : public ActionDecoderInterface {
   public:
    explicit SquashedDecoder(std::vector<double> scale);

    size_t action_size() const override { return scale_.size(); }
    void decode(std::span<const float> raw, std::span<double> action) const override;

   private:
    std::vector<double> scale_;
};

}  // namespace auto_battlebot::rl_policy
