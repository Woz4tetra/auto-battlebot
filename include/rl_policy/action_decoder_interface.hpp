#pragma once

#include <memory>
#include <span>

#include "rl_policy/policy_bundle.hpp"

namespace auto_battlebot::rl_policy {

class ActionDecoderInterface {
   public:
    virtual ~ActionDecoderInterface() = default;

    virtual size_t action_size() const = 0;

    /** `raw` and `action` are both action_size() long. Never allocates. */
    virtual void decode(std::span<const float> raw, std::span<double> action) const = 0;
};

std::unique_ptr<ActionDecoderInterface> make_action_decoder(const DecoderSpec &spec);

}  // namespace auto_battlebot::rl_policy
