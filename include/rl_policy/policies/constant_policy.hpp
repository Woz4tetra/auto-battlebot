#pragma once

#include "rl_policy/policy_interface.hpp"

namespace auto_battlebot::rl_policy {

class ConstantPolicy : public PolicyInterface {
   public:
    void load(const PolicyBundle &bundle) override;
    std::vector<TensorSpec> input_specs() const override { return {}; }
    void reset() override {}
    ActionCommand act(const TensorSet &) override { return action_; }

   private:
    ActionCommand action_;
};

}  // namespace auto_battlebot::rl_policy
