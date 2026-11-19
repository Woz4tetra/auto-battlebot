#pragma once

#include <memory>
#include <vector>

#include "rl_policy/action_decoder_interface.hpp"
#include "rl_policy/network_runtime_interface.hpp"
#include "rl_policy/policy_interface.hpp"

namespace auto_battlebot::rl_policy {

class NetworkPolicy : public PolicyInterface {
   public:
    void load(const PolicyBundle &bundle) override;
    std::vector<TensorSpec> input_specs() const override;
    void reset() override;
    ActionCommand act(const TensorSet &observation) override;
    std::optional<SelftestResult> selftest_result() const override { return selftest_; }

   private:
    std::unique_ptr<NetworkRuntimeInterface> runtime_;
    std::unique_ptr<ActionDecoderInterface> decoder_;
    std::vector<double> action_;
    std::optional<SelftestResult> selftest_;
};

}  // namespace auto_battlebot::rl_policy
