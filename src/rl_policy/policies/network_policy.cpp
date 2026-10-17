#include "rl_policy/policies/network_policy.hpp"

#include <algorithm>

namespace auto_battlebot::rl_policy {

void NetworkPolicy::load(const PolicyBundle &bundle) {
    const std::string where = "policy bundle " + bundle.directory.string() + ": ";
    if (bundle.policy_kind != PolicyKind::NETWORK || !bundle.decoder || !bundle.selftest ||
        bundle.networks.empty()) {
        throw PolicyBundleError(where +
                                "NetworkPolicy needs a NETWORK bundle with [decoder], "
                                "[[networks]] and [selftest]");
    }
    if (bundle.networks.size() != 1) {
        throw PolicyBundleError(where +
                                "NetworkPolicy runs one network, the actor; the bundle "
                                "lists " +
                                std::to_string(bundle.networks.size()));
    }

    auto runtime = make_network_runtime(bundle.networks.front().runtime);
    runtime->load(bundle.networks.front());
    auto decoder = make_action_decoder(*bundle.decoder);
    if (decoder->action_size() != runtime->outputs()[0].data.size()) {
        throw PolicyBundleError(
            where + "the decoder makes " + std::to_string(decoder->action_size()) +
            " action values from the actor's " + runtime->outputs()[0].spec.to_string());
    }

    const SelftestResult result = run_selftest(*runtime, *decoder, *bundle.selftest);
    if (!result.passed) {
        throw PolicyBundleError(where + "self-test failed on this box: " + result.summary());
    }

    runtime_ = std::move(runtime);
    decoder_ = std::move(decoder);
    action_.assign(decoder_->action_size(), 0.0);
    selftest_ = result;
    reset();
}

std::vector<TensorSpec> NetworkPolicy::input_specs() const {
    if (!runtime_) return {};
    return runtime_->inputs().specs();
}

void NetworkPolicy::reset() {
    if (runtime_) runtime_->reset();
}

ActionCommand NetworkPolicy::act(const TensorSet &observation) {
    TensorSet &inputs = runtime_->inputs();
    for (size_t i = 0; i < inputs.size(); ++i) {
        Tensor &input = inputs[i];
        const Tensor *source = observation.find(input.spec.name);
        // RlPolicyNavigation checks names and shapes at initialize(); this only guards a caller
        // that skipped that check.
        if (source == nullptr || source->data.size() != input.data.size()) return ActionCommand{};
        std::copy(source->data.begin(), source->data.end(), input.data.begin());
    }
    runtime_->run();
    decoder_->decode(runtime_->outputs()[0].data, action_);
    return ActionCommand{action_[0], action_[1]};
}

}  // namespace auto_battlebot::rl_policy
