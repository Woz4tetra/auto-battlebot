#include "rl_policy/policies/network_policy.hpp"

#include <algorithm>

namespace auto_battlebot::rl_policy {

void NetworkPolicy::load(const PolicyBundle &bundle) {
    const std::string where = "policy bundle " + bundle.directory.string() + ": ";
    const auto *network_bundle = std::get_if<NetworkBundle>(&bundle.policy);
    if (network_bundle == nullptr || network_bundle->networks.empty()) {
        throw PolicyBundleError(where +
                                "NetworkPolicy needs a NETWORK bundle with [decoder], "
                                "[[networks]] and [selftest]");
    }
    if (network_bundle->networks.size() != 1) {
        throw PolicyBundleError(where +
                                "NetworkPolicy runs one network, the actor; the bundle "
                                "lists " +
                                std::to_string(network_bundle->networks.size()));
    }

    auto runtime = make_network_runtime(network_bundle->actor().runtime);
    runtime->load(network_bundle->actor());
    auto decoder = make_action_decoder(network_bundle->decoder);
    if (decoder->action_size() != runtime->outputs()[0].values().size()) {
        throw PolicyBundleError(
            where + "the decoder makes " + std::to_string(decoder->action_size()) +
            " action values from the actor's " + runtime->outputs()[0].spec.to_string());
    }

    const SelftestResult result = run_selftest(*runtime, *decoder, network_bundle->selftest);
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
        if (source == nullptr || source->values().size() != input.values().size())
            return ActionCommand{};
        std::ranges::copy(source->values(), input.values().begin());
    }
    runtime_->run();
    decoder_->decode(runtime_->outputs()[0].values(), action_);
    return ActionCommand{action_[0], action_[1]};
}

}  // namespace auto_battlebot::rl_policy
