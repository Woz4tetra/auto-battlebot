#pragma once

#include <memory>
#include <vector>

#include "enums/network_runtime_kind.hpp"
#include "rl_policy/policy_bundle.hpp"
#include "rl_policy/tensor.hpp"

namespace auto_battlebot::rl_policy {

/**
 * Runs one network of a policy bundle: named float tensors in, named float tensors out.
 *
 * load() reads the network file and allocates every buffer. After that, the caller writes
 * inputs(), calls run() and reads outputs(); run() never allocates, so it is safe on the control
 * thread. Recurrent state, if the network has any, lives inside the runtime and reset() zeros it.
 */
class NetworkRuntimeInterface {
   public:
    virtual ~NetworkRuntimeInterface() = default;

    virtual void load(const NetworkSpec &spec) = 0;

    virtual TensorSet &inputs() = 0;
    virtual const TensorSet &outputs() const = 0;

    virtual void run() = 0;

    virtual void reset() = 0;
};

std::unique_ptr<NetworkRuntimeInterface> make_network_runtime(NetworkRuntimeKind kind);

}  // namespace auto_battlebot::rl_policy
