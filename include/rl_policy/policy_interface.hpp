#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "enums/policy_kind.hpp"
#include "rl_policy/policy_bundle.hpp"
#include "rl_policy/selftest.hpp"
#include "rl_policy/tensor.hpp"

namespace auto_battlebot::rl_policy {

/** One policy step's output, in the normalized [-1, 1] units every navigation emits. */
struct ActionCommand {
    double linear = 0.0;
    double angular = 0.0;
};

class PolicyInterface {
   public:
    virtual ~PolicyInterface() = default;

    virtual void load(const PolicyBundle &bundle) = 0;

    virtual std::vector<TensorSpec> input_specs() const = 0;

    virtual void reset() = 0;

    /** One step. `observation` holds every tensor input_specs() names. Never allocates. */
    virtual ActionCommand act(const TensorSet &observation) = 0;

    virtual std::optional<SelftestResult> selftest_result() const { return std::nullopt; }
};

std::unique_ptr<PolicyInterface> make_policy(PolicyKind kind);

}  // namespace auto_battlebot::rl_policy
