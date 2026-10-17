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

/**
 * Observation in, action out. RlPolicyNavigation sees only this interface, so a new policy kind
 * (a planner, a world model) is a class and an enum value, with no change to the navigation.
 */
class PolicyInterface {
   public:
    virtual ~PolicyInterface() = default;

    /** Build the policy from a parsed bundle and run its checks (network sizes, self-test).
     * Throws PolicyBundleError naming what failed. */
    virtual void load(const PolicyBundle &bundle) = 0;

    /** The observation tensors act() reads, by name and shape. Empty for a policy that ignores
     * the observation. */
    virtual std::vector<TensorSpec> input_specs() const = 0;

    /** Clear any memory of earlier steps: recurrent state, for a network that has it. */
    virtual void reset() = 0;

    /** One step. `observation` holds every tensor input_specs() names. Never allocates. */
    virtual ActionCommand act(const TensorSet &observation) = 0;

    /** The self-test load() ran, when the policy has one. */
    virtual std::optional<SelftestResult> selftest_result() const { return std::nullopt; }
};

/** The policy for `kind`, not yet loaded. */
std::unique_ptr<PolicyInterface> make_policy(PolicyKind kind);

}  // namespace auto_battlebot::rl_policy
