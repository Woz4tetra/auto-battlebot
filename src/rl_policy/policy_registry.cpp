#include "rl_policy/policies/constant_policy.hpp"
#include "rl_policy/policies/network_policy.hpp"
#include "rl_policy/policy_interface.hpp"

namespace auto_battlebot::rl_policy {

std::unique_ptr<PolicyInterface> make_policy(PolicyKind kind) {
    switch (kind) {
        case PolicyKind::NETWORK:
            return std::make_unique<NetworkPolicy>();
        case PolicyKind::CONSTANT:
            return std::make_unique<ConstantPolicy>();
    }
    throw PolicyBundleError("no policy for this kind");
}

}  // namespace auto_battlebot::rl_policy
