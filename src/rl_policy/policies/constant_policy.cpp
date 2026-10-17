#include "rl_policy/policies/constant_policy.hpp"

namespace auto_battlebot::rl_policy {

void ConstantPolicy::load(const PolicyBundle &bundle) {
    if (bundle.policy_kind != PolicyKind::CONSTANT || !bundle.constant_action ||
        bundle.constant_action->size() != 2) {
        throw PolicyBundleError("policy bundle " + bundle.directory.string() +
                                ": ConstantPolicy needs a CONSTANT bundle with [constant] action "
                                "= [linear, angular]");
    }
    action_ = ActionCommand{(*bundle.constant_action)[0], (*bundle.constant_action)[1]};
}

}  // namespace auto_battlebot::rl_policy
