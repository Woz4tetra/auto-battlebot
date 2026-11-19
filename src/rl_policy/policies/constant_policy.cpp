#include "rl_policy/policies/constant_policy.hpp"

namespace auto_battlebot::rl_policy {

void ConstantPolicy::load(const PolicyBundle &bundle) {
    const auto *constant = std::get_if<ConstantBundle>(&bundle.policy);
    if (constant == nullptr) {
        throw PolicyBundleError("policy bundle " + bundle.directory.string() +
                                ": ConstantPolicy needs a CONSTANT bundle with [constant] action "
                                "= [linear, angular]");
    }
    action_ = ActionCommand{constant->linear, constant->angular};
}

}  // namespace auto_battlebot::rl_policy
