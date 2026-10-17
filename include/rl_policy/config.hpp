#pragma once

#include <string>

#include "config/config_parser.hpp"

namespace auto_battlebot::rl_policy {

/**
 * The `[navigation.policy]` table. Config names only the bundle: the bundle's manifest decides
 * the policy kind, runtime, decoder, observation spec and execution, so a bundle cannot be paired
 * with the wrong decoder or observation layout.
 *
 *     [navigation.policy]
 *     bundle = "data/models/nav_policies/rl_nav_<run>"
 */
struct RlPolicyConfiguration {
    /** Bundle directory. A relative path resolves against the project root, like hazards_file. */
    std::string bundle;

    void parse(ConfigParser &parser) {
        bundle = parser.get_required_string("bundle");
        parser.validate_no_extra_fields();
    }
};

}  // namespace auto_battlebot::rl_policy
