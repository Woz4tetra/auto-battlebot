#pragma once

#include <string>

#include "config/config_parser.hpp"

namespace auto_battlebot::rl_policy {

struct RlPolicyConfiguration {
    std::string bundle;

    void parse(ConfigParser &parser) {
        bundle = parser.get_required_string("bundle");
        parser.validate_no_extra_fields();
    }
};

}  // namespace auto_battlebot::rl_policy
