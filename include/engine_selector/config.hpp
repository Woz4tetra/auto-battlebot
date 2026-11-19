#pragma once

#include <string>
#include <vector>

#include "config/config_parser.hpp"

namespace auto_battlebot {
struct EngineSelectorConfiguration {
    std::vector<std::string> candidates;

    void parse(ConfigParser &parser, const std::string &field_name);
};
}  // namespace auto_battlebot
