#pragma once

#include <string>

namespace auto_battlebot
{
    enum class Group
    {
        OURS,
        THEIRS
    };

    std::string group_to_string(Group group);
    Group string_to_group(const std::string &str);

} // namespace auto_battlebot
