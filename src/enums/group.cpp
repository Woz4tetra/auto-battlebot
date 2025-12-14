#include "enums/group.hpp"
#include <stdexcept>

namespace auto_battlebot
{
    std::string group_to_string(Group group)
    {
        switch (group)
        {
        case Group::OURS:
            return "OURS";
        case Group::THEIRS:
            return "THEIRS";
        default:
            throw std::invalid_argument("Unknown Group");
        }
    }

    Group string_to_group(const std::string &str)
    {
        if (str == "OURS")
            return Group::OURS;
        if (str == "THEIRS")
            return Group::THEIRS;
        throw std::invalid_argument("Unknown Group string: " + str);
    }

} // namespace auto_battlebot
