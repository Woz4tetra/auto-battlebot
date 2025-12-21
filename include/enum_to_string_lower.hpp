#pragma once

#include <string>
#include <algorithm>
#include <magic_enum.hpp>

namespace auto_battlebot
{
    template <typename E>
    std::string enum_to_string_lower(E value)
    {
        std::string result{magic_enum::enum_name(value)};
        std::transform(result.begin(), result.end(), result.begin(),
                       [](unsigned char c)
                       { return std::tolower(c); });
        return result;
    }
} // namespace auto_battlebot
