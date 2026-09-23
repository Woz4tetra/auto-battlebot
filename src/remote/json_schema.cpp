#include <algorithm>
#include <cctype>

#include "remote/json_message.hpp"

namespace auto_battlebot::remote {

namespace detail {

std::string lowercase(std::string_view text) {
    std::string out(text);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

void throw_type_error(std::string_view expected, const nlohmann::json& got) {
    throw std::invalid_argument("expected " + std::string(expected) + ", got " + got.type_name());
}

}  // namespace detail

nlohmann::json SchemaFields::object_schema() const {
    nlohmann::json out = {{"type", "object"}, {"properties", properties_}};
    out["required"] = required_;
    return out;
}

}  // namespace auto_battlebot::remote
