#pragma once

#include <cstddef>
#include <optional>
#include <string_view>

#include "remote/protocol.hpp"

namespace auto_battlebot::remote {

std::optional<RemoteCommand> decode_command(std::string_view topic, const std::byte* data,
                                            size_t len);

}  // namespace auto_battlebot::remote
