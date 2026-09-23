#pragma once

#include <cstddef>
#include <optional>
#include <string_view>

#include "remote/protocol.hpp"

namespace auto_battlebot::remote {

/** Matches the topic against RemoteTopics::commands and parses the payload into that command's
 *  struct. Unknown topics warn with the valid list; bad JSON or missing fields warn with the
 *  schema name and the parse error. An empty payload reads as `{}`. Never throws. */
std::optional<RemoteCommand> decode_command(std::string_view topic, const std::byte* data,
                                            size_t len);

}  // namespace auto_battlebot::remote
