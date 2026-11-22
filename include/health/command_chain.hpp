#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "transmitter/config.hpp"

namespace auto_battlebot {

using CommandChain = std::map<std::string, std::string>;

std::optional<CommandChain> resolve_command_chain(const TransmitterConfiguration &transmitter);

std::string format_command_chain(const CommandChain &chain);

std::string command_chain_hash(const CommandChain &chain);

std::vector<std::string> command_chain_differences(const CommandChain &bundle,
                                                   const CommandChain &profile);

}  // namespace auto_battlebot
