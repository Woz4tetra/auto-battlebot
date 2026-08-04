#pragma once

#include <string>
#include <vector>

#include "config/config_parser.hpp"

namespace auto_battlebot {
/**
 * Candidate engine files for one model, in preference order.
 *
 * Parses a nested table, so it can be embedded in any model configuration:
 *
 *   [keypoint_model.engine]
 *   candidates = [
 *       "data/models/pose_x86_64_sm89.engine",
 *       "data/models/pose_aarch64_sm87.engine",
 *   ]
 *
 * Order is a preference, not a claim. EngineSelector asks TensorRT which candidate
 * actually deserializes on this GPU instead of decoding the sm tag in the filename, so
 * listing engines for architectures this machine does not have is expected and cheap.
 */
struct EngineSelectorConfiguration {
    std::vector<std::string> candidates;

    void parse(ConfigParser &parser, const std::string &field_name);
};
}  // namespace auto_battlebot
