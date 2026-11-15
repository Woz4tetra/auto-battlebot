#pragma once

#include <optional>
#include <string>

#include "engine_selector/config.hpp"
#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
class EngineSelector {
   public:
    EngineSelector(EngineSelectorConfiguration config, std::string owner);

    // Loads the first candidate that deserializes on this device into `engine` and
    // returns its path. Returns nullopt when no candidate works, after logging every
    // candidate and why it was rejected.
    //
    // Logs the outcome exactly once. spdlog fans out to stdout and the mcap /log
    // topic (see setup_logging in src/logging/logging.cpp), so one call covers both.
    std::optional<std::string> select(TrtEngine &engine);

   private:
    EngineSelectorConfiguration config_;
    std::string owner_;
};

// Describes CUDA device 0, e.g. "NVIDIA RTX A6000 (sm86)". Returns a diagnostic string
// instead of throwing when CUDA is unavailable, since this is only used in log lines.
std::string describe_cuda_device();
}  // namespace auto_battlebot
