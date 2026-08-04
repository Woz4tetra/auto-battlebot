#pragma once

#include <optional>
#include <string>

#include "engine_selector/config.hpp"
#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
/**
 * Picks the engine file that runs on this machine's GPU.
 *
 * A TensorRT engine is compiled ahead of time against one GPU architecture and one
 * TensorRT version, and it is not portable across either. Deserializing a mismatched
 * engine fails with "the engine plan file is not compatible with this version of
 * TensorRT", which names the version even when the real mismatch is compute capability,
 * and says nothing about what the machine is or what was tried.
 *
 * Selection asks TensorRT rather than parsing the sm tag out of the filename: the first
 * candidate that deserializes on this device is the correct one. Filenames stay purely
 * descriptive.
 *
 * One instance per model that loads engines; the model factories construct them.
 */
class EngineSelector {
   public:
    // `owner` names the model in log output, e.g. "YoloKeypointModel".
    EngineSelector(EngineSelectorConfiguration config, std::string owner);

    // Loads the first candidate that deserializes on this device into `engine` and
    // returns its path. Returns nullopt when no candidate works, after logging every
    // candidate and why it was rejected.
    //
    // Logs the outcome exactly once. spdlog fans out to stdout and the mcap /rosout
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
