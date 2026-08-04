#include "engine_selector/engine_selector.hpp"

#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <utility>
#include <vector>

namespace auto_battlebot {

std::string describe_cuda_device() {
    int device = 0;
    cudaError_t err = cudaGetDevice(&device);
    if (err != cudaSuccess) {
        return std::string("no usable CUDA device (") + cudaGetErrorString(err) + ")";
    }

    cudaDeviceProp props{};
    err = cudaGetDeviceProperties(&props, device);
    if (err != cudaSuccess) {
        return std::string("CUDA device ") + std::to_string(device) +
               " (properties unavailable: " + cudaGetErrorString(err) + ")";
    }

    return std::string(props.name) + " (sm" + std::to_string(props.major) +
           std::to_string(props.minor) + ")";
}

EngineSelector::EngineSelector(EngineSelectorConfiguration config, std::string owner)
    : config_(std::move(config)), owner_(std::move(owner)) {}

std::optional<std::string> EngineSelector::select(TrtEngine &engine) {
    std::vector<std::string> rejections;

    for (const std::string &candidate : config_.candidates) {
        // Cheapest filter first. Engine files are synced per architecture, so on any
        // given machine most candidates simply are not present and cost nothing to skip.
        if (!std::filesystem::exists(candidate)) {
            rejections.push_back(candidate + " -> not present on this machine");
            continue;
        }

        // TensorRT reports a rejected candidate as an error. That is the misleading
        // message this class replaces, and it is an expected outcome here, so keep it
        // out of the error stream while probing.
        bool loaded = false;
        {
            TrtEngine::ScopedQuietLogging quiet;
            loaded = engine.load(candidate);
        }

        if (loaded) {
            spdlog::info("EngineSelector[{}]: loaded {} on {}", owner_, candidate,
                         describe_cuda_device());
            for (const std::string &rejection : rejections) {
                spdlog::debug("EngineSelector[{}]: skipped {}", owner_, rejection);
            }
            return candidate;
        }

        rejections.push_back(candidate +
                             " -> rejected by TensorRT (built for a different GPU "
                             "architecture or TensorRT version)");
    }

    spdlog::error("EngineSelector[{}]: no candidate engine loads on {}. Tried {} candidate(s):",
                  owner_, describe_cuda_device(), config_.candidates.size());
    for (const std::string &rejection : rejections) {
        spdlog::error("EngineSelector[{}]:   {}", owner_, rejection);
    }
    spdlog::error(
        "EngineSelector[{}]: build engines for this GPU, or add one that matches to the "
        "candidates list.",
        owner_);
    return std::nullopt;
}

}  // namespace auto_battlebot
