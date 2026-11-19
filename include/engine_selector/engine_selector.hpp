#pragma once

#include <optional>
#include <string>

#include "engine_selector/config.hpp"
#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
class EngineSelector {
   public:
    EngineSelector(EngineSelectorConfiguration config, std::string owner);

    std::optional<std::string> select(TrtEngine &engine);

   private:
    EngineSelectorConfiguration config_;
    std::string owner_;
};

std::string describe_cuda_device() noexcept;
}  // namespace auto_battlebot
