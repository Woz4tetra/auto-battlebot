#pragma once

#include <memory>

#include "engine_selector/engine_selector.hpp"

namespace auto_battlebot {

template <typename ConfigT>
std::shared_ptr<EngineSelector> make_test_selector(const ConfigT &config) {
    return std::make_shared<EngineSelector>(config.engine, "test");
}

}  // namespace auto_battlebot
