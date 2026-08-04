#pragma once

#include <memory>

#include "engine_selector/engine_selector.hpp"

namespace auto_battlebot {

// Model constructors take an injected EngineSelector. Tests that only exercise
// construction or uninitialized behavior still have to supply one, so build it from
// whatever the config already carries.
template <typename ConfigT>
std::shared_ptr<EngineSelector> make_test_selector(const ConfigT &config) {
    return std::make_shared<EngineSelector>(config.engine, "test");
}

}  // namespace auto_battlebot
