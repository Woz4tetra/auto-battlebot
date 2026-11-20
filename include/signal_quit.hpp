#pragma once

#include "quittable.hpp"

namespace auto_battlebot {

class ScopedSignalQuit {
   public:
    explicit ScopedSignalQuit(Quittable &target);
    ~ScopedSignalQuit();

    ScopedSignalQuit(const ScopedSignalQuit &) = delete;
    ScopedSignalQuit &operator=(const ScopedSignalQuit &) = delete;
};

}  // namespace auto_battlebot
