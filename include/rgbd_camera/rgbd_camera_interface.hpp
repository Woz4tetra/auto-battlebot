#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class RgbdCameraInterface {
   public:
    virtual ~RgbdCameraInterface() = default;
    virtual bool initialize() = 0;
    // Request cancellation of a blocking initialize() call (e.g. while zed_.open() is pending).
    // Safe to call from another thread. Default is a no-op for cameras that open immediately.
    virtual void cancel_initialize() {}
    virtual bool get(CameraData &data, bool get_depth = false) = 0;
    virtual bool should_close() = 0;
};

}  // namespace auto_battlebot
