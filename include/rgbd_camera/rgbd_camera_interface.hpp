#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    class RgbdCameraInterface
    {
    public:
        virtual ~RgbdCameraInterface() = default;
        virtual bool initialize() = 0;
        virtual bool update() = 0;
        virtual bool get(CameraData &data) = 0;
        virtual bool should_close() = 0;
    };

} // namespace auto_battlebot
