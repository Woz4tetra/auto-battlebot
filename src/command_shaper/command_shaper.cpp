#include <algorithm>
#include <cmath>

#include "command_shaper/command_shaper_interface.hpp"

namespace auto_battlebot {

StickCommand saturate_stick(StickCommand stick, double saturation_limit) {
    const double yaw = std::clamp(stick.yaw, -1.0, 1.0);
    double headroom = 1.0;
    if (saturation_limit > 0.0) {
        headroom = std::max(0.0, std::clamp(saturation_limit, 0.0, 1.0) - std::abs(yaw));
    }
    return {.forward = std::clamp(stick.forward, -headroom, headroom),
            .lateral = stick.lateral,
            .yaw = yaw};
}

}  // namespace auto_battlebot
