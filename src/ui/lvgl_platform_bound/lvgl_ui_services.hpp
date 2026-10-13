#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <string>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "navigation/navigation_interface.hpp"

namespace auto_battlebot::ui_internal {

class IDebugOverlayRenderer {
   public:
    virtual ~IDebugOverlayRenderer() = default;
    virtual void render(cv::Mat &image, const RobotDescriptionsStamped &robots,
                        const std::optional<NavigationPathSegment> &path,
                        const FieldDescription &field, const CameraInfo &camera_info) = 0;
};

std::unique_ptr<IDebugOverlayRenderer> make_debug_overlay_renderer();

}  // namespace auto_battlebot::ui_internal
