#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <string>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "navigation/navigation_interface.hpp"
#include "ui/battery_options.hpp"

namespace auto_battlebot::ui_internal {

struct BatteryReading {
    double percent = 0.0;
    bool valid = false;
};

class IBatterySource {
   public:
    virtual ~IBatterySource() = default;
    virtual BatteryReading read() = 0;
};

class IDebugOverlayRenderer {
   public:
    virtual ~IDebugOverlayRenderer() = default;
    virtual void render(cv::Mat &image, const RobotDescriptionsStamped &robots,
                        const std::optional<NavigationPathSegment> &path,
                        const FieldDescription &field, const CameraInfo &camera_info) = 0;
};

std::unique_ptr<IBatterySource> make_battery_source(const std::string &normalized_source_name,
                                                    const BatteryOptions &options);
std::unique_ptr<IBatterySource> make_battery_source(const std::string &normalized_source_name);
std::unique_ptr<IDebugOverlayRenderer> make_debug_overlay_renderer();

}  // namespace auto_battlebot::ui_internal
