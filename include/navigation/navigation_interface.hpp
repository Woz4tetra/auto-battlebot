#pragma once

#include <optional>

#include "data_structures.hpp"

namespace auto_battlebot {
/** 2D path segment for debug visualization (field frame). */
struct NavigationPathSegment {
    double our_x = 0.0, our_y = 0.0;
    double target_x = 0.0, target_y = 0.0;
};

/** Bundled navigation state for visualization. */
struct NavigationVisualization {
    Header header;
    std::optional<NavigationPathSegment> path;
    VelocityCommand command{0.0, 0.0, 0.0};
    RobotDescriptionsStamped robots;
};

class NavigationInterface {
   public:
    virtual ~NavigationInterface() = default;
    virtual bool initialize() = 0;
    virtual VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field) = 0;
    /** Last pursuit path (our position to target). Empty if not available. */
    virtual std::optional<NavigationPathSegment> get_last_path() const { return std::nullopt; }
};

}  // namespace auto_battlebot
