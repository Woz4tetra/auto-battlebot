#pragma once

#include <optional>

#include "data_structures/field.hpp"
#include "data_structures/header.hpp"
#include "data_structures/robot.hpp"
#include "data_structures/target_selection.hpp"
#include "data_structures/velocity.hpp"

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
    virtual VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field,
                                   const TargetSelection &target) = 0;

    /** Snapshot of the last navigation tick (header, robots, command, path) for publishing. */
    virtual const NavigationVisualization &get_last_visualization() const = 0;
};

}  // namespace auto_battlebot
