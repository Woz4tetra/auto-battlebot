#pragma once

#include <foxglove/schemas.hpp>

#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "navigation/navigation_interface.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::SceneUpdate to_field_scene(const FieldDescriptionWithInlierPoints &field);

std::optional<foxglove::schemas::PointCloud> to_field_point_cloud(
    const FieldDescriptionWithInlierPoints &field);

foxglove::schemas::SceneUpdate to_hazard_scene(const FieldDescription &field);

foxglove::schemas::SceneUpdate to_robot_scene(const RobotDescriptionsStamped &robots);

foxglove::schemas::SceneUpdate to_navigation_scene(const NavigationVisualization &nav);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
