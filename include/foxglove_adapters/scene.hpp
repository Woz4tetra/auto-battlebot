#pragma once

#include <foxglove/schemas.hpp>

#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "navigation/navigation_interface.hpp"

// Field, hazard, robot and navigation geometry as foxglove.SceneUpdate messages. Entity ids
// follow the `<ns>/<id>` rule in docs/foxglove_recording_format.md so converted legacy
// recordings and live recordings look identical to every reader.

namespace auto_battlebot {
namespace foxglove_adapters {

/** The field border: entity `field/0`, a closed LINE_STRIP through the four corners in the
 *  field's own frame via tf_camera_from_fieldcenter. */
foxglove::schemas::SceneUpdate to_field_scene(const FieldDescriptionWithInlierPoints &field);

/** The inlier cloud the plane was fitted on, as float32 xyz with no color. Returns nullopt when
 *  the description carries no cloud. */
std::optional<foxglove::schemas::PointCloud> to_field_point_cloud(
    const FieldDescriptionWithInlierPoints &field);

/**
 * One ring per keep-out disc (`hazards/<i>`), drawn at the hazard's own radius in the camera
 * frame via tf_camera_from_fieldcenter. Static hazards are amber, tracked ones magenta, so a
 * replay can tell arena geometry from a live house-bot track at a glance.
 */
foxglove::schemas::SceneUpdate to_hazard_scene(const FieldDescription &field);

/** Robot bodies (`robot_bounds/<frame index>`), heading arrows, labels, keypoint spheres and the
 *  line joining keypoints. */
foxglove::schemas::SceneUpdate to_robot_scene(const RobotDescriptionsStamped &robots);

/** Pursuit line to target, target crosshair, velocity arrow, angular velocity arc, plus
 *  deletions for every absent namespace so stale visuals clear. */
foxglove::schemas::SceneUpdate to_navigation_scene(const NavigationVisualization &nav);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
