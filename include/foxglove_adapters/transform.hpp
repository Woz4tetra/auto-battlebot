#pragma once

#include <foxglove/schemas.hpp>

#include "data_structures/transform.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

/** TransformStamped (4x4 parent-from-child matrix) to foxglove.FrameTransform. A matrix smaller
 *  than 3x4 yields the identity. */
foxglove::schemas::FrameTransform to_frame_transform(const TransformStamped &transform);

/** Wrap one transform in a foxglove.FrameTransforms message. */
foxglove::schemas::FrameTransforms to_frame_transforms(const TransformStamped &transform);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
