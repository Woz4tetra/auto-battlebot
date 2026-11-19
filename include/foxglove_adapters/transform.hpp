#pragma once

#include <foxglove/schemas.hpp>

#include "data_structures/transform.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::FrameTransform to_frame_transform(const TransformStamped &transform);

foxglove::schemas::FrameTransforms to_frame_transforms(const TransformStamped &transform);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
