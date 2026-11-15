#pragma once

#include <foxglove/schemas.hpp>
#include <string>

#include "data_structures/detection.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

std::string to_detections_json(const DetectionsStamped &detections);

foxglove::schemas::ImageAnnotations to_image_annotations(const DetectionsStamped &detections);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
