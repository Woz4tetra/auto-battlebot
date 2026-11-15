#pragma once

#include <foxglove/schemas.hpp>
#include <string>

#include "data_structures/detection.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

/**
 * Raw detector output as the `/blob_detections` / `/keypoint_detections` JSON payload:
 * {"stamp":<sec>,"w":<px>,"h":<px>,"dets":[{"x1","y1","x2","y2","conf","class_id","label","kps"}]}
 */
std::string to_detections_json(const DetectionsStamped &detections);

foxglove::schemas::ImageAnnotations to_image_annotations(const DetectionsStamped &detections);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
