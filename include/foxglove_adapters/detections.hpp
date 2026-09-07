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

/**
 * The same detections drawn on the Image panel: a LINE_LOOP box per detection, a circle per
 * keypoint, a label with confidence at the box top-left. Live only, never recorded; the JSON
 * channel is the structured truth.
 */
foxglove::schemas::ImageAnnotations to_image_annotations(const DetectionsStamped &detections);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
