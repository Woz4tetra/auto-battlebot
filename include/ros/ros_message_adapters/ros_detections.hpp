#pragma once

#include <std_msgs/String.hxx>

#include "data_structures/detection.hpp"

namespace auto_battlebot {
namespace ros_adapters {
/**
 * @brief Convert raw detector output to a compact JSON std_msgs::String
 *
 * Payload:
 * {"stamp":<sec>,"w":<px>,"h":<px>,"dets":[{"x1","y1","x2","y2","conf","class_id","label"}]}
 */
std_msgs::String to_ros_detections(const DetectionsStamped &detections);

}  // namespace ros_adapters
}  // namespace auto_battlebot
