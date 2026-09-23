#pragma once

#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "remote/messages.hpp"

namespace auto_battlebot::remote {

/**
 * The field border projected into the camera image, as fractions of the image size, for the
 * dashboard to draw over any preview size. Pinhole on the rectified image, the same projection
 * the LVGL overlay uses. Each side is sampled so a border that passes behind the camera is cut
 * where it does instead of dropped whole; every returned polyline has at least two points.
 * Empty when the field or the intrinsics are missing.
 */
std::vector<std::vector<ImagePoint>> project_field_outline(const FieldDescription &field,
                                                           const CameraInfo &camera_info,
                                                           int samples_per_side = 16);

}  // namespace auto_battlebot::remote
