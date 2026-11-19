#pragma once

#include <optional>
#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "remote/messages.hpp"

namespace auto_battlebot::remote {

std::optional<ImagePoint> project_field_point(const FieldDescription &field,
                                              const CameraInfo &camera_info, double x, double y);

std::optional<FieldPoint> unproject_image_point(const FieldDescription &field,
                                                const CameraInfo &camera_info,
                                                const ImagePoint &point);

std::vector<std::vector<ImagePoint>> project_field_outline(const FieldDescription &field,
                                                           const CameraInfo &camera_info,
                                                           int samples_per_side = 16);

std::vector<ImagePoint> project_camera_marker(const FieldDescription &field,
                                              const CameraInfo &camera_info);

}  // namespace auto_battlebot::remote
