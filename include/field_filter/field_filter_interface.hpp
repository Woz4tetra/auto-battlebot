#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    class FieldFilterInterface
    {
    public:
        virtual ~FieldFilterInterface() = default;
        virtual void reset(TransformStamped tf_visodom_from_camera) = 0;
        virtual FieldDescription compute_field(CameraData camera_data, FieldMaskStamped field_mask) = 0;
        virtual FieldDescription track_field(TransformStamped tf_visodom_from_camera, FieldDescription initial_description) = 0;
    };

} // namespace auto_battlebot
