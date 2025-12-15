#pragma once

#include "field_filter/field_filter_interface.hpp"

namespace auto_battlebot
{
    class NoopFieldFilter : public FieldFilterInterface
    {
    public:
        void reset(TransformStamped tf_visodom_from_camera) override {}

        FieldDescription compute_field(CameraData camera_data, FieldMaskStamped field_mask) override
        {
            return FieldDescription{};
        }

        FieldDescription track_field(TransformStamped tf_visodom_from_camera, FieldDescription initial_description) override
        {
            return FieldDescription{};
        }
    };

} // namespace auto_battlebot
