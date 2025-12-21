#pragma once

#include "field_filter/field_filter_interface.hpp"

namespace auto_battlebot
{
    class PointCloudFieldFilter : public FieldFilterInterface
    {
    public:
        PointCloudFieldFilter();
        ~PointCloudFieldFilter() override = default;

        void reset(TransformStamped tf_visodom_from_camera) override;
        FieldDescription compute_field(CameraData camera_data, FieldMaskStamped field_mask) override;
        FieldDescription track_field(TransformStamped tf_visodom_from_camera, FieldDescription initial_description) override;

    private:
        // Private member variables and helper methods will go here
    };

} // namespace auto_battlebot
