#pragma once

#include "field_filter/field_filter_interface.hpp"

namespace auto_battlebot
{
    class NoopFieldFilter : public FieldFilterInterface
    {
    public:
        void reset([[maybe_unused]] TransformStamped tf_visodom_from_camera) override {}

        std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field([[maybe_unused]] const CameraData &camera_data, [[maybe_unused]] const FieldMaskStamped &field_mask) override
        {
            return std::make_shared<FieldDescriptionWithInlierPoints>(FieldDescriptionWithInlierPoints{});
        }

        FieldDescription track_field([[maybe_unused]] TransformStamped tf_visodom_from_camera, [[maybe_unused]] std::shared_ptr<FieldDescriptionWithInlierPoints> initial_description) override
        {
            return FieldDescription{};
        }
    };

} // namespace auto_battlebot
