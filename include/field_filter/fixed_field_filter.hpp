#pragma once

#include "field_filter/field_filter_interface.hpp"

namespace auto_battlebot {
inline void mark_plane_found(FieldDescriptionWithInlierPoints &description) {
    description.header.frame_id = FrameId::FIELD;
    description.child_frame_id = FrameId::FIELD;
}

class FixedFieldFilter : public FieldFilterInterface {
   public:
    FixedFieldFilter(double size_x, double size_y) : size_x_(size_x), size_y_(size_y) {}

    void reset([[maybe_unused]] TransformStamped tf_visodom_from_camera) override {}

    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(
        [[maybe_unused]] const CameraData &camera_data,
        [[maybe_unused]] const MaskStamped &field_mask) override {
        auto description = std::make_shared<FieldDescriptionWithInlierPoints>();
        mark_plane_found(*description);
        description->size.size = Size{size_x_, size_y_, 0.0};
        return description;
    }

    FieldDescription track_field(
        TransformStamped tf_visodom_from_camera,
        std::shared_ptr<FieldDescriptionWithInlierPoints> initial_description) override {
        FieldDescription description;
        description.child_frame_id = FrameId::FIELD;
        description.tf_camera_from_fieldcenter = tf_visodom_from_camera.transform;
        description.header = tf_visodom_from_camera.header;
        if (initial_description) {
            description.size = initial_description->size;
        } else {
            description.size.size = Size{size_x_, size_y_, 0.0};
        }
        return description;
    }

   private:
    double size_x_;
    double size_y_;
};

}  // namespace auto_battlebot
