#pragma once

#include "field_filter/field_filter_interface.hpp"

namespace auto_battlebot {
/**
 * @brief Field filter that reports a fixed, configured arena size.
 *
 * Used by the kinematic sim, where the field is known a-priori and no perception runs. This gives
 * navigation the wall bounds it needs without a RANSAC field fit.
 *
 * The camera-from-fieldcenter transform is taken straight from the sim, which places its camera in
 * the same frame it reports robot poses in. Leaving it identity is not harmless: the UI projects
 * robot markers and the field border through it, and an identity transform divides by a robot's
 * ~1 cm height instead of its metres of range, throwing every marker far outside the image.
 */
class FixedFieldFilter : public FieldFilterInterface {
   public:
    FixedFieldFilter(double size_x, double size_y) : size_x_(size_x), size_y_(size_y) {}

    void reset([[maybe_unused]] TransformStamped tf_visodom_from_camera) override {}

    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(
        [[maybe_unused]] const CameraData &camera_data,
        [[maybe_unused]] const MaskStamped &field_mask) override {
        auto description = std::make_shared<FieldDescriptionWithInlierPoints>();
        // Non-EMPTY frame_id signals a valid field to the Runner's plane-found check.
        description->header.frame_id = FrameId::FIELD;
        description->child_frame_id = FrameId::FIELD;
        description->size.size = Size{size_x_, size_y_, 0.0};
        return description;
    }

    FieldDescription track_field(
        TransformStamped tf_visodom_from_camera,
        std::shared_ptr<FieldDescriptionWithInlierPoints> initial_description) override {
        FieldDescription description;
        description.child_frame_id = FrameId::FIELD;
        // The sim's world origin is the field centre, so the camera pose it ships is already
        // camera-from-fieldcenter and needs no composition.
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
