#pragma once

#include <spdlog/spdlog.h>

#include <memory>

#include "field_filter/field_filter_interface.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
/**
 * @brief Base for filters that fit the field once and then rebase it as the camera moves.
 *
 * `compute_field` runs at init and answers in the camera frame of that instant, which the filter
 * pins as camera-world. Every later frame rebases through the camera's motion since. Keeping one
 * definition of that rebase matters: each field re-init redefines field -> camera_world, and when
 * two filters carried their own copy the sparse mask topics let Foxglove project a stale mask
 * through the new camera pose. `scripts/fix_field_mask_frames.py` exists to repair recordings
 * that hit it.
 *
 * On a clamped RGB camera `tf_visodom_from_camera` is identity and the rebase is a no-op, which
 * is the point: nothing to drift.
 */
class CameraWorldFieldFilter : public FieldFilterInterface {
   public:
    void reset(TransformStamped tf_visodom_from_camera) override {
        tf_visodom_from_cameraworld_ = tf_visodom_from_camera;
        spdlog::info("{} reset with transform: {}", name(),
                     transform_to_string(tf_visodom_from_camera));
    }

    FieldDescription track_field(
        TransformStamped tf_visodom_from_camera,
        std::shared_ptr<FieldDescriptionWithInlierPoints> initial_description) override {
        if (tf_visodom_from_cameraworld_.header.frame_id == FrameId::EMPTY) {
            // filter isn't initialized
            return FieldDescription{};
        }
        Eigen::MatrixXd tf_cameraworld_from_camera =
            tf_visodom_from_cameraworld_.transform.tf.inverse() *
            tf_visodom_from_camera.transform.tf;
        FieldDescription next_field_description{};
        next_field_description.header.stamp = tf_visodom_from_camera.header.stamp;
        next_field_description.header.frame_id = FrameId::CAMERA;
        next_field_description.child_frame_id = initial_description->child_frame_id;
        next_field_description.size = initial_description->size;
        Transform tf_cameraworld_from_fieldcenter = initial_description->tf_camera_from_fieldcenter;
        next_field_description.tf_camera_from_fieldcenter =
            Transform{tf_cameraworld_from_camera.inverse() * tf_cameraworld_from_fieldcenter.tf};
        return next_field_description;
    }

   protected:
    /** For the log line in reset, so the message names the concrete filter. */
    virtual const char *name() const = 0;

    TransformStamped tf_visodom_from_cameraworld_;
};
}  // namespace auto_battlebot
