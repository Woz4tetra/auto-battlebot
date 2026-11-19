#pragma once

#include <spdlog/spdlog.h>

#include <memory>

#include "field_filter/field_filter_interface.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
class CameraWorldFieldFilter : public FieldFilterInterface {
   public:
    void reset(TransformStamped tf_visodom_from_camera) override {
        tf_visodom_from_cameraworld_ = tf_visodom_from_camera;
        spdlog::info("{} reset with transform: {}", filter_name_for_log(),
                     transform_to_string(tf_visodom_from_camera));
    }

    FieldDescription track_field(
        TransformStamped tf_visodom_from_camera,
        std::shared_ptr<FieldDescriptionWithInlierPoints> initial_description) override {
        if (tf_visodom_from_cameraworld_.header.frame_id == FrameId::EMPTY) {
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
    virtual const char *filter_name_for_log() const = 0;

    TransformStamped tf_visodom_from_cameraworld_;
};
}  // namespace auto_battlebot
