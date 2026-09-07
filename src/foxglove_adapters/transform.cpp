#include "foxglove_adapters/transform.hpp"

#include <Eigen/Geometry>

#include "foxglove_adapters/common.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::FrameTransform to_frame_transform(const TransformStamped &transform) {
    foxglove::schemas::FrameTransform out;
    out.timestamp = to_timestamp(transform.header.stamp);
    out.parent_frame_id = frame_id_string(transform.header.frame_id);
    out.child_frame_id = frame_id_string(transform.child_frame_id);

    const auto &tf = transform.transform.tf;
    if (tf.rows() >= 3 && tf.cols() >= 4) {
        out.translation = to_vector3(tf(0, 3), tf(1, 3), tf(2, 3));
        Eigen::Matrix3d rotation_matrix = tf.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation_matrix);
        out.rotation = to_quaternion(quat.w(), quat.x(), quat.y(), quat.z());
    } else {
        out.translation = to_vector3(0.0, 0.0, 0.0);
        out.rotation = to_quaternion(1.0, 0.0, 0.0, 0.0);
    }
    return out;
}

foxglove::schemas::FrameTransforms to_frame_transforms(const TransformStamped &transform) {
    foxglove::schemas::FrameTransforms out;
    out.transforms.push_back(to_frame_transform(transform));
    return out;
}

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
