#pragma once

#include <Eigen/Dense>
#include "header.hpp"
#include "enums/frame_id.hpp"

namespace auto_battlebot
{
    struct Transform
    {
        Eigen::MatrixXd tf = Eigen::Matrix4d::Identity();
    };

    struct TransformStamped
    {
        Header header;
        FrameId child_frame_id = FrameId::EMPTY;
        Transform transform;
    };

} // namespace auto_battlebot
