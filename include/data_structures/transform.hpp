#pragma once

#include <Eigen/Dense>
#include "header.hpp"

namespace auto_battlebot {

struct Transform {
    Eigen::MatrixXd tf;
};

struct TransformStamped {
    Header header;
    std::string child_frame_id;
    Transform transform;
};

}  // namespace auto_battlebot
