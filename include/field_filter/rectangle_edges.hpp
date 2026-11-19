#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <limits>
#include <vector>

namespace auto_battlebot {

inline size_t lowest_midpoint_edge_index(const std::vector<Eigen::Vector2f> &corners) {
    size_t lowest = 0;
    float lowest_midpoint_y = std::numeric_limits<float>::max();
    for (size_t i = 0; i < corners.size(); ++i) {
        const float midpoint_y = (corners[i].y() + corners[(i + 1) % corners.size()].y()) / 2.0f;
        if (midpoint_y < lowest_midpoint_y) {
            lowest_midpoint_y = midpoint_y;
            lowest = i;
        }
    }
    return lowest;
}

}  // namespace auto_battlebot
