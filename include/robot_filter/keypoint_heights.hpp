#pragma once

#include <unordered_map>

#include "enums/label.hpp"

namespace auto_battlebot {

/**
 * @brief Height of tracked keypoints above the field plane, used to correct the radial bias of
 *        flat-plane projection. Heights are per robot label with a shared default for labels
 *        without an explicit entry. A height of 0.0 disables the correction.
 */
struct KeypointHeights {
    double default_meters = 0.0;
    std::unordered_map<Label, double> per_label_meters;

    double height_for(Label label) const {
        const auto it = per_label_meters.find(label);
        return it == per_label_meters.end() ? default_meters : it->second;
    }
};

}  // namespace auto_battlebot
