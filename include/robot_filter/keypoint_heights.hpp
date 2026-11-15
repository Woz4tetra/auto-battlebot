#pragma once

#include <unordered_map>

#include "enums/label.hpp"

namespace auto_battlebot {

struct KeypointHeights {
    double default_meters = 0.0;
    std::unordered_map<Label, double> per_label_meters;

    double height_for(Label label) const {
        const auto it = per_label_meters.find(label);
        return it == per_label_meters.end() ? default_meters : it->second;
    }
};

}  // namespace auto_battlebot
