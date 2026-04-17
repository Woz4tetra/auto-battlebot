#pragma once

#include <cmath>
#include <string>

#include "enum_to_string_lower.hpp"
#include "enums.hpp"
#include "enums/frame_id.hpp"

namespace auto_battlebot {
/**
 * @brief RGB color struct with normalized float values [0.0, 1.0]
 */
struct ColorRGBf {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;

    /**
     * @brief Get color as BGR cv::Scalar (0-255 range)
     */
    inline auto to_bgr_255() const -> std::tuple<float, float, float> {
        return {b * 255.0f, g * 255.0f, r * 255.0f};
    }

    /**
     * @brief Get color as RGB tuple (0-1 range)
     */
    inline auto to_rgb_1() const -> std::tuple<float, float, float> { return {r, g, b}; }
};

/**
 * @brief Generate a distinct color for an index using HSV color wheel
 * Uses golden angle distribution for visually distinct colors
 * @param index The index to generate a color for
 * @return ColorRGBf with normalized RGB values
 */
inline ColorRGBf get_color_for_index(int index) {
    // Use HSV color wheel for distinct colors
    float hue = std::fmod(index * 137.5f, 360.0f);  // Golden angle for good distribution
    float sat = 0.8f;
    float val = 0.9f;

    // HSV to RGB conversion
    float c = val * sat;
    float x = c * (1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
    float m = val - c;
    float r, g, b;

    if (hue < 60) {
        r = c;
        g = x;
        b = 0;
    } else if (hue < 120) {
        r = x;
        g = c;
        b = 0;
    } else if (hue < 180) {
        r = 0;
        g = c;
        b = x;
    } else if (hue < 240) {
        r = 0;
        g = x;
        b = c;
    } else if (hue < 300) {
        r = x;
        g = 0;
        b = c;
    } else {
        r = c;
        g = 0;
        b = x;
    }

    return ColorRGBf{r + m, g + m, b + m};
}

inline ColorRGBf get_color_for_index(FrameId frame_id) {
    int index = static_cast<int>(frame_id);
    return get_color_for_index(index);
}

inline ColorRGBf get_color_for_index(Label label) {
    int index = static_cast<int>(label);
    return get_color_for_index(index);
}

inline ColorRGBf get_color_for_index(Group group) {
    int index = static_cast<int>(group);
    return get_color_for_index(index);
}

/**
 * @brief Extract short name from label enum string (last part after underscore)
 * @param enum_name Full enum name string (e.g., "ROBOT_FRIENDLY_RED")
 * @return Short name (e.g., "RED")
 */
inline std::string get_short_name(const std::string &enum_name) {
    size_t last_underscore = enum_name.rfind('_');
    if (last_underscore != std::string::npos && last_underscore + 1 < enum_name.size()) {
        return enum_name.substr(last_underscore + 1);
    }
    // If no underscore or at end, return first 4 chars
    return enum_name.substr(0, std::min(size_t(4), enum_name.size()));
}

/**
 * @brief Extract short name from Label enum directly
 * @param label The label enum value
 * @return Short name (e.g., "RED")
 */
inline std::string get_short_name(Label label) {
    std::string full_name = enum_to_string_lower(label);
    size_t last_underscore = full_name.rfind('_');
    if (last_underscore != std::string::npos && last_underscore + 1 < full_name.length()) {
        return full_name.substr(last_underscore + 1);
    }
    return full_name;
}

}  // namespace auto_battlebot
