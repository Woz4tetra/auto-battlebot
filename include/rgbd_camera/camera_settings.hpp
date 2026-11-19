#pragma once

#include <algorithm>

#include "enums/anti_banding.hpp"
#include "remote/json_message.hpp"

namespace auto_battlebot {

struct CameraSettings {
    bool auto_exposure = true;
    int exposure_us = 0;
    int analog_gain_mdb = 0;
    int digital_gain = 1;
    bool auto_white_balance = true;
    int white_balance_k = 4600;
    int saturation = 4;
    int gamma = 5;
    int sharpness = 4;
    AntiBanding anti_banding = AntiBanding::OFF;

    bool operator==(const CameraSettings &) const = default;

    AB_JSON_MESSAGE(CameraSettings, "auto_battlebot.CameraSettings", auto_exposure, exposure_us,
                    analog_gain_mdb, digital_gain, auto_white_balance, white_balance_k, saturation,
                    gamma, sharpness, anti_banding)
};

struct CameraSettingRange {
    bool supported = false;
    int min = 0;
    int max = 0;

    AB_JSON_MESSAGE(CameraSettingRange, "auto_battlebot.CameraSettingRange", supported, min, max)
};

inline CameraSettingRange exposure_range_capped_at_frame_period(CameraSettingRange range,
                                                                int camera_fps) {
    range.max = std::min(range.max, 1'000'000 / camera_fps);
    return range;
}

struct CameraSettingLimits {
    CameraSettingRange exposure_us;
    CameraSettingRange analog_gain_mdb;
    CameraSettingRange digital_gain;
    CameraSettingRange white_balance_k;
    CameraSettingRange saturation;
    CameraSettingRange gamma;
    CameraSettingRange sharpness;
    CameraSettingRange anti_banding;

    AB_JSON_MESSAGE(CameraSettingLimits, "auto_battlebot.CameraSettingLimits", exposure_us,
                    analog_gain_mdb, digital_gain, white_balance_k, saturation, gamma, sharpness,
                    anti_banding)
};

struct CameraSettingsState {
    CameraSettings settings;
    CameraSettingLimits limits;
};

}  // namespace auto_battlebot
