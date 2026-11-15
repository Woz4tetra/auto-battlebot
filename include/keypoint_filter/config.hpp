#pragma once

#include <toml++/toml.h>

#include <string>
#include <vector>

namespace auto_battlebot {

struct KeypointHeightGateConfiguration {
    bool reject_enable = true;
    double min_elevation_meters = 0.035;
    /** Ceiling on absolute height above the field plane. Anything higher is not a robot on the
     *  floor: walls, the ceiling, people leaning over the arena. */
    double max_height_meters = 0.40;
    double top_percentile = 0.90;
    /** Percentile of the surrounding ring taken as the local floor. A low percentile because the
     *  ring can still catch part of a robot, and the floor is the low part of that. */
    double floor_percentile = 0.25;
    /** Inner and outer radius of the floor reference ring, as multiples of the inscribed circle's
     *  radius. The gap exists so a robot spilling past its inscribed circle does not contaminate
     *  its own floor reference. */
    double ring_inner_scale = 1.4;
    double ring_outer_scale = 2.4;
    /** Half-width of the square depth sample window centred on each keypoint, in pixels. Feeds the
     *  per-keypoint height used for projection, not the gating decision. */
    int sample_radius_px = 5;
    /** Minimum valid depth samples, in the circle and in the ring separately, before a reading is
     *  trusted. Below this the gate abstains for that detection. */
    int min_valid_samples = 8;
    /** Cap on depth samples per detection. Larger boxes stride to stay under it, which bounds the
     *  per-frame cost regardless of how close a robot is. */
    int max_circle_samples = 1024;
};

struct StaticDetectionGateConfiguration {
    bool enable = false;
    double match_radius_meters = 0.20;
    double static_radius_meters = 0.080;
    /** How long a cluster must hold still before its detections are suppressed. */
    double min_dwell_seconds = 4.0;
    /** Observations required over that dwell, so a stale track cannot qualify on two hits. */
    int min_observations = 30;
    /** Observation count at which the running mean and deviation stop averaging and start
     *  tracking. Without this the statistics go numb after a few thousand frames and a robot
     *  parking on a logo could never move the verdict. About 2 s at 30 Hz. */
    int responsiveness_window = 60;
    /** Clusters unseen for this long are forgotten. */
    double forget_seconds = 2.0;
};

struct KeypointFilterConfiguration {
    KeypointHeightGateConfiguration height;
    StaticDetectionGateConfiguration static_gate;
};

KeypointFilterConfiguration load_keypoint_filter_from_toml(
    const toml::table &toml_data, std::vector<std::string> &parsed_sections);

}  // namespace auto_battlebot
