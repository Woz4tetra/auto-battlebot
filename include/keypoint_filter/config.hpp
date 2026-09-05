#pragma once

#include <toml++/toml.h>

#include <string>
#include <vector>

namespace auto_battlebot {

/** How KeypointHeightGate measures elevation above the field, and whether it rejects on it.
 *
 *  The measurement that works is *local relief*, not absolute height: the upper percentile inside
 *  the detection's inscribed circle minus the floor immediately around it. Comparing a detection
 *  against its own surroundings cancels field-plane fit error, plane drift and depth bias, all of
 *  which otherwise smear the two populations together.
 *
 *  Measured on the 2026-08-29 MassD replay, separating the arena-logo detections (persistent
 *  field-frame clusters) from moving ones, elevation scores AUC 0.928 against moving opponent
 *  detections and 0.959 against our own robots, versus 0.903 for absolute height. */
struct KeypointHeightGateConfiguration {
    /** Reject groups outside the band. Measurement and annotation happen either way.
     *
     *  On at the default threshold, which removes 91.1% of arena-logo detections on the 2026-08-29
     *  MassD replay while keeping 89.8% of moving opponent detections and 91.1% of the frames that
     *  had any opponent detection at all. */
    bool reject_enable = true;
    /** Minimum elevation above the surrounding floor, in metres.
     *
     *  Swept end to end on the 2026-08-29 MassD replay, scoring arena-logo detections (persistent
     *  field-frame clusters) against moving ones (logo removed / moving kept):
     *    0.025 -> 71.0% / 95.2%
     *    0.030 -> 88.4% / 89.6%
     *    0.035 -> 91.1% / 89.8%   <- default, dominates 0.030 on both axes
     *    0.040 -> 94.2% / 85.6%
     *    0.045 -> 96.4% / 75.3%
     *  Past 0.035 the cost to real detections climbs steeply for a few more points of logo. */
    double min_elevation_meters = 0.035;
    /** Ceiling on absolute height above the field plane. Anything higher is not a robot on the
     *  floor: walls, the ceiling, people leaning over the arena. */
    double max_height_meters = 0.40;
    /** Percentile of the depths inside the detection circle taken as the detection's top.
     *
     *  Not the median. The camera views the arena about 23 degrees off horizontal, so even an
     *  inscribed circle holds floor alongside the robot and a median measures the floor. The
     *  question is "does anything in this detection stand up", which is an upper percentile. */
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

/** Suppression of robot-blob detections that hold the same field position.
 *
 *  Arena floor graphics are fixed in the field frame; robots are not. Measured on the 2026-08-29
 *  MassD replay, the two logo blobs sit a median 0.043 m and 0.049 m from their own centre, while
 *  a moving detection travels 0.127 m over 4 s even at the 10th percentile. That gap is what this
 *  gate reads.
 *
 *  It cannot tell a genuinely immobilised robot from a painted one. A robot that drives into a
 *  suppressed cluster raises its spread and switches suppression off there until it leaves, which
 *  is the safe direction to fail. */
struct StaticDetectionGateConfiguration {
    /** Off by default. Measured end to end on the 2026-08-29 MassD replay it is a worse trade than
     *  the height gate at every setting tried, and stacks badly on top of it: height alone removes
     *  91.1% of logo detections for 89.8% of moving ones kept, while adding this gate at
     *  static_radius 0.08 / dwell 4 s reaches 95.1% removed but keeps only 75.5%. Its own best
     *  standalone points are 57.5% removed / 89.3% kept (0.06 m, 4 s) and 34.7% / 95.9% (0.05 m,
     *  8 s). Enable it for an arena where the height gate underperforms, and re-measure. */
    bool enable = false;
    /** Radius within which a new detection is treated as the same object as a tracked cluster.
     *  Wide enough to hold a logo blob together (p99 deviation 0.174 m) without reaching across
     *  the arena. */
    double match_radius_meters = 0.20;
    /** A cluster counts as static while its *mean* deviation from its own running centre stays
     *  under this. Mean, not maximum: a single noisy frame must not disqualify a logo, and one
     *  frame of a robot passing through must not either. Logo blobs measure 0.043-0.049 m. */
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
