#pragma once

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>

#include "data_structures.hpp"

namespace auto_battlebot {

/**
 * Everything RobotFilterInterface::update() needs for one tick, captured from a live run so it
 * can be replayed later without a camera, GPU models, or the ZED SDK. See
 * src/record_filter_inputs.cpp (writer) and src/replay_filter_inputs.cpp (reader).
 */
struct FilterInputRecord {
    int tick = 0;
    /** Position in the source SVO, or -1 if not available (e.g. a live camera). */
    int64_t svo_frame_index = -1;
    KeypointsStamped keypoints;
    KeypointsStamped robot_blob_keypoints;
    CameraInfo camera_info;
    FieldDescription field_description;
};

/** Appends one record as a single line of JSON (JSON Lines / .jsonl), so a recording can be
 * streamed to disk incrementally and a truncated file still parses every complete line. */
void write_filter_input_record(std::ostream &out, const FilterInputRecord &record);

/** Parses one line previously written by write_filter_input_record(). Returns std::nullopt for a
 * blank line (e.g. a trailing newline at EOF). Throws nlohmann::json::exception on malformed
 * JSON or a missing/mistyped field. */
std::optional<FilterInputRecord> parse_filter_input_record(const std::string &line);

}  // namespace auto_battlebot
