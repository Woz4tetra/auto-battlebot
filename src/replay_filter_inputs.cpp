#include <spdlog/spdlog.h>

#include <fstream>
#include <magic_enum.hpp>

#include "config/config.hpp"
#include "directories.hpp"
#include "robot_filter/filter_input_record.hpp"

// Replays a .jsonl file written by record_filter_inputs.cpp through robot_filter->update(), with
// no camera, GPU models, or ZED SDK involved -- only the config's [robot_filter] (and
// [runner].default_opponent_count / [clock]) sections matter here, so this stays fast and needs
// no GPU at all.
//
// Usage: ./replay_filter_inputs <input .jsonl path> [config path or profile name]
int main(int argc, char** argv) {
    using namespace auto_battlebot;

    if (argc < 2) {
        spdlog::error("Usage: {} <input .jsonl path> [config path or profile name]", argv[0]);
        return 1;
    }
    std::filesystem::path input_path(argv[1]);
    std::filesystem::path config_path =
        (argc > 2) ? normalize_config_path(argv[2])
                   : get_config_dir() / "playback/mrs_buff_mk3_playback.toml";

    spdlog::info("Loading config: {}", config_path.string());
    ClassConfiguration class_config = load_classes_from_config(config_path);

    auto clock = make_clock(*class_config.clock);
    auto robot_filter = make_robot_filter(*class_config.robot_filter, clock);

    std::ifstream in(input_path);
    if (!in) {
        spdlog::error("Failed to open {}", input_path.string());
        return 1;
    }

    bool initialized = false;
    std::string line;
    int record_count = 0;
    while (std::getline(in, line)) {
        std::optional<FilterInputRecord> record = parse_filter_input_record(line);
        if (!record) continue;  // blank line (e.g. trailing newline at EOF)

        if (!initialized) {
            robot_filter->initialize(class_config.runner.default_opponent_count);
            initialized = true;
        }

        if (clock) clock->set(record->keypoints.header.stamp);

        // record_filter_inputs.cpp doesn't capture command feedback (the caller's spec is just
        // keypoints/robot_blob_keypoints/camera_info/field_description), so this always replays
        // with no commanded velocity, same as scratch.cpp's live loop.
        RobotDescriptionsStamped robots =
            robot_filter->update(record->keypoints, record->field_description, record->camera_info,
                                 record->robot_blob_keypoints, CommandFeedback{});

        for (const auto& robot : robots.descriptions) {
            spdlog::info("[{}] {} pos=({:.2f}, {:.2f}) stale={}", record->tick,
                         magic_enum::enum_name(robot.frame_id), robot.pose.position.x,
                         robot.pose.position.y, robot.is_stale);
        }
        ++record_count;
    }

    spdlog::info("Replayed {} records from {}", record_count, input_path.string());
    return 0;
}
