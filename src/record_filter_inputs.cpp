#include <spdlog/spdlog.h>

#include <fstream>

#include "config/config.hpp"
#include "directories.hpp"
#include "robot_filter/filter_input_record.hpp"

// Batch-extracts robot_filter's inputs (keypoints, robot_blob_keypoints, camera_info,
// field_description -- see include/robot_filter/filter_input_record.hpp) from a live run over an
// SVO, one JSON line per tick. Shares its camera/perception setup with scratch.cpp, but doesn't
// construct robot_filter at all: this tool only cares about what feeds it, not its output. Pairs
// with replay_filter_inputs.cpp, which reads the file back and drives robot_filter->update()
// without a camera, GPU models, or the ZED SDK.
//
// Usage: ./record_filter_inputs [config path or profile name] [output .jsonl path] [max ticks]
//   Defaults: mrs_buff_mk3 playback profile, ./filter_inputs.jsonl, unlimited (runs to EOF).
int main(int argc, char** argv) {
    using namespace auto_battlebot;

    std::filesystem::path config_path =
        (argc > 1) ? normalize_config_path(argv[1])
                   : get_config_dir() / "playback/mrs_buff_mk3_playback.toml";
    std::filesystem::path output_path =
        (argc > 2) ? std::filesystem::path(argv[2]) : std::filesystem::path("filter_inputs.jsonl");
    int max_ticks = (argc > 3) ? std::stoi(argv[3]) : -1;

    spdlog::info("Loading config: {}", config_path.string());
    ClassConfiguration class_config = load_classes_from_config(config_path);

    // A batch extraction wants to run through the recording as fast as possible, not paced to
    // the SVO's original real time -- see the svo_real_time_mode discussion in scratch.cpp.
    if (class_config.camera->type == "ZedRgbdCamera") {
        config_cast<ZedRgbdCameraConfiguration>(*class_config.camera).svo_real_time_mode = false;
    }

    auto camera = make_rgbd_camera(*class_config.camera);
    auto field_model = make_mask_model(*class_config.field_model);
    auto robot_mask_model = make_robot_blob_model(*class_config.robot_mask_model);
    auto keypoint_model = make_keypoint_model(*class_config.keypoint_model);
    auto field_filter = make_field_filter(*class_config.field_filter);

    if (!camera->initialize()) {
        spdlog::error("Failed to initialize camera");
        return 1;
    }
    if (!field_model->initialize()) {
        spdlog::error("Failed to initialize field model");
        return 1;
    }
    if (!robot_mask_model->initialize()) {
        spdlog::error("Failed to initialize robot blob model");
        return 1;
    }
    if (!keypoint_model->initialize()) {
        spdlog::error("Failed to initialize keypoint model");
        return 1;
    }

    std::ofstream out(output_path, std::ios::out | std::ios::trunc);
    if (!out) {
        spdlog::error("Failed to open {} for writing", output_path.string());
        return 1;
    }
    spdlog::info("Writing filter inputs to {}", output_path.string());

    std::shared_ptr<FieldDescriptionWithInlierPoints> initial_field;
    bool field_initialized = false;
    int tick = 0;

    while (max_ticks < 0 || tick < max_ticks) {
        CameraData camera_data;
        if (!camera->get(camera_data, /*get_depth=*/!field_initialized)) {
            if (camera->should_close()) {
                spdlog::info("Camera signalled end of playback after {} ticks.", tick);
            } else {
                spdlog::warn("camera->get() failed without should_close(); stopping.");
            }
            break;
        }

        if (!field_initialized) {
            if (!camera_data.tracking_ok) continue;
            field_filter->reset(camera_data.tf_visodom_from_camera);
            MaskStamped field_mask = field_model->update(camera_data.rgb);
            if (field_mask.mask.mask.empty()) {
                spdlog::warn("Empty field mask this frame; waiting for a usable one.");
                continue;
            }
            initial_field = field_filter->compute_field(camera_data, field_mask);
            if (initial_field->header.frame_id == FrameId::EMPTY) {
                spdlog::warn("Failed to find a field plane this frame; retrying.");
                continue;
            }
            field_initialized = true;
            spdlog::info("Field initialized after {} ticks.", tick);
        }

        FilterInputRecord record;
        record.tick = tick;
        record.svo_frame_index = camera_data.frame_identity.svo_frame_index;
        record.keypoints = keypoint_model->update(camera_data.rgb);
        record.robot_blob_keypoints = robot_mask_model->update(camera_data.rgb);
        record.camera_info = camera_data.camera_info;
        record.field_description =
            field_filter->track_field(camera_data.tf_visodom_from_camera, initial_field);

        write_filter_input_record(out, record);

        if (tick % 100 == 0) {
            spdlog::info("tick {} (svo_frame {})", tick, record.svo_frame_index);
        }
        ++tick;
    }

    out.close();
    spdlog::info("Wrote {} records to {}", tick, output_path.string());
    return 0;
}
