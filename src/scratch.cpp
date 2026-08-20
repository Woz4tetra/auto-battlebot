#include <spdlog/spdlog.h>

#include <magic_enum.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include "config/config.hpp"
#include "directories.hpp"
#include "label_utils.hpp"

namespace {
using namespace auto_battlebot;

// Small filled circle + label text, shared by both keypoint sources below; only the marker
// radius and prefix differ, so the two callers stay one-liners.
void draw_point(cv::Mat& img, double x, double y, Label label, const std::string& text,
                int radius) {
    cv::Point pt(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)));
    auto [b, g, r] = get_color_for_index(label).to_bgr_255();
    cv::circle(img, pt, radius + 2, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    cv::circle(img, pt, radius, cv::Scalar(b, g, r), -1, cv::LINE_AA);
    cv::putText(img, text, pt + cv::Point(radius + 4, 4), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::putText(img, text, pt + cv::Point(radius + 3, 3), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

// Draws both keypoint sources on top of the frame and blocks until the user presses 'n' (next
// frame) or 'q'/Esc (quit). Any other key is ignored so a stray keypress can't skip a frame.
// Returns false on quit.
bool show_frame_and_wait_for_key(const cv::Mat& rgb, const KeypointsStamped& keypoints,
                                 const KeypointsStamped& robot_blob_keypoints, int tick,
                                 int64_t svo_frame_index) {
    // rgb.setTo(cv::Scalar(255, 255, 255));
    if (rgb.empty()) return true;
    cv::Mat vis = rgb.clone();

    for (const auto& kp : robot_blob_keypoints.keypoints) {
        draw_point(vis, kp.x, kp.y, kp.label, "blob:" + get_short_name(kp.label), 5);
    }
    for (const auto& kp : keypoints.keypoints) {
        draw_point(vis, kp.x, kp.y, kp.label,
                   get_short_name(std::string(magic_enum::enum_name(kp.keypoint_label))), 4);
    }
    // svo_frame_index is the actual position in the file (vs. tick, this loop's own counter) --
    // watch it advance by exactly 1 per 'n' press to confirm capture is truly paused in between.
    cv::putText(vis,
                "tick " + std::to_string(tick) + "  svo_frame " + std::to_string(svo_frame_index) +
                    "  [n]ext  [q]uit",
                cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2,
                cv::LINE_AA);

    cv::imshow("scratch: robot_filter playback", vis);
    while (true) {
        int key = cv::waitKey(0) & 0xFF;
        if (key == 'q' || key == 27) return false;  // quit (Esc also works)
        if (key == 'n') return true;                // advance to the next frame
        // Anything else (window manager events, modifier keys, ...) -- keep waiting.
    }
}
}  // namespace

// Runs RobotFrontBackSimpleFilter (whichever robot_filter the config selects) against the same
// SVO and models the real playback profile uses. Reuses load_classes_from_config, so this stays
// one call in sync with whatever main.cpp does -- only the Runner's UI/transmitter/navigation/
// publisher/mcap machinery is stripped out. Mirrors Runner::initialize()/initialize_field()/
// tick() at the calls actually needed to drive robot_filter_->update().
//
// Shows the RGB frame with keypoint/blob detections overlaid and single-steps on 'n' (needs a
// real X11 DISPLAY -- run via scripts/docker/dev_shell.sh without --no-display).
//
// Usage: ./scratch [config path or profile name], defaults to the mrs_buff_mk3 playback profile.
int main(int argc, char** argv) {
    using namespace auto_battlebot;

    std::filesystem::path config_path =
        argc > 1 ? normalize_config_path(argv[1])
                 : get_config_dir() / "playback/mrs_buff_mk3_playback.toml";

    spdlog::info("Loading config: {}", config_path.string());
    ClassConfiguration class_config = load_classes_from_config(config_path);

    // pause_capture() below stops ZedRgbdCamera's background thread from grabbing while we're
    // blocked on the keypress, but that alone isn't enough: with svo_real_time_mode left on (the
    // config's default for playback), the ZED SDK's own real-time pacing tracks wall-clock time
    // across grab() calls and makes the very next grab() after resuming skip ahead to "catch up"
    // to however long we were paused -- confirmed directly against the SDK (delta of ~160 frames
    // after a 5s wait, pause_svo_reading() included, vs. exactly 1 with this flag off). See
    // sl::InitParameters::svo_real_time_mode: "Each grab() call increases [the SVO position] by
    // one, except when using svo_real_time_mode." Forcing it off here only affects this
    // single-step debug tool, not the real playback config Runner/main.cpp use.
    if (class_config.camera->type == "ZedRgbdCamera") {
        config_cast<ZedRgbdCameraConfiguration>(*class_config.camera).svo_real_time_mode = false;
    }

    auto camera = make_rgbd_camera(*class_config.camera);
    auto field_model = make_mask_model(*class_config.field_model);
    auto robot_mask_model = make_robot_blob_model(*class_config.robot_mask_model);
    auto keypoint_model = make_keypoint_model(*class_config.keypoint_model);
    auto field_filter = make_field_filter(*class_config.field_filter);
    auto clock = make_clock(*class_config.clock);
    auto robot_filter = make_robot_filter(*class_config.robot_filter, clock);

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

    std::shared_ptr<FieldDescriptionWithInlierPoints> initial_field;
    bool field_initialized = false;
    int tick = 0;

    while (true) {
        CameraData camera_data;
        // get_depth=true only while waiting to initialize the field: that is the one step that
        // needs a depth-carrying frame (field_filter->compute_field fits a plane to the point
        // cloud). Every tick after that only needs RGB, same as Runner::tick.
        if (!camera->get(camera_data, /*get_depth=*/!field_initialized)) {
            if (camera->should_close()) {
                spdlog::info("Camera signalled end of playback after {} ticks.", tick);
            } else {
                spdlog::warn("camera->get() failed without should_close(); stopping.");
            }
            break;
        }
        if (clock) clock->set(camera_data.rgb.header.stamp);

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
            robot_filter->initialize(class_config.runner.default_opponent_count);
            field_initialized = true;
            spdlog::info("Field initialized after {} ticks.", tick);
        }

        FieldDescription field_description =
            field_filter->track_field(camera_data.tf_visodom_from_camera, initial_field);
        KeypointsStamped keypoints = keypoint_model->update(camera_data.rgb);
        KeypointsStamped robot_blob_keypoints = robot_mask_model->update(camera_data.rgb);

        // The full pipeline feeds real transmitter command feedback here for motion prediction;
        // an empty CommandFeedback{} just means the filter sees no commanded velocity this tick.
        RobotDescriptionsStamped robots =
            robot_filter->update(keypoints, field_description, camera_data.camera_info,
                                 robot_blob_keypoints, CommandFeedback{});

        for (const auto& robot : robots.descriptions) {
            spdlog::info("[{}] {} pos=({:.2f}, {:.2f}) stale={}", tick,
                         magic_enum::enum_name(robot.frame_id), robot.pose.position.x,
                         robot.pose.position.y, robot.is_stale);
        }

        // Pause background capture while blocked on the keypress: ZedRgbdCamera otherwise keeps
        // grabbing (real-time-paced or not) the whole time nobody is calling get(), so the next
        // get() would silently return a frame far ahead of this one instead of the next one.
        camera->pause_capture();
        bool advance =
            show_frame_and_wait_for_key(camera_data.rgb.image, keypoints, robot_blob_keypoints,
                                        tick, camera_data.frame_identity.svo_frame_index);
        camera->resume_capture();
        if (!advance) {
            spdlog::info("Quit requested.");
            break;
        }

        ++tick;
    }

    cv::destroyAllWindows();
    return 0;
}
