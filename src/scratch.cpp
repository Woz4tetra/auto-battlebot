#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <deque>
#include <magic_enum.hpp>
#include <opencv2/opencv.hpp>

#include "config/config.hpp"
#include "directories.hpp"
#include "label_utils.hpp"
#include "transform_utils.hpp"

namespace {
using namespace auto_battlebot;

constexpr size_t kDefaultHistorySize = 20;

// RobotFrontBackSimpleFilter never populates RobotDescription::size (it reads per-detection blob
// and keypoint sizes internally but never writes one back to its output), so it is always
// {0,0,0}. Fall back to an approximate 30lb-class footprint so the corners box below isn't a
// degenerate point.
constexpr double kFallbackRobotSizeMeters = 0.3;

// One previously-shown frame's display inputs, cached so 'p' can redraw it without re-touching
// the camera, GPU models, or robot_filter. rgb is cloned on capture: camera_data.rgb.image is
// reused by the next camera->get() call, so without cloning every cached entry would alias the
// same (constantly overwritten) buffer.
struct FrameHistoryEntry {
    cv::Mat rgb;
    KeypointsStamped keypoints;
    KeypointsStamped robot_blob_keypoints;
    RobotDescriptionsStamped robots;
    FieldDescription field_description;
    CameraInfo camera_info;
    int tick = 0;
    int64_t svo_frame_index = -1;
};

enum class Step { kNext, kPrev, kQuit };

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

// Projects a camera-frame 3D point to a pixel via pinhole intrinsics -- the inverse of
// transform_utils.hpp's pixel_to_camera_ray. False if the point is at/behind the camera (nothing
// sane to draw) or intrinsics are degenerate.
bool project_to_pixel(const CameraInfo& camera_info, const Eigen::Vector3d& point_camera,
                      cv::Point2d& out_pixel) {
    if (camera_info.intrinsics.rows != 3 || camera_info.intrinsics.cols != 3) return false;
    const double fx = camera_info.intrinsics.at<double>(0, 0);
    const double fy = camera_info.intrinsics.at<double>(1, 1);
    const double cx = camera_info.intrinsics.at<double>(0, 2);
    const double cy = camera_info.intrinsics.at<double>(1, 2);
    if (std::abs(fx) < 1e-6 || std::abs(fy) < 1e-6) return false;
    if (point_camera.z() <= 1e-3) return false;
    out_pixel.x = fx * point_camera.x() / point_camera.z() + cx;
    out_pixel.y = fy * point_camera.y() / point_camera.z() + cy;
    return std::isfinite(out_pixel.x) && std::isfinite(out_pixel.y);
}

void draw_dashed_segment(cv::Mat& img, cv::Point a, cv::Point b, const cv::Scalar& color,
                         int thickness) {
    constexpr int kDashes = 8;
    for (int d = 0; d < kDashes; d += 2) {
        const double t0 = static_cast<double>(d) / kDashes;
        const double t1 = static_cast<double>(d + 1) / kDashes;
        const cv::Point start(a.x + static_cast<int>((b.x - a.x) * t0),
                              a.y + static_cast<int>((b.y - a.y) * t0));
        const cv::Point end(a.x + static_cast<int>((b.x - a.x) * t1),
                            a.y + static_cast<int>((b.y - a.y) * t1));
        cv::line(img, start, end, color, thickness, cv::LINE_AA);
    }
}

// Reprojects one RobotDescription's estimated footprint into the image: its Size box in the XY
// plane, oriented by its Pose (robot-local corners -> field frame via pose_to_matrix -> camera
// frame via field.tf_camera_from_fieldcenter -> pixels). Solid outline for a fresh measurement,
// dashed for a predicted/stale one (robot_filter still emits a pose for those, dead-reckoned
// forward -- see docs/robot_filter_pipeline.md). Skips the whole box rather than drawing a
// degenerate partial polygon if any corner falls behind the camera.
void draw_robot_corners(cv::Mat& img, const RobotDescription& robot, const FieldDescription& field,
                        const CameraInfo& camera_info) {
    const Eigen::Matrix4d tf_field_from_robot = pose_to_matrix(robot.pose);
    const double size_x = robot.size.x > 0.0 ? robot.size.x : kFallbackRobotSizeMeters;
    const double size_y = robot.size.y > 0.0 ? robot.size.y : kFallbackRobotSizeMeters;
    const double hx = size_x * 0.5;
    const double hy = size_y * 0.5;
    const Eigen::Vector3d local_corners[4] = {
        {hx, hy, 0.0}, {hx, -hy, 0.0}, {-hx, -hy, 0.0}, {-hx, hy, 0.0}};

    std::vector<cv::Point> pixel_corners;
    pixel_corners.reserve(4);
    for (const auto& local : local_corners) {
        const Eigen::Vector3d field_point = transform_point(tf_field_from_robot, local);
        const Eigen::Vector3d camera_point =
            transform_point(field.tf_camera_from_fieldcenter.tf, field_point);
        cv::Point2d pixel;
        if (!project_to_pixel(camera_info, camera_point, pixel)) return;
        pixel_corners.emplace_back(static_cast<int>(std::round(pixel.x)),
                                   static_cast<int>(std::round(pixel.y)));
    }

    auto [b, g, r] = get_color_for_index(robot.frame_id).to_bgr_255();
    const cv::Scalar color(b, g, r);
    const int thickness = robot.is_stale ? 1 : 2;
    if (robot.is_stale) {
        for (size_t i = 0; i < pixel_corners.size(); ++i) {
            draw_dashed_segment(img, pixel_corners[i],
                                pixel_corners[(i + 1) % pixel_corners.size()], color, thickness);
        }
    } else {
        cv::polylines(img, pixel_corners, /*isClosed=*/true, color, thickness, cv::LINE_AA);
    }

    const std::string text =
        std::string(magic_enum::enum_name(robot.frame_id)) + (robot.is_stale ? " (stale)" : "");
    const cv::Point label_pos = pixel_corners[0] + cv::Point(4, -6);
    cv::putText(img, text, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2,
                cv::LINE_AA);
    cv::putText(img, text, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
}

// Draws one cached frame's keypoint sources on top of its RGB image and blocks until the user
// presses 'n' (next), 'p' (previous), or 'q'/Esc (quit). Any other key is ignored so a stray
// keypress can't skip a frame.
Step show_frame_and_wait_for_key(const FrameHistoryEntry& entry, bool has_older,
                                 bool history_full) {
    if (entry.rgb.empty()) return Step::kNext;
    cv::Mat vis = entry.rgb.clone();

    for (const auto& kp : entry.robot_blob_keypoints.keypoints) {
        draw_point(vis, kp.x, kp.y, kp.label, "blob:" + get_short_name(kp.label), 5);
    }
    for (const auto& kp : entry.keypoints.keypoints) {
        draw_point(vis, kp.x, kp.y, kp.label,
                   get_short_name(std::string(magic_enum::enum_name(kp.keypoint_label))), 4);
    }
    for (const auto& robot : entry.robots.descriptions) {
        draw_robot_corners(vis, robot, entry.field_description, entry.camera_info);
    }
    // svo_frame_index is the actual position in the file (vs. tick, this loop's own counter) --
    // watch it advance/retreat by exactly 1 per 'n'/'p' press to confirm capture is truly paused
    // in between and history bookkeeping lines up.
    cv::putText(vis,
                "tick " + std::to_string(entry.tick) + "  svo_frame " +
                    std::to_string(entry.svo_frame_index) + "  [n]ext  [p]rev" +
                    (has_older ? "" : " (oldest cached)") + "  [q]uit",
                cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2,
                cv::LINE_AA);
    if (history_full) {
        cv::putText(vis, "history full: oldest cached frame drops as new ones arrive",
                    cv::Point(10, 48), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 200, 255), 1,
                    cv::LINE_AA);
    }

    cv::imshow("scratch: robot_filter playback", vis);
    while (true) {
        int key = cv::waitKey(0) & 0xFF;
        if (key == 'q' || key == 27) return Step::kQuit;  // Esc also works
        if (key == 'n') return Step::kNext;
        if (key == 'p') return Step::kPrev;
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
// real X11 DISPLAY -- run via scripts/docker/dev_shell.sh without --no-display). 'p' goes back
// through a capped in-memory cache of the last `history size` frames shown -- purely a display
// rewind: it redraws a previously-captured frame without re-running the camera, GPU models, or
// robot_filter, so the filter's printed console output does not rewind with it (robot_filter is
// stateful -- its hold windows, temporal motion prediction, and frame-ID assignment memory all
// assume forward-only progress, so replaying an old tick's inputs into it now would desync that
// state rather than reproduce what it believed back then).
//
// Usage: ./scratch [config path or profile name] [history size]
//   Defaults: mrs_buff_mk3 playback profile, 20 cached frames.
int main(int argc, char** argv) {
    using namespace auto_battlebot;

    std::filesystem::path config_path =
        argc > 1 ? normalize_config_path(argv[1])
                 : get_config_dir() / "playback/mrs_buff_mk3_playback.toml";
    size_t history_size = argc > 2 ? static_cast<size_t>(std::stoul(argv[2])) : kDefaultHistorySize;
    if (history_size == 0) {
        spdlog::error("history size must be at least 1");
        return 1;
    }

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

    // history.back() is the most recently captured (live) frame. view_offset counts how many
    // steps back from that we're currently displaying: 0 means live, >0 means browsing the cache.
    // need_new_frame is deliberately a separate flag from "view_offset == 0": walking 'n' back
    // down to the live edge must first redisplay the already-cached history.back() (view_offset
    // hits 0 with need_new_frame still false), and only a *further* 'n' press from there should
    // fetch a genuinely new frame. Folding both into "view_offset == 0" would fetch (and silently
    // skip redisplaying) a new frame the instant the walk-back reached the live edge.
    std::deque<FrameHistoryEntry> history;
    size_t view_offset = 0;
    bool need_new_frame = true;

    while (true) {
        if (need_new_frame) {
            CameraData camera_data;
            // get_depth=true only while waiting to initialize the field: that is the one step
            // that needs a depth-carrying frame (field_filter->compute_field fits a plane to the
            // point cloud). Every tick after that only needs RGB, same as Runner::tick.
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

            // The full pipeline feeds real transmitter command feedback here for motion
            // prediction; an empty CommandFeedback{} just means the filter sees no commanded
            // velocity this tick.
            RobotDescriptionsStamped robots =
                robot_filter->update(keypoints, field_description, camera_data.camera_info,
                                     robot_blob_keypoints, CommandFeedback{});

            for (const auto& robot : robots.descriptions) {
                spdlog::info("[{}] {} pos=({:.2f}, {:.2f}) stale={}", tick,
                             magic_enum::enum_name(robot.frame_id), robot.pose.position.x,
                             robot.pose.position.y, robot.is_stale);
            }

            FrameHistoryEntry entry;
            entry.rgb = camera_data.rgb.image.clone();
            entry.keypoints = std::move(keypoints);
            entry.robot_blob_keypoints = std::move(robot_blob_keypoints);
            entry.robots = std::move(robots);
            entry.field_description = field_description;
            entry.camera_info = camera_data.camera_info;
            entry.tick = tick;
            entry.svo_frame_index = camera_data.frame_identity.svo_frame_index;
            history.push_back(std::move(entry));
            if (history.size() > history_size) history.pop_front();

            ++tick;
            need_new_frame = false;
            view_offset = 0;
        }

        const FrameHistoryEntry& displayed = history[history.size() - 1 - view_offset];
        bool has_older = view_offset + 1 < history.size();
        bool history_full = history.size() >= history_size;

        // Pause background capture while blocked on the keypress: ZedRgbdCamera otherwise keeps
        // grabbing (real-time-paced or not) the whole time nobody is calling get(), so the next
        // get() would silently return a frame far ahead of this one instead of the next one.
        // Kept paused for the whole wait regardless of view_offset, including while browsing
        // cached history, so nothing advances behind our back before the next live fetch.
        camera->pause_capture();
        Step step = show_frame_and_wait_for_key(displayed, has_older, history_full);
        camera->resume_capture();

        if (step == Step::kQuit) {
            spdlog::info("Quit requested.");
            break;
        }
        if (step == Step::kNext) {
            if (view_offset > 0) {
                view_offset--;  // walk back toward live using the cache; no fetch
            } else {
                need_new_frame = true;  // already at the live edge -- advance for real
            }
        } else if (step == Step::kPrev) {
            if (has_older) {
                view_offset++;
            } else {
                spdlog::info("No older cached frame (history holds the last {} frames).",
                             history.size());
            }
        }
    }

    cv::destroyAllWindows();
    return 0;
}
