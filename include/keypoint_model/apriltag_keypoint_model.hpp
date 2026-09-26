#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <optional>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "keypoint_model/config.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {

/** One PnP solution: rvec and tvec map the marker frame into the camera frame. */
struct TagPoseSolution {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    double reprojection_error_px = 0.0;
};

/** Both SOLVEPNP_IPPE_SQUARE solutions for one tag, sorted by reprojection error ascending.
 *  `corners` are in OpenCV aruco order, clockwise from the marker's top-left. Empty on failure. */
std::vector<TagPoseSolution> solve_tag_pose(const std::array<cv::Point2f, 4> &corners,
                                            double tag_size_m, const cv::Mat &intrinsics,
                                            const cv::Mat &distortion);

/** One robot tag's mounting, resolved from its config entry. */
struct TagMount {
    int id = 0;
    /** Marker frame to body frame (FLU at the axle midpoint). */
    cv::Matx33d rotation_body_tag = cv::Matx33d::eye();
    cv::Vec3d translation_body_tag{0.0, 0.0, 0.0};
    /** True when the printed face points up the body z axis, so the camera above sees it with
     *  the robot upright. A tag facing down shows only with the robot upside down. */
    bool faces_up = true;
    /** Floor up in the body frame while the robot rests with this tag showing. Upright it leans
     *  back by the rest pitch (the robot sits nose down on its wedge tip); upside down it is body
     *  -z, since the resting pitch on the top face is unmeasured. */
    cv::Vec3d floor_up_body{0.0, 0.0, 1.0};
};

TagMount make_tag_mount(const AprilTagMountConfiguration &config, double rest_pitch_rad);

/** The floor's up direction in the camera frame implied by one tag pose, assuming the robot
 *  rests on the floor. */
cv::Vec3d floor_up_from_tag_pose(const TagPoseSolution &solution, const TagMount &mount);

/** True when the pose puts the camera above the floor through the robot's axle. The planar flip
 *  of a steeply viewed tag can put it below, which no real frame does. */
bool camera_above_floor(const TagPoseSolution &solution, const TagMount &mount);

/**
 * Index of the IPPE solution to trust, from `solutions` sorted by reprojection error.
 *
 * A solution that puts the camera under the floor is dropped when the other does not. Of the
 * rest, the one whose implied floor up lies closest to the reference wins: the other IPPE
 * solution is the planar flip, which tilts the robot off the floor. The reference is
 * `floor_up_camera` when known. Without it the reference is camera -z, right for an overhead
 * camera and up to the tripod tilt off otherwise, so it only decides when the two solutions
 * differ by more than 15 degrees against it. Otherwise, or when the two lie within 2 degrees of
 * a known floor, the lowest reprojection error wins.
 */
size_t select_ippe_solution(const std::vector<TagPoseSolution> &solutions, const TagMount &mount,
                            const std::optional<cv::Vec3d> &floor_up_camera);

/**
 * Learns the floor's up direction in the (fixed) camera frame from the tags themselves.
 *
 * Every frame gives two candidates, one per IPPE solution. The right ones all agree, since the
 * robot drives on one floor; the flipped ones move with the robot's position and heading. Until
 * one direction has the support of most frames and no rival cluster does, estimate() stays empty
 * and selection falls back to reprojection error. After that it tracks slowly, so a wheelie or
 * a flip does not drag it.
 */
class FloorNormalEstimator {
   public:
    /** Before the estimate locks: record one frame's candidates. */
    void add_frame(const std::vector<cv::Vec3d> &candidates);
    /** After the estimate locks: nudge it toward the chosen candidate if it is close. */
    void refine(const cv::Vec3d &chosen);
    const std::optional<cv::Vec3d> &estimate() const { return estimate_; }

   private:
    void try_lock();

    std::vector<std::vector<cv::Vec3d>> frames_;
    std::optional<cv::Vec3d> estimate_;
};

/**
 * Keypoints for Mr Stabs Mk2 from its AprilTags, for sessions where the tag is ground truth.
 *
 * Per frame it searches a window around the last detection (the full frame when there is none,
 * or when the window finds nothing), solves both IPPE poses for each robot tag, and records the
 * raw measurement on `/apriltag/robot_tags`, frames without a detection included, so the corners
 * can be re-solved offline. It then composes the chosen tag pose with the tag's mounting, projects
 * the configured front and back body points into the image, and returns them labeled
 * MR_STABS_MK2 with their height above the floor, the same shape the YOLO model produces.
 */
class AprilTagKeypointModel : public KeypointModelInterface {
   public:
    AprilTagKeypointModel(const AprilTagKeypointModelConfiguration &config,
                          std::shared_ptr<VizSink> sink,
                          std::shared_ptr<McapRecorder> mcap_recorder);

    bool initialize() override { return true; }
    ModelResultStamped update(RgbImage image, const CameraInfo &camera_info) override;
    DetectionsStamped last_detections() const override { return last_detections_; }

    /** Window the next frame will search first; empty means the full frame. */
    const std::optional<cv::Rect> &search_roi() const { return roi_; }
    /** The last `/apriltag/robot_tags` payload. */
    const std::string &last_tags_json() const { return last_tags_json_; }
    const FloorNormalEstimator &floor_estimator() const { return floor_estimator_; }

   private:
    struct Detection {
        int id = 0;
        std::array<cv::Point2f, 4> corners;
        std::vector<TagPoseSolution> solutions;
    };

    std::vector<Detection> detect(const cv::Mat &gray, const cv::Rect &region) const;
    const TagMount *mount_for(int id) const;
    void emit_keypoints(const Detection &detection, const TagMount &mount,
                        const cv::Mat &intrinsics, const cv::Mat &distortion,
                        ModelResultStamped &result);
    std::string to_json(uint64_t image_stamp_ns, const RgbImage &image,
                        const CameraInfo &camera_info, const std::optional<cv::Rect> &roi,
                        const std::vector<Detection> &detections) const;

    AprilTagKeypointModelConfiguration config_;
    std::vector<TagMount> mounts_;
    cv::aruco::ArucoDetector detector_;
    OutputChannel channel_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    std::optional<cv::Rect> roi_;
    FloorNormalEstimator floor_estimator_;
    DetectionsStamped last_detections_;
    std::string last_tags_json_;
};

}  // namespace auto_battlebot
