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

struct TagPoseSolution {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    double reprojection_error_px = 0.0;
};

std::vector<TagPoseSolution> solve_tag_pose(const std::array<cv::Point2f, 4> &corners,
                                            double tag_size_m, const cv::Mat &intrinsics,
                                            const cv::Mat &distortion);

struct TagMount {
    int id = 0;
    cv::Matx33d rotation_body_tag = cv::Matx33d::eye();
    cv::Vec3d translation_body_tag{0.0, 0.0, 0.0};
    bool printed_face_up_body_z = true;
    cv::Vec3d floor_up_body{0.0, 0.0, 1.0};
};

TagMount make_tag_mount(const AprilTagMountConfiguration &config, double rest_pitch_rad);

cv::Vec3d floor_up_from_tag_pose(const TagPoseSolution &solution, const TagMount &mount);

/** True when the pose puts the camera above the floor through the robot's axle. The planar flip
 *  of a steeply viewed tag can put it below, which no real frame does. */
bool camera_above_floor(const TagPoseSolution &solution, const TagMount &mount);

size_t select_ippe_solution(const std::vector<TagPoseSolution> &solutions, const TagMount &mount,
                            const std::optional<cv::Vec3d> &floor_up_camera);

class FloorNormalEstimator {
   public:
    void add_frame(const std::vector<cv::Vec3d> &candidates);
    void refine(const cv::Vec3d &chosen);
    const std::optional<cv::Vec3d> &estimate() const { return estimate_; }

   private:
    void try_lock();

    std::vector<std::vector<cv::Vec3d>> frames_;
    std::optional<cv::Vec3d> estimate_;
};

class AprilTagKeypointModel : public KeypointModelInterface {
   public:
    AprilTagKeypointModel(const AprilTagKeypointModelConfiguration &config,
                          std::shared_ptr<VizSink> sink,
                          std::shared_ptr<McapRecorder> mcap_recorder);

    bool initialize() override { return true; }
    ModelResultStamped update(RgbImage image, const CameraInfo &camera_info) override;
    DetectionsStamped last_detections() const override { return last_detections_; }

    const std::optional<cv::Rect> &search_roi() const { return roi_; }
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
