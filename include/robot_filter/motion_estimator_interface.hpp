#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/command_feedback.hpp"
#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "robot_filter/frame_id_assigner.hpp"

namespace auto_battlebot {

struct MotionEstimatorContext {
    Eigen::Matrix4d tf_fieldcenter_from_camera = Eigen::Matrix4d::Identity();
    CameraInfo camera_info;
    double field_bounds_margin_meters = 0.0;
};

class MotionEstimatorInterface {
   public:
    virtual ~MotionEstimatorInterface() = default;

    virtual void reset() = 0;

    virtual void predict(double now, const CommandFeedback &command_feedback) = 0;

    virtual std::vector<RobotDescription> update(std::vector<RobotDescription> measurements,
                                                 double timestamp,
                                                 FrameIdAssigner &frame_id_assigner,
                                                 const FieldDescription &field,
                                                 const MotionEstimatorContext &context) = 0;

    virtual std::optional<std::vector<RobotDescription>> coast([[maybe_unused]] double now) {
        return std::nullopt;
    }

    virtual double render_lead_s() const { return 0.0; }
};

}  // namespace auto_battlebot
