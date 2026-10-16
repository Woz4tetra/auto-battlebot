#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "data_structures.hpp"

namespace auto_battlebot {

class SimConnection {
   public:
    static void configure(const std::string &host, int port);
    static std::shared_ptr<SimConnection> instance();

    ~SimConnection();

    SimConnection(const SimConnection &) = delete;
    SimConnection &operator=(const SimConnection &) = delete;

    bool connect();
    void disconnect();
    bool is_connected() const;

    void set_command(VelocityCommand cmd);
    bool step_and_receive(CameraData &data);

    const std::vector<Pose2D> &last_ground_truth_poses() const { return last_gt_poses_; }
    /** What simulated perception delivered on the last tick. */
    const SimMeasurementFrame &last_measurements() const { return last_measurements_; }
    /** Camera-from-field transform of the last tick: the sim's world origin is the field centre. */
    const Eigen::Matrix4d &last_camera_from_field() const { return last_camera_from_field_; }

   private:
    SimConnection(const std::string &host, int port);

    bool send_all(const void *buf, size_t len);
    bool recv_all(void *buf, size_t len);

    std::string host_;
    int port_;
    int sock_fd_ = -1;
    VelocityCommand pending_command_{0.0, 0.0, 0.0};
    std::vector<Pose2D> last_gt_poses_;
    SimMeasurementFrame last_measurements_;
    Eigen::Matrix4d last_camera_from_field_ = Eigen::Matrix4d::Identity();

    static std::shared_ptr<SimConnection> instance_;
};

}  // namespace auto_battlebot
