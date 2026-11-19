#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "data_structures.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "robot_blob_model/robot_blob_model_interface.hpp"

namespace auto_battlebot {

struct BatchResult {
    ModelResultStamped keypoints;
    ModelResultStamped robot_blob_keypoints;
    double keypoint_model_elapsed_ms = 0.0;
    double robot_blob_model_elapsed_ms = 0.0;
};

class ParallelModelBatch {
   public:
    ParallelModelBatch(std::shared_ptr<KeypointModelInterface> keypoint_model,
                       std::shared_ptr<RobotBlobModelInterface> robot_blob_model);
    ~ParallelModelBatch();
    ParallelModelBatch(const ParallelModelBatch &) = delete;
    ParallelModelBatch &operator=(const ParallelModelBatch &) = delete;

    BatchResult update(const RgbImage &image, const CameraInfo &camera_info);

   private:
    using RunModel = std::function<ModelResultStamped(const RgbImage &, const CameraInfo &)>;
    void worker_loop(const RunModel &run_model, uint64_t &done_id, ModelResultStamped &result_slot,
                     double &elapsed_slot, std::atomic<bool> &exited_flag);

    std::shared_ptr<KeypointModelInterface> keypoint_model_;
    std::shared_ptr<RobotBlobModelInterface> robot_blob_model_;

    std::mutex mutex_;
    std::condition_variable dispatch_cv_;
    std::condition_variable done_cv_;
    RgbImage shared_image_;
    CameraInfo shared_camera_info_;
    uint64_t request_id_ = 0;
    uint64_t keypoint_done_id_ = 0;
    uint64_t robot_blob_done_id_ = 0;
    BatchResult result_;
    bool stop_ = false;

    std::atomic<bool> keypoint_thread_exited_{false};
    std::atomic<bool> robot_blob_thread_exited_{false};
    std::thread keypoint_thread_;
    std::thread robot_blob_thread_;
};

}  // namespace auto_battlebot
