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

// Result of one parallel perception tick: both model outputs plus the per-model wall
// times measured inside the workers, so the caller can still attribute latency per model.
struct BatchResult {
    ModelResultStamped keypoints;
    ModelResultStamped robot_blob_keypoints;
    double keypoint_model_elapsed_ms = 0.0;
    double robot_blob_model_elapsed_ms = 0.0;
};

/**
 * @brief Runs the keypoint model and the robot blob model in parallel on two persistent
 * worker threads, joining on both before returning.
 *
 * Depends only on the model interfaces, so any implementation selected by the factories
 * (TensorRT, noop, ...) stays swappable without changes here. Worker threads are created
 * once at construction and woken per tick via a condition variable, mirroring the ZED
 * producer/consumer handshake, to avoid per-frame thread creation in the hot loop.
 */
class ParallelModelBatch {
   public:
    ParallelModelBatch(std::shared_ptr<KeypointModelInterface> keypoint_model,
                       std::shared_ptr<RobotBlobModelInterface> robot_blob_model);
    ~ParallelModelBatch();
    ParallelModelBatch(const ParallelModelBatch &) = delete;
    ParallelModelBatch &operator=(const ParallelModelBatch &) = delete;

    /**
     * @brief Dispatch the image to both workers and block until both finish.
     *
     * The wait is bounded so a hung model cannot stall the main loop forever; on timeout
     * the missing model's result fields are left default-constructed (empty keypoints).
     */
    BatchResult update(const RgbImage &image);

   private:
    void worker_loop(const std::function<ModelResultStamped(const RgbImage &)> &run_model,
                     uint64_t &done_id, ModelResultStamped &result_slot, double &elapsed_slot,
                     std::atomic<bool> &exited_flag);

    std::shared_ptr<KeypointModelInterface> keypoint_model_;
    std::shared_ptr<RobotBlobModelInterface> robot_blob_model_;

    std::mutex mutex_;
    std::condition_variable dispatch_cv_;  // main -> workers: new request or stop
    std::condition_variable done_cv_;      // workers -> main: a result landed
    // All fields below are guarded by mutex_. shared_image_ is a shallow cv::Mat copy;
    // each worker takes its own shallow copy so the pixel buffer stays alive even if
    // update() times out and the caller's image goes away.
    RgbImage shared_image_;
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
