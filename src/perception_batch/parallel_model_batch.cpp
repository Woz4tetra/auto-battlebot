#include "perception_batch/parallel_model_batch.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <utility>

namespace auto_battlebot {

namespace {
// Bound on how long update() waits for both models. ~30x the worst per-model time seen in
// latency reports; hitting it means a model is wedged, not slow.
constexpr std::chrono::milliseconds kUpdateWaitTimeout(1000);
constexpr std::chrono::seconds kJoinHardTimeout(2);

// Same pattern as ZedRgbdCamera: poll a done flag the worker sets on exit so shutdown can
// give up on a thread wedged inside a model update instead of blocking forever in join().
bool join_with_timeout(std::thread &thread, const std::atomic<bool> &exited_flag,
                       const char *context) {
    if (!thread.joinable()) return true;
    const auto deadline = std::chrono::steady_clock::now() + kJoinHardTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (exited_flag.load(std::memory_order_acquire)) {
            thread.join();
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    spdlog::critical("{}: worker thread stuck in model update after {}s; detaching", context,
                     kJoinHardTimeout.count());
    spdlog::default_logger()->flush();
    thread.detach();
    return false;
}
}  // namespace

ParallelModelBatch::ParallelModelBatch(std::shared_ptr<KeypointModelInterface> keypoint_model,
                                       std::shared_ptr<RobotBlobModelInterface> robot_blob_model)
    : keypoint_model_(std::move(keypoint_model)), robot_blob_model_(std::move(robot_blob_model)) {
    keypoint_thread_ = std::thread([this] {
        worker_loop([this](const RgbImage &image) { return keypoint_model_->update(image); },
                    keypoint_done_id_, result_.keypoints, result_.keypoint_model_elapsed_ms,
                    keypoint_thread_exited_);
    });
    robot_blob_thread_ = std::thread([this] {
        worker_loop([this](const RgbImage &image) { return robot_blob_model_->update(image); },
                    robot_blob_done_id_, result_.robot_blob_keypoints,
                    result_.robot_blob_model_elapsed_ms, robot_blob_thread_exited_);
    });
}

ParallelModelBatch::~ParallelModelBatch() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    dispatch_cv_.notify_all();
    join_with_timeout(keypoint_thread_, keypoint_thread_exited_,
                      "ParallelModelBatch keypoint worker");
    join_with_timeout(robot_blob_thread_, robot_blob_thread_exited_,
                      "ParallelModelBatch robot blob worker");
}

void ParallelModelBatch::worker_loop(
    const std::function<ModelResultStamped(const RgbImage &)> &run_model, uint64_t &done_id,
    ModelResultStamped &result_slot, double &elapsed_slot, std::atomic<bool> &exited_flag) {
    uint64_t last_seen_id = 0;
    while (true) {
        RgbImage image;
        uint64_t id = 0;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            dispatch_cv_.wait(lock, [&] { return stop_ || request_id_ != last_seen_id; });
            if (stop_) break;
            id = request_id_;
            last_seen_id = id;
            image = shared_image_;
        }
        const auto start = std::chrono::steady_clock::now();
        ModelResultStamped output = run_model(image);
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Drop the result if update() already timed out on this request and moved on.
            if (id == request_id_) {
                result_slot = std::move(output);
                elapsed_slot = elapsed_ms;
                done_id = id;
            }
        }
        done_cv_.notify_one();
    }
    exited_flag.store(true, std::memory_order_release);
}

BatchResult ParallelModelBatch::update(const RgbImage &image) {
    uint64_t id = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shared_image_ = image;
        result_ = BatchResult{};
        id = ++request_id_;
    }
    dispatch_cv_.notify_all();

    std::unique_lock<std::mutex> lock(mutex_);
    const bool completed = done_cv_.wait_for(lock, kUpdateWaitTimeout, [&] {
        return keypoint_done_id_ == id && robot_blob_done_id_ == id;
    });
    if (!completed) {
        spdlog::error(
            "ParallelModelBatch::update timed out after {} ms (keypoint done: {}, robot blob "
            "done: {})",
            kUpdateWaitTimeout.count(), keypoint_done_id_ == id, robot_blob_done_id_ == id);
    }
    return std::move(result_);
}

}  // namespace auto_battlebot
