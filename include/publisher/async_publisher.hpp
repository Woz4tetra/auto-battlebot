#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "publisher/publisher_interface.hpp"

namespace auto_battlebot {

class AsyncPublisher : public PublisherInterface {
   public:
    static constexpr size_t kMaxPendingCalls = 64;

    explicit AsyncPublisher(std::shared_ptr<PublisherInterface> inner);
    ~AsyncPublisher() override;

    AsyncPublisher(const AsyncPublisher &) = delete;
    AsyncPublisher &operator=(const AsyncPublisher &) = delete;
    AsyncPublisher(AsyncPublisher &&) = delete;
    AsyncPublisher &operator=(AsyncPublisher &&) = delete;

    void publish_camera_data(const CameraData &data) override;
    void publish_field_mask(const MaskStamped &field_mask, const RgbImage &image,
                            const CameraInfo &camera_info) override;
    void publish_initial_field_description(const FieldDescriptionWithInlierPoints &field) override;
    void publish_field_description(
        const FieldDescription &field_description,
        const FieldDescriptionWithInlierPoints &initial_field_description) override;
    void publish_hazards(const FieldDescription &field_description) override;
    void publish_robots(const RobotDescriptionsStamped &robots) override;
    void publish_keypoint_detections(const DetectionsStamped &detections) override;
    void publish_navigation(const NavigationVisualization &nav) override;
    void flush() override;

   private:
    void enqueue(std::function<void()> call);
    void run();

    std::shared_ptr<PublisherInterface> inner_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable space_cv_;
    std::condition_variable idle_cv_;
    std::deque<std::function<void()>> pending_;
    uint64_t enqueued_ = 0;
    uint64_t completed_ = 0;
    bool stop_ = false;
    std::thread thread_;
};

}  // namespace auto_battlebot
