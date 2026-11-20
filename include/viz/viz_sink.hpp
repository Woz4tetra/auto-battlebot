#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "viz/frame.hpp"
#include "viz/schema.hpp"

namespace auto_battlebot {

class VizSink {
   public:
    using ClientMessageHandler =
        std::function<void(const std::string& topic, const std::byte* data, size_t len)>;

    explicit VizSink(std::string socket_path = viz::default_socket_path());
    ~VizSink();

    VizSink(const VizSink&) = delete;
    VizSink& operator=(const VizSink&) = delete;

    uint32_t advertise(const std::string& topic, const std::string& message_encoding,
                       const VizSchema& schema, bool latch);

    void publish(uint32_t channel_id, const std::byte* data, size_t len, uint64_t log_time_ns);

    uint32_t num_subscribers(uint32_t channel_id) const;

    void set_client_message_handler(ClientMessageHandler handler);

    bool connected() const { return connected_.load(); }
    uint64_t dropped_messages() const { return dropped_.load(); }
    const std::string& socket_path() const { return socket_path_; }

   private:
    struct SinkThreadState {
        std::vector<std::byte> out_;
        size_t out_sent_ = 0;
        std::chrono::steady_clock::time_point last_send_progress_{};
        std::chrono::steady_clock::time_point connected_at_{};
        std::chrono::steady_clock::time_point retry_at_{};
        size_t bytes_sent_ = 0;
        bool settled_ = false;
        bool lost_warned_ = false;
        bool unreachable_warned_ = false;
        std::vector<std::byte> read_buffer_;
    };

    void run();
    bool try_connect();
    void lose_connection(const char* reason, bool closed_by_relay = false);
    bool relay_has_read() const;
    void disconnect();
    void wait_for_wake(int timeout_ms);
    void wake();
    bool send_pending();
    bool take_next_frame();
    bool read_incoming();
    void clear_queues_locked();

    std::string socket_path_;
    int fd_ = -1;
    int wake_fd_ = -1;

    mutable std::mutex mutex_;
    std::deque<std::vector<std::byte>> priority_queue_;
    std::deque<std::vector<std::byte>> bulk_queue_;
    size_t priority_bytes_ = 0;
    size_t bulk_bytes_ = 0;
    std::vector<viz::AdvertiseFrame> advertises_;
    std::unordered_map<std::string, uint32_t> channel_by_topic_;
    std::unordered_set<uint32_t> bulk_channels_;
    std::unordered_map<uint32_t, uint32_t> subscribers_;
    std::mutex handler_mutex_;
    ClientMessageHandler client_message_handler_;

    std::atomic<bool> connected_{false};
    std::atomic<bool> stop_{false};
    std::atomic<uint64_t> dropped_{0};

    SinkThreadState sink_thread_state_;
    std::thread thread_;
};

}  // namespace auto_battlebot
