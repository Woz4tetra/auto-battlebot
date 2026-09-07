#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "viz/frame.hpp"
#include "viz/schema.hpp"

namespace auto_battlebot {

/**
 * App-side client of the viz_relay unix socket.
 *
 * Never blocks the caller and never fails startup: `publish()` copies the encoded frame into a
 * bounded queue and returns. A background thread owns the socket, connects (and reconnects on a
 * one second timer), replays every advertise after a reconnect, drains the queue with blocking
 * writes, and reads subscriber counts flowing back from the relay. When the relay is down the
 * queue is dropped on the floor and `dropped_messages()` counts what was lost.
 */
class VizSink {
   public:
    explicit VizSink(std::string socket_path = viz::default_socket_path());
    ~VizSink();

    VizSink(const VizSink&) = delete;
    VizSink& operator=(const VizSink&) = delete;

    /** Register a topic. Returns the channel id to pass to publish(). Idempotent per topic. */
    uint32_t advertise(const std::string& topic, const std::string& message_encoding,
                       const VizSchema& schema, bool latch);

    void publish(uint32_t channel_id, const std::byte* data, size_t len, uint64_t log_time_ns);

    uint32_t num_subscribers(uint32_t channel_id) const;
    bool connected() const { return connected_.load(); }
    uint64_t dropped_messages() const { return dropped_.load(); }
    const std::string& socket_path() const { return socket_path_; }

   private:
    void run();
    bool try_connect();
    void disconnect();
    bool send_all(const std::vector<std::byte>& frame);
    void read_incoming();

    std::string socket_path_;
    int fd_ = -1;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::vector<std::byte>> queue_;
    size_t queued_bytes_ = 0;
    std::vector<viz::AdvertiseFrame> advertises_;
    std::unordered_map<std::string, uint32_t> channel_by_topic_;
    std::unordered_map<uint32_t, uint32_t> subscribers_;

    std::atomic<bool> connected_{false};
    std::atomic<bool> stop_{false};
    std::atomic<uint64_t> dropped_{0};
    bool lost_warned_ = false;
    std::vector<std::byte> read_buffer_;
    std::thread thread_;
};

}  // namespace auto_battlebot
