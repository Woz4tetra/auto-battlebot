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

/**
 * App-side client of the viz_relay unix socket.
 *
 * Never blocks the caller and never fails startup: `publish()` copies the encoded frame into a
 * bounded queue and returns. A background thread owns the socket, connects (and reconnects on a
 * one second timer), replays every advertise after a reconnect, drains the queues, and reads
 * subscriber counts and client messages flowing back from the relay. When the relay is down the
 * queues are dropped on the floor and `dropped_messages()` counts what was lost.
 *
 * The socket is non-blocking and the thread polls it for reading and writing at once, so a relay
 * that stops reading cannot hold up commands coming the other way, or the destructor. A frame
 * that makes no progress for kSendStallTimeout drops the connection and reconnects.
 *
 * Image, video and point cloud channels go in a bulk queue; everything else (status, logs,
 * scene updates) goes in a priority queue that is sent first, so status never waits behind
 * seconds of camera frames. Each channel stays in one queue, so its own messages keep their order.
 *
 * Messages a Foxglove client publishes (e.g. `{}` on /command/reinit_field) come back through the
 * relay and reach the client message handler on the sink thread.
 */
class VizSink {
   public:
    using ClientMessageHandler =
        std::function<void(const std::string& topic, const std::byte* data, size_t len)>;

    explicit VizSink(std::string socket_path = viz::default_socket_path());
    ~VizSink();

    VizSink(const VizSink&) = delete;
    VizSink& operator=(const VizSink&) = delete;

    /** Register a topic. Returns the channel id to pass to publish(). Idempotent per topic. */
    uint32_t advertise(const std::string& topic, const std::string& message_encoding,
                       const VizSchema& schema, bool latch);

    void publish(uint32_t channel_id, const std::byte* data, size_t len, uint64_t log_time_ns);

    uint32_t num_subscribers(uint32_t channel_id) const;

    /** Runs on the sink thread and must not block. Pass nullptr to clear; clearing waits for an
     * in-flight call, so whatever the handler captured may be destroyed once this returns. */
    void set_client_message_handler(ClientMessageHandler handler);

    bool connected() const { return connected_.load(); }
    uint64_t dropped_messages() const { return dropped_.load(); }
    const std::string& socket_path() const { return socket_path_; }

   private:
    void run();
    bool try_connect();
    /** Ends the connection; the next attempt waits kReconnectInterval. `closed_by_relay` is
     *  true when the relay hung up, rather than this side giving up on it. */
    void lose_connection(const char* reason, bool closed_by_relay = false);
    /** The relay has read some of what this connection sent. A frozen relay's listen backlog
     *  still completes connect() and buffers the first sends, so a connection alone proves
     *  nothing. */
    bool relay_has_read() const;
    void disconnect();
    /** Waits up to `timeout` for a wake() or for stop. */
    void wait_for_wake(int timeout_ms);
    void wake();
    /** Sends what the socket takes of out_. False when the connection is gone. */
    bool send_pending();
    /** Moves the next queued frame, priority first, into out_. */
    bool take_next_frame();
    /** False when the relay closed the connection or sent garbage. */
    bool read_incoming();
    void clear_queues_locked();

    std::string socket_path_;
    int fd_ = -1;
    /** eventfd: publish() and the destructor wake the sink thread out of poll(). */
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

    // Sink thread only.
    std::vector<std::byte> out_;
    size_t out_sent_ = 0;
    std::chrono::steady_clock::time_point last_send_progress_{};
    std::chrono::steady_clock::time_point connected_at_{};
    std::chrono::steady_clock::time_point retry_at_{};
    /** Bytes written to this connection. */
    size_t bytes_sent_ = 0;
    /** The connection has lasted long enough, and the relay has read from it, so it counts as
     *  attached rather than refused or frozen. */
    bool settled_ = false;
    bool lost_warned_ = false;
    bool unreachable_warned_ = false;
    std::vector<std::byte> read_buffer_;
    std::thread thread_;
};

}  // namespace auto_battlebot
