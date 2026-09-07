#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <foxglove/error.hpp>
#include <memory>
#include <string>
#include <vector>

#include "mcap_recorder/mcap_recorder.hpp"
#include "viz/schema.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {

/** Wall clock in nanoseconds since the epoch; the MCAP log time for everything recorded. */
inline uint64_t wall_time_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

/**
 * One topic, fanned out to the live relay and the MCAP recorder from a single encode.
 *
 * Either sink may be null. `has_consumers()` is the gate producers check before doing expensive
 * work such as JPEG encoding: it is true when a Foxglove client subscribes to the topic or the
 * recorder would write it.
 */
class OutputChannel {
   public:
    OutputChannel(std::string topic, std::string message_encoding, VizSchema schema, bool latch,
                  std::shared_ptr<VizSink> sink, std::shared_ptr<McapRecorder> recorder);

    const std::string& topic() const { return topic_; }

    /** Send pre-encoded bytes. */
    void log(const std::byte* data, size_t len, uint64_t log_time_ns);
    void log(const std::string& text, uint64_t log_time_ns) {
        log(reinterpret_cast<const std::byte*>(text.data()), text.size(), log_time_ns);
    }

    /** Encode a Foxglove SDK schema struct once and send it to both sinks. */
    template <typename Message>
    bool log_message(Message& message, uint64_t log_time_ns) {
        size_t encoded_len = 0;
        auto status = message.encode(buffer_.data(), buffer_.size(), &encoded_len);
        if (status == foxglove::FoxgloveError::BufferTooShort) {
            buffer_.resize(encoded_len);
            status = message.encode(buffer_.data(), buffer_.size(), &encoded_len);
        }
        if (status != foxglove::FoxgloveError::Ok) return false;
        log(reinterpret_cast<const std::byte*>(buffer_.data()), encoded_len, log_time_ns);
        return true;
    }

    uint32_t num_subscribers() const;
    bool records() const;
    bool has_consumers() const { return num_subscribers() > 0 || records(); }

   private:
    std::string topic_;
    std::shared_ptr<VizSink> sink_;
    std::shared_ptr<McapRecorder> recorder_;
    uint32_t sink_channel_ = 0;
    McapRecorder::ChannelId recorder_channel_ = 0;
    std::vector<uint8_t> buffer_;
};

}  // namespace auto_battlebot
