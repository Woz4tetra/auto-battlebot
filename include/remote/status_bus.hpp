#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>

#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "remote/protocol.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot::remote {

/**
 * Publishes the status topics in RemoteTopics::status to the relay and the MCAP recorder.
 *
 * Every status channel is advertised at construction, so a client sees the full list before the
 * first message. Throttling is UI pacing on std::chrono::steady_clock and never feeds control, so
 * it does not follow the playback clock. Thread-safe; publish() never blocks on the network.
 */
class StatusBus {
   public:
    StatusBus(std::shared_ptr<VizSink> sink, std::shared_ptr<McapRecorder> recorder);

    /** Serializes and publishes on the topic RemoteTopics maps to Message. Drops the call when
     *  the topic's max_rate_hz says it is too soon. Returns whether it published. */
    template <typename Message>
    bool publish(const Message& message) {
        constexpr std::size_t index = row_index<Message>();
        const auto& row = std::get<index>(RemoteTopics::status);
        std::lock_guard<std::mutex> lock(mutex_);
        Slot& slot = slots_[index];
        const auto now = std::chrono::steady_clock::now();
        if (row.max_rate_hz > 0.0 && slot.published_once &&
            now - slot.last_publish < std::chrono::duration<double>(1.0 / row.max_rate_hz)) {
            return false;
        }
        slot.last_publish = now;
        slot.published_once = true;
        slot.channel->log(to_json_string(message), wall_time_ns());
        return true;
    }

   private:
    struct Slot {
        std::unique_ptr<OutputChannel> channel;
        std::chrono::steady_clock::time_point last_publish{};
        bool published_once = false;
    };

    template <typename Message, std::size_t I = 0>
    static constexpr std::size_t row_index() {
        using Row = std::tuple_element_t<I, StatusRows>;
        if constexpr (std::is_same_v<typename Row::message_type, Message>) {
            return I;
        } else {
            static_assert(I + 1 < std::tuple_size_v<StatusRows>,
                          "message type has no row in RemoteTopics::status");
            return row_index<Message, I + 1>();
        }
    }

    std::mutex mutex_;
    std::array<Slot, std::tuple_size_v<StatusRows>> slots_;
};

/** Records every drained command to MCAP on its own command topic. Recorder only: the relay
 *  already has these topics as client channels, so the app never advertises them. */
class CommandLog {
   public:
    explicit CommandLog(std::shared_ptr<McapRecorder> recorder);
    void record(const RemoteCommand& command);

   private:
    std::array<std::unique_ptr<OutputChannel>, std::variant_size_v<RemoteCommand>> channels_;
};

}  // namespace auto_battlebot::remote
