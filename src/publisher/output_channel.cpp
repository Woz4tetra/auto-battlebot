#include "publisher/output_channel.hpp"

namespace auto_battlebot {

OutputChannel::OutputChannel(std::string topic, std::string message_encoding, VizSchema schema,
                             bool latch, std::shared_ptr<VizSink> sink,
                             std::shared_ptr<McapRecorder> recorder)
    : topic_(std::move(topic)), sink_(std::move(sink)), recorder_(std::move(recorder)) {
    if (sink_) sink_channel_ = sink_->advertise(topic_, message_encoding, schema, latch);
    if (recorder_) recorder_channel_ = recorder_->open_channel(topic_, message_encoding, schema);
    buffer_.resize(4096);
}

void OutputChannel::log(const std::byte* data, size_t len, uint64_t log_time_ns) {
    // The SDK refuses zero-length messages, and an empty payload carries nothing anyway.
    if (len == 0) return;
    if (recorder_ && recorder_channel_ != 0) {
        recorder_->write(recorder_channel_, data, len, log_time_ns);
    }
    if (sink_ && sink_channel_ != 0) sink_->publish(sink_channel_, data, len, log_time_ns);
}

uint32_t OutputChannel::num_subscribers() const {
    return sink_ ? sink_->num_subscribers(sink_channel_) : 0;
}

bool OutputChannel::records() const { return recorder_ && recorder_->records_topic(topic_); }

}  // namespace auto_battlebot
