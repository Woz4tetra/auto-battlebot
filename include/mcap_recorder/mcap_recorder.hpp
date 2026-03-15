#pragma once

#include <miniros/rostime.h>
#include <miniros/serialization.h>
#include <miniros/traits/message_traits.h>

#include <cstring>
#include <filesystem>
#include <mcap/writer.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace auto_battlebot {

class McapRecorder {
   public:
    explicit McapRecorder(const std::string& config_name);
    ~McapRecorder();

    McapRecorder(const McapRecorder&) = delete;
    McapRecorder& operator=(const McapRecorder&) = delete;

    void set_ignored_topics(const std::vector<std::string>& topics) {
        std::lock_guard<std::mutex> lock(mutex_);
        ignored_topics_ = std::unordered_set<std::string>(topics.begin(), topics.end());
    }

    // Write a ROS message to the MCAP file using current time
    template <typename T>
    void write(const std::string& topic, const T& msg) {
        uint64_t time_ns = miniros::Time::now().toNSec();
        write(topic, msg, time_ns);
    }

    // Write a ROS message to the MCAP file with explicit timestamp
    template <typename T>
    void write(const std::string& topic, const T& msg, uint64_t time_ns) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!writer_open_) return;
        if (ignored_topics_.count(topic)) return;

        mcap::ChannelId channel_id = get_or_create_channel<T>(topic);

        uint32_t serialized_len = miniros::serialization::serializationLength(msg);
        std::vector<uint8_t> buf(serialized_len);
        miniros::serialization::OStream ostream(buf.data(), serialized_len);
        miniros::serialization::serialize(ostream, msg);

        mcap::Message mcap_msg;
        mcap_msg.channelId = channel_id;
        mcap_msg.sequence = sequence_++;
        mcap_msg.logTime = time_ns;
        mcap_msg.publishTime = time_ns;
        mcap_msg.data = reinterpret_cast<const std::byte*>(buf.data());
        mcap_msg.dataSize = serialized_len;

        auto status = writer_.write(mcap_msg);
        if (!status.ok()) {
            // Avoid recursive logging here; write to stderr directly
            std::fprintf(stderr, "[McapRecorder] write failed: %s\n", status.message.c_str());
        }
    }

    std::filesystem::path file_path() const { return file_path_; }

   private:
    template <typename T>
    mcap::ChannelId get_or_create_channel(const std::string& topic) {
        auto it = channel_ids_.find(topic);
        if (it != channel_ids_.end()) {
            return it->second;
        }

        // Register schema
        const char* datatype = miniros::message_traits::DataType<T>::value();
        const char* definition = miniros::message_traits::Definition<T>::value();

        mcap::Schema schema;
        schema.name = datatype;
        schema.encoding = "ros1msg";
        const auto* def_bytes = reinterpret_cast<const std::byte*>(definition);
        schema.data = mcap::ByteArray(def_bytes, def_bytes + std::strlen(definition));
        writer_.addSchema(schema);

        // Register channel
        mcap::Channel channel;
        channel.topic = topic;
        channel.messageEncoding = "ros1";
        channel.schemaId = schema.id;
        // Include connection header metadata that Foxglove expects
        channel.metadata["callerid"] = "auto_battlebot";
        channel.metadata["md5sum"] = miniros::message_traits::MD5Sum<T>::value();
        channel.metadata["type"] = datatype;
        channel.metadata["message_definition"] = definition;
        writer_.addChannel(channel);

        channel_ids_[topic] = channel.id;
        return channel.id;
    }

    static std::filesystem::path make_file_path(const std::string& config_name);

    mcap::McapWriter writer_;
    std::filesystem::path file_path_;
    bool writer_open_{false};
    std::mutex mutex_;
    std::unordered_map<std::string, mcap::ChannelId> channel_ids_;
    std::unordered_set<std::string> ignored_topics_;
    uint32_t sequence_{0};
};

}  // namespace auto_battlebot
