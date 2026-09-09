#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/mcap.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mcap_recorder/config.hpp"
#include "viz/schema.hpp"

namespace auto_battlebot {

/**
 * Writes already-encoded messages to an MCAP file with the Foxglove SDK writer.
 *
 * Callers open a channel once per topic and then write bytes against it. The recorder is the
 * single gate on what lands in the file: `enabled_` (toggled from the UI) and the configured
 * ignored topics. Producers can ask `records_topic()` before doing expensive encoding.
 */
class McapRecorder {
   public:
    using ChannelId = uint32_t;

    // `active_profile` is the resolved config profile id; it is embedded in the recording
    // filename and written into the file as an `active_profile` metadata record.
    explicit McapRecorder(const std::string& active_profile);
    ~McapRecorder();

    McapRecorder(const McapRecorder&) = delete;
    McapRecorder& operator=(const McapRecorder&) = delete;

    /** Zero for either disables that limit. */
    void set_size_limits(uint64_t max_size_gb, uint64_t holding_dir_max_size_gb) {
        std::lock_guard<std::mutex> lock(mutex_);
        constexpr uint64_t kBytesPerGb = 1024ULL * 1024ULL * 1024ULL;
        max_size_bytes_ = max_size_gb * kBytesPerGb;
        holding_dir_max_size_bytes_ = holding_dir_max_size_gb * kBytesPerGb;
    }

    void set_ignored_topics(const std::vector<std::string>& topics) {
        std::lock_guard<std::mutex> lock(mutex_);
        ignored_topics_ = std::unordered_set<std::string>(topics.begin(), topics.end());
    }
    bool set_enabled(bool enabled);
    bool is_enabled() const;
    // True when a write() on the topic would actually be recorded. Lets producers skip
    // expensive serialization for topics nobody records.
    bool records_topic(const std::string& topic) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return records_topic_locked(topic);
    }
    void close();

    /** Register a topic. Idempotent per topic; returns 0 when the writer is not open. */
    ChannelId open_channel(const std::string& topic, const std::string& message_encoding,
                           const VizSchema& schema);

    void write(ChannelId channel, const std::byte* data, size_t len, uint64_t log_time_ns);

    /** File-level metadata, alongside `active_profile`. Used for `calibration_id`, so a recording
     *  says which calibration rectified its frames and a revised one can be applied later. */
    void write_metadata(const std::string& key, const std::string& value);

    /** Refuse to record over a file something is reading. With video in the main MCAP, a replay
     *  with recording on reads file A and writes file B, and pointing the output at the input
     *  would corrupt the source. The guard belongs here rather than in config, where it would be
     *  one merge away from being wrong. */
    static void reserve_input_path(const std::filesystem::path& path);

    const std::filesystem::path& file_path() const { return file_path_; }

   private:
    struct Channel {
        std::string topic;
        std::unique_ptr<foxglove::RawChannel> channel;
    };

    /** What each channel was opened with, so rollover can re-create it against the new writer. */
    struct ChannelSpec {
        std::string topic;
        std::string message_encoding;
        VizSchema schema;
    };

    bool records_topic_locked(const std::string& topic) const {
        return writer_open_ && enabled_ && ignored_topics_.count(topic) == 0;
    }
    static std::filesystem::path make_file_path(const std::string& active_profile, int segment);
    ChannelId open_channel_locked(const std::string& topic, const std::string& message_encoding,
                                  const VizSchema& schema);
    bool open_writer(const std::filesystem::path& path);
    /** Close the current segment and open the next, re-creating every channel against the new
     *  writer so each segment is independently readable. Caller holds mutex_. */
    void roll_over_locked();
    void check_rollover_locked();

    foxglove::Context context_;
    std::optional<foxglove::McapWriter> writer_;
    std::string active_profile_;
    std::filesystem::path file_path_;
    bool writer_open_{false};
    bool enabled_{false};
    mutable std::mutex mutex_;
    std::vector<Channel> channels_;
    std::vector<ChannelSpec> channel_specs_;
    std::unordered_map<std::string, ChannelId> channel_by_topic_;
    std::unordered_set<std::string> ignored_topics_;

    // Rollover. The MCAP has become the large file now that video lives in it, so the size cap,
    // holding-directory eviction and segment numbering that SvoRecorder owned belong here. One run
    // writes one file, so this only matters for a run left going for hours.
    uint64_t max_size_bytes_ = 0;
    uint64_t holding_dir_max_size_bytes_ = 0;
    uint64_t writes_since_size_check_ = 0;
    int segment_ = 0;
    std::vector<std::pair<std::string, std::string>> metadata_;

    void enforce_holding_dir_size_locked();
};

std::shared_ptr<McapRecorder> make_mcap_recorder(const McapRecorderConfig& config,
                                                 const std::string& active_profile);

}  // namespace auto_battlebot
