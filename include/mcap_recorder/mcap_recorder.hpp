#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/mcap.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include "mcap_recorder/config.hpp"
#include "viz/schema.hpp"

namespace auto_battlebot {

class McapRecorder {
   public:
    using ChannelId = uint32_t;

    static constexpr size_t kLargeMessageBytes = 64 * 1024;
    static constexpr size_t kMaxQueuedLargeBytes = 128ULL * 1024 * 1024;
    static constexpr size_t kMaxQueuedBytes = 192ULL * 1024 * 1024;

    explicit McapRecorder(const std::string& active_profile);
    ~McapRecorder();

    McapRecorder(const McapRecorder&) = delete;
    McapRecorder& operator=(const McapRecorder&) = delete;

    /** Zero for either disables that limit. */
    void set_size_limits(uint64_t max_size_gb, uint64_t holding_dir_max_size_gb);
    void set_ignored_topics(const std::vector<std::string>& topics);
    /** Off queues closing the current file, which is then complete and safe to download while
     *  the app runs; flush() waits for that. On picks a new timestamped file and queues opening
     *  it with every channel and the metadata re-declared. False after close(), or when the new
     *  path is one a camera is replaying from. A file that fails to open turns recording off and
     *  is logged by the writer thread. */
    bool set_enabled(bool enabled);
    bool is_enabled() const;
    bool records_topic(const std::string& topic) const;
    /** Stops recording, writes out everything queued, and joins the writer thread. Final: a late
     *  UI toggle after shutdown cannot start a new file. */
    void close();
    void flush();
    bool past_size_limit() const;

    /** Register a topic. Idempotent per topic. The id stays valid across files: a topic
     *  registered while recording is off gets its channel when the next file opens. */
    ChannelId open_channel(const std::string& topic, const std::string& message_encoding,
                           const VizSchema& schema);

    void write(ChannelId channel, const std::byte* data, size_t len, uint64_t log_time_ns);

    void write_metadata(const std::string& key, const std::string& value);

    /** Refuse to record over a file something is reading. With video in the main MCAP, a replay
     *  with recording on reads file A and writes file B, and pointing the output at the input
     *  would corrupt the source. The guard belongs here rather than in config, where it would be
     *  one merge away from being wrong. */
    static void reserve_input_path(const std::filesystem::path& path);

    std::filesystem::path file_path() const;

    /** Messages write() refused because the queue was at its cap. */
    uint64_t dropped_messages() const;

    std::optional<double> file_age_s() const;

   private:
    friend struct McapRecorderTestAccess;

    struct OpenOp {
        std::filesystem::path path;
        /** Snapshot at the toggle, so a key added after it arrives once, as its own MetadataOp. */
        std::vector<std::pair<std::string, std::string>> metadata;
    };
    struct CloseOp {};
    struct WriteOp {
        ChannelId channel;
        uint64_t log_time_ns;
        std::vector<std::byte> data;
    };
    struct MetadataOp {
        std::string key;
        std::string value;
    };
    using Op = std::variant<OpenOp, CloseOp, WriteOp, MetadataOp>;

    struct ChannelSpec {
        std::string topic;
        std::string message_encoding;
        VizSchema schema;
    };

    static std::filesystem::path make_file_path(const std::string& active_profile, int segment);
    static bool is_reserved(const std::filesystem::path& path);
    /** Queue opening a new file. False, with the path in `refused`, when a camera is replaying
     *  from that path. Caller holds mutex_. */
    bool start_file_locked(std::filesystem::path& refused);
    /** make_file_path() with the current time, suffixed _partNN past any file already there or
     *  already handed out, since the writer may not have created the last one yet. */
    std::filesystem::path next_file_path_locked();
    void push_locked(Op op, size_t bytes);

    void writer_loop();
    void handle(OpenOp& op);
    void handle(CloseOp& op);
    void handle(WriteOp& op);
    void handle(MetadataOp& op);
    void close_file();
    foxglove::RawChannel* channel_for(ChannelId id);
    void enforce_holding_dir_size(const std::filesystem::path& directory);

    std::string active_profile_;

    // Shared, guarded by mutex_.
    mutable std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable written_cv_;
    std::deque<Op> queue_;
    size_t queued_bytes_ = 0;
    uint64_t queued_seq_ = 0;
    uint64_t written_seq_ = 0;
    uint64_t dropped_messages_ = 0;
    bool stop_ = false;
    bool enabled_ = false;
    std::atomic<bool> enabled_hint_{false};
    bool closed_for_good_ = false;
    bool paused_for_test_ = false;
    std::filesystem::path file_path_;
    std::chrono::steady_clock::time_point file_started_{};
    std::unordered_set<std::string> issued_paths_;
    std::vector<ChannelSpec> channel_specs_;
    std::unordered_map<std::string, ChannelId> channel_by_topic_;
    std::unordered_set<std::string> ignored_topics_;
    std::vector<std::pair<std::string, std::string>> metadata_;
    uint64_t max_size_bytes_ = 0;
    uint64_t holding_dir_max_size_bytes_ = 0;

    // Writer thread only.
    foxglove::Context context_;
    std::optional<foxglove::McapWriter> writer_;
    std::filesystem::path open_path_;
    std::vector<std::unique_ptr<foxglove::RawChannel>> channels_;
    /** Channels that failed to create in this file, so a bad schema is logged once, not per
     *  message. */
    std::vector<bool> channel_failed_;
    uint64_t write_failures_ = 0;
    uint64_t dropped_reported_ = 0;

    std::mutex join_mutex_;
    std::thread writer_thread_;
};

std::shared_ptr<McapRecorder> make_mcap_recorder(const McapRecorderConfig& config,
                                                 const std::string& active_profile);

}  // namespace auto_battlebot
