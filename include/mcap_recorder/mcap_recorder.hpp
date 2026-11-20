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
#include <functional>
#include <future>
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

    McapRecorder(const std::string& active_profile, std::filesystem::path recordings_dir);
    ~McapRecorder();

    McapRecorder(const McapRecorder&) = delete;
    McapRecorder& operator=(const McapRecorder&) = delete;

    void set_size_limits(std::optional<uint64_t> max_size_gb,
                         std::optional<uint64_t> holding_dir_max_size_gb);
    void set_ignored_topics(const std::vector<std::string>& topics);
    bool set_enabled(bool enabled, std::future<void>* completed = nullptr);
    bool is_enabled() const;
    bool records_topic(const std::string& topic) const;
    void close();
    void flush();
    bool past_size_limit() const;

    ChannelId open_channel(const std::string& topic, const std::string& message_encoding,
                           const VizSchema& schema);

    void write(ChannelId channel, const std::byte* data, size_t len, uint64_t log_time_ns);

    void write_metadata(const std::string& key, const std::string& value);

    static void reserve_input_path(const std::filesystem::path& path);

    std::filesystem::path file_path() const;
    uint64_t generation() const { return generation_.load(); }

    uint64_t dropped_messages() const;

    std::optional<double> file_age_s() const;

   private:
    friend struct McapRecorderTestAccess;

    struct OpenOp {
        std::filesystem::path path;
        std::vector<std::pair<std::string, std::string>> metadata;
        std::shared_ptr<std::promise<void>> done;
    };
    struct CloseOp {
        std::shared_ptr<std::promise<void>> done;
    };
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
    using ChannelLogFn =
        std::function<foxglove::FoxgloveError(foxglove::RawChannel&, const WriteOp&)>;

    static foxglove::FoxgloveError log_to_channel(foxglove::RawChannel& channel, const WriteOp& op);

    struct ChannelSpec {
        std::string topic;
        std::string message_encoding;
        VizSchema schema;
    };

    static std::filesystem::path make_file_path(const std::filesystem::path& recordings_dir,
                                                const std::string& active_profile, int segment);
    static bool is_reserved(const std::filesystem::path& path);
    bool start_file_locked(std::filesystem::path& refused, std::future<void>* completed = nullptr);
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
    std::filesystem::path recordings_dir_;

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
    std::atomic<uint64_t> generation_{0};
    bool closed_for_good_ = false;
    bool paused_for_test_ = false;
    ChannelLogFn channel_log_ = &McapRecorder::log_to_channel;
    std::filesystem::path file_path_;
    std::chrono::steady_clock::time_point file_started_{};
    std::unordered_set<std::string> issued_paths_;
    std::vector<ChannelSpec> channel_specs_;
    std::unordered_map<std::string, ChannelId> channel_by_topic_;
    std::unordered_set<std::string> ignored_topics_;
    std::vector<std::pair<std::string, std::string>> metadata_;
    std::optional<uint64_t> max_size_bytes_;
    std::optional<uint64_t> holding_dir_max_size_bytes_;

    struct WriterThreadState {
        foxglove::Context context;
        std::optional<foxglove::McapWriter> writer;
        std::filesystem::path open_path;
        std::vector<std::unique_ptr<foxglove::RawChannel>> channels;
        std::vector<bool> channel_failed;
        uint64_t write_failures = 0;
        uint64_t dropped_reported = 0;
    };
    WriterThreadState writer_state_;

    std::mutex join_mutex_;
    std::thread writer_thread_;
};

std::shared_ptr<McapRecorder> make_mcap_recorder(const McapRecorderConfig& config,
                                                 const std::string& active_profile);

}  // namespace auto_battlebot
