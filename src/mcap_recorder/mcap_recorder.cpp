#include "mcap_recorder/mcap_recorder.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <foxglove/error.hpp>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

namespace auto_battlebot {
namespace {

std::mutex& reserved_mutex() {
    static std::mutex mutex;
    return mutex;
}
std::unordered_set<std::string>& reserved_paths() {
    static std::unordered_set<std::string> paths;
    return paths;
}

std::string reservation_key(const std::filesystem::path& path) {
    std::error_code ec;
    const std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
    return (ec ? path : canonical).string();
}

/** Drops are reported at most this often, so a disk that stays behind does not flood the log. */
constexpr auto kDropReportInterval = std::chrono::seconds(1);
}  // namespace

void McapRecorder::reserve_input_path(const std::filesystem::path& path) {
    const std::string key = reservation_key(path);
    std::lock_guard<std::mutex> lock(reserved_mutex());
    reserved_paths().insert(key);
}

bool McapRecorder::is_reserved(const std::filesystem::path& path) {
    const std::string key = reservation_key(path);
    std::lock_guard<std::mutex> lock(reserved_mutex());
    return reserved_paths().count(key) != 0;
}

McapRecorder::McapRecorder(const std::string& active_profile) : active_profile_(active_profile) {
    std::filesystem::path refused;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        metadata_.emplace_back("active_profile", active_profile_);
        start_file_locked(refused);
    }
    if (!refused.empty()) {
        spdlog::error("[McapRecorder] Refusing to record over {}, which a camera is replaying from",
                      refused.string());
    }
    writer_thread_ = std::thread(&McapRecorder::writer_loop, this);
}

McapRecorder::~McapRecorder() { close(); }

void McapRecorder::set_size_limits(uint64_t max_size_gb, uint64_t holding_dir_max_size_gb) {
    constexpr uint64_t kBytesPerGb = 1024ULL * 1024ULL * 1024ULL;
    std::lock_guard<std::mutex> lock(mutex_);
    max_size_bytes_ = max_size_gb * kBytesPerGb;
    holding_dir_max_size_bytes_ = holding_dir_max_size_gb * kBytesPerGb;
}

void McapRecorder::set_ignored_topics(const std::vector<std::string>& topics) {
    std::lock_guard<std::mutex> lock(mutex_);
    ignored_topics_ = std::unordered_set<std::string>(topics.begin(), topics.end());
}

std::filesystem::path McapRecorder::next_file_path_locked() {
    std::filesystem::path path = make_file_path(active_profile_, 0);
    for (int part = 1; std::filesystem::exists(path) || issued_paths_.count(path.string()) != 0;
         ++part) {
        path = make_file_path(active_profile_, part);
    }
    issued_paths_.insert(path.string());
    return path;
}

bool McapRecorder::start_file_locked(std::filesystem::path& refused) {
    std::filesystem::path path = next_file_path_locked();
    if (is_reserved(path)) {
        refused = std::move(path);
        return false;
    }
    file_path_ = path;
    file_started_ = std::chrono::steady_clock::now();
    push_locked(OpenOp{std::move(path), metadata_}, 0);
    enabled_ = true;
    enabled_hint_.store(true);
    return true;
}

void McapRecorder::push_locked(Op op, size_t bytes) {
    // After close() the writer thread is draining for the last time; nothing new may land.
    if (stop_) return;
    queue_.push_back(std::move(op));
    queued_bytes_ += bytes;
    ++queued_seq_;
}

bool McapRecorder::set_enabled(bool enabled) {
    std::filesystem::path refused;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled) {
            if (enabled_) {
                enabled_ = false;
                enabled_hint_.store(false);
                push_locked(CloseOp{}, 0);
            }
        } else if (closed_for_good_) {
            return false;
        } else if (!enabled_) {
            start_file_locked(refused);
        }
    }
    work_cv_.notify_one();
    if (refused.empty()) return true;
    spdlog::error("[McapRecorder] Refusing to record over {}, which a camera is replaying from",
                  refused.string());
    return false;
}

bool McapRecorder::is_enabled() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

bool McapRecorder::records_topic(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_ && ignored_topics_.count(topic) == 0;
}

std::optional<double> McapRecorder::file_age_s() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) return std::nullopt;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - file_started_).count();
}

std::filesystem::path McapRecorder::file_path() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_path_;
}

bool McapRecorder::past_size_limit() const {
    std::filesystem::path path;
    uint64_t max_size_bytes = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (max_size_bytes_ == 0 || !enabled_) return false;
        path = file_path_;
        max_size_bytes = max_size_bytes_;
    }
    // Outside the lock: a stat is I/O. The writer thread may not have created a new file yet, so
    // a missing file reads as under the limit.
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    return !ec && size >= max_size_bytes;
}

void McapRecorder::close() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_for_good_ = true;
        if (enabled_) {
            enabled_ = false;
            enabled_hint_.store(false);
            push_locked(CloseOp{}, 0);
        }
        stop_ = true;
    }
    work_cv_.notify_all();
    std::lock_guard<std::mutex> join_lock(join_mutex_);
    if (writer_thread_.joinable()) writer_thread_.join();
}

void McapRecorder::flush() {
    std::unique_lock<std::mutex> lock(mutex_);
    const uint64_t target = queued_seq_;
    written_cv_.wait(lock, [this, target] { return written_seq_ >= target; });
}

McapRecorder::ChannelId McapRecorder::open_channel(const std::string& topic,
                                                   const std::string& message_encoding,
                                                   const VizSchema& schema) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = channel_by_topic_.find(topic); it != channel_by_topic_.end()) {
        return it->second;
    }
    // The writer thread creates the channel in the current file on its first message, and in
    // every later file from this spec.
    channel_specs_.push_back(ChannelSpec{topic, message_encoding, schema});
    const ChannelId id = static_cast<ChannelId>(channel_specs_.size());
    channel_by_topic_[topic] = id;
    return id;
}

void McapRecorder::write(ChannelId channel, const std::byte* data, size_t len,
                         uint64_t log_time_ns) {
    if (len == 0 || !enabled_hint_.load(std::memory_order_relaxed)) return;
    // Copied before taking the lock, so the lock is held only for the push.
    WriteOp op{channel, log_time_ns, std::vector<std::byte>(data, data + len)};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_ || channel == 0 || channel > channel_specs_.size()) return;
        if (ignored_topics_.count(channel_specs_[channel - 1].topic) != 0) return;
        const size_t cap = len > kLargeMessageBytes ? kMaxQueuedLargeBytes : kMaxQueuedBytes;
        if (queued_bytes_ + len > cap) {
            // Counted here and reported by the writer thread. write() runs inside the log sink,
            // so it must never log.
            ++dropped_messages_;
            return;
        }
        push_locked(std::move(op), len);
    }
    work_cv_.notify_one();
}

uint64_t McapRecorder::dropped_messages() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_messages_;
}

void McapRecorder::write_metadata(const std::string& key, const std::string& value) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& entry : metadata_) {
            if (entry.first == key) {
                entry.second = value;
                return;
            }
        }
        metadata_.emplace_back(key, value);
        if (!enabled_) return;
        push_locked(MetadataOp{key, value}, 0);
    }
    work_cv_.notify_one();
}

void McapRecorder::writer_loop() {
    auto last_drop_report = std::chrono::steady_clock::time_point{};
    while (true) {
        std::deque<Op> batch;
        size_t batch_bytes = 0;
        uint64_t batch_seq = 0;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            work_cv_.wait(lock, [this] { return stop_ || (!queue_.empty() && !writer_held_); });
            if (queue_.empty()) break;
            batch.swap(queue_);
            // queued_bytes_ keeps counting the batch until it is written, so the cap bounds
            // everything in memory, not only what is waiting.
            batch_bytes = queued_bytes_;
            batch_seq = queued_seq_;
        }
        for (Op& op : batch) {
            std::visit([this](auto& typed) { handle(typed); }, op);
        }
        uint64_t dropped = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queued_bytes_ -= batch_bytes;
            written_seq_ = batch_seq;
            dropped = dropped_messages_;
        }
        written_cv_.notify_all();

        const auto now = std::chrono::steady_clock::now();
        if (dropped > dropped_reported_ && now - last_drop_report >= kDropReportInterval) {
            spdlog::warn("[McapRecorder] Write queue full: dropped {} messages ({} total)",
                         dropped - dropped_reported_, dropped);
            dropped_reported_ = dropped;
            last_drop_report = now;
        }
    }
    if (writer_) close_file();
}

void McapRecorder::handle(OpenOp& op) {
    if (writer_) close_file();
    enforce_holding_dir_size(op.path.parent_path());

    std::error_code ec;
    std::filesystem::create_directories(op.path.parent_path(), ec);
    context_ = foxglove::Context::create();
    const std::string path_string = op.path.string();
    foxglove::McapWriterOptions opts;
    opts.context = context_;
    opts.path = path_string;
    // No compression: the Jetson writes this at 60 Hz next to the control loop.
    opts.compression = foxglove::McapCompression::None;
    auto writer = foxglove::McapWriter::create(opts);
    if (!writer.has_value()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Only if no later toggle has already moved recording on to another file.
            if (file_path_ == op.path && enabled_) {
                enabled_ = false;
                enabled_hint_.store(false);
            }
        }
        spdlog::error("[McapRecorder] Failed to open {}: {}", path_string,
                      foxglove::strerror(writer.error()));
        return;
    }
    writer_.emplace(std::move(writer.value()));
    open_path_ = op.path;
    channels_.clear();
    channel_failed_.clear();
    write_failures_ = 0;

    auto status = writer_->writeMetadata("auto_battlebot", op.metadata.begin(), op.metadata.end());
    if (status != foxglove::FoxgloveError::Ok) {
        spdlog::warn("[McapRecorder] Failed to write metadata: {}", foxglove::strerror(status));
    }
    spdlog::info("[McapRecorder] Recording to {} (profile: {})", path_string,
                 active_profile_.empty() ? "<none>" : active_profile_);
}

void McapRecorder::handle(CloseOp& /*op*/) {
    if (writer_) close_file();
}

void McapRecorder::handle(WriteOp& op) {
    if (!writer_) return;
    foxglove::RawChannel* channel = channel_for(op.channel);
    if (!channel) return;
    auto status = channel->log(op.data.data(), op.data.size(), op.log_time_ns);
    if (status == foxglove::FoxgloveError::Ok) return;
    // Logged once per file and counted after. A disk error repeats every message, and each log
    // line is itself a /log message headed for the same failing file.
    if (write_failures_++ == 0) {
        spdlog::warn("[McapRecorder] Write to {} failed: {}. Counting further failures.",
                     open_path_.string(), foxglove::strerror(status));
    }
}

void McapRecorder::handle(MetadataOp& op) {
    if (!writer_) return;
    std::vector<std::pair<std::string, std::string>> single = {{op.key, op.value}};
    auto status = writer_->writeMetadata("auto_battlebot", single.begin(), single.end());
    if (status != foxglove::FoxgloveError::Ok) {
        spdlog::warn("[McapRecorder] Failed to write {} metadata: {}", op.key,
                     foxglove::strerror(status));
    }
}

foxglove::RawChannel* McapRecorder::channel_for(ChannelId id) {
    if (id == 0) return nullptr;
    if (channels_.size() < id) {
        channels_.resize(id);
        channel_failed_.resize(id, false);
    }
    if (channels_[id - 1]) return channels_[id - 1].get();
    if (channel_failed_[id - 1]) return nullptr;

    ChannelSpec spec;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (id > channel_specs_.size()) return nullptr;
        spec = channel_specs_[id - 1];
    }
    std::optional<foxglove::Schema> sdk_schema;
    if (!spec.schema.name.empty()) sdk_schema = spec.schema.to_sdk();
    auto created =
        foxglove::RawChannel::create(spec.topic, spec.message_encoding, sdk_schema, context_);
    if (!created.has_value()) {
        channel_failed_[id - 1] = true;
        spdlog::error("[McapRecorder] Failed to open channel {}: {}", spec.topic,
                      foxglove::strerror(created.error()));
        return nullptr;
    }
    channels_[id - 1] = std::make_unique<foxglove::RawChannel>(std::move(created.value()));
    return channels_[id - 1].get();
}

void McapRecorder::close_file() {
    for (auto& channel : channels_) {
        if (channel) channel->close();
    }
    channels_.clear();
    channel_failed_.clear();
    auto status = writer_->close();
    writer_.reset();
    if (status != foxglove::FoxgloveError::Ok) {
        spdlog::warn("[McapRecorder] Closing {}: {}", open_path_.string(),
                     foxglove::strerror(status));
    }
    if (write_failures_ > 0) {
        spdlog::warn("[McapRecorder] {} writes to {} failed", write_failures_, open_path_.string());
    }
    spdlog::info("[McapRecorder] Closed {}", open_path_.string());
}

void McapRecorder::enforce_holding_dir_size(const std::filesystem::path& directory) {
    uint64_t limit = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        limit = holding_dir_max_size_bytes_;
    }
    if (limit == 0) return;
    std::error_code ec;
    if (!std::filesystem::exists(directory, ec)) return;

    // error_code overloads throughout: this runs on the writer thread, where an exception from a
    // file vanishing mid-scan would terminate the app.
    struct Entry {
        std::filesystem::path path;
        std::filesystem::file_time_type mtime;
        uintmax_t size;
    };
    std::vector<Entry> files;
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
        std::error_code entry_ec;
        if (!entry.is_regular_file(entry_ec) || entry.path().extension() != ".mcap") continue;
        const auto mtime = std::filesystem::last_write_time(entry.path(), entry_ec);
        const auto size = std::filesystem::file_size(entry.path(), entry_ec);
        if (entry_ec) continue;
        files.push_back(Entry{entry.path(), mtime, size});
    }
    std::sort(files.begin(), files.end(),
              [](const Entry& a, const Entry& b) { return a.mtime < b.mtime; });

    uintmax_t total = 0;
    for (const auto& file : files) total += file.size;
    for (const auto& file : files) {
        if (total <= limit) break;
        std::error_code remove_ec;
        if (std::filesystem::remove(file.path, remove_ec)) {
            spdlog::info("[McapRecorder] Deleted the oldest recording to free space: {}",
                         file.path.string());
            total -= file.size;
        }
    }
}

std::shared_ptr<McapRecorder> make_mcap_recorder(const McapRecorderConfig& config,
                                                 const std::string& active_profile) {
    if (!config.enable) return nullptr;
    auto recorder = std::make_shared<McapRecorder>(active_profile);
    recorder->set_ignored_topics(config.ignored_topics);
    recorder->set_size_limits(config.max_size_gb, config.holding_dir_max_size_gb);
    return recorder;
}

std::filesystem::path McapRecorder::make_file_path(const std::string& active_profile, int segment) {
    std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);

    std::string profile_slug = active_profile.empty() ? "unknown" : active_profile;
    for (char& c : profile_slug) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '.' && c != '-') c = '_';
    }

    std::ostringstream oss;
    oss << "auto_battlebot_" << profile_slug << "_";
    oss << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S");
    if (segment > 0) {
        oss << "_part" << std::setfill('0') << std::setw(2) << segment;
    }
    oss << ".mcap";

    return std::filesystem::current_path() / "data" / "recordings" / oss.str();
}

}  // namespace auto_battlebot
