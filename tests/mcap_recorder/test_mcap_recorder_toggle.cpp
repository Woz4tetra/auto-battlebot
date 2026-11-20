#include <gtest/gtest.h>
#include <spdlog/sinks/sink.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <map>
#include <mcap/reader.hpp>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "../scratch_recordings.hpp"
#include "logging/logging.hpp"
#include "mcap_recorder/mcap_recorder.hpp"

namespace auto_battlebot {
namespace {

struct FileContents {
    bool has_summary = false;
    std::map<std::string, int> count_by_topic;
    std::map<std::string, std::string> metadata;
    std::map<std::string, int> metadata_count;
};

FileContents read_file(const std::filesystem::path &path) {
    FileContents out;
    mcap::McapReader reader;
    if (!reader.open(path.string()).ok()) return out;
    out.has_summary = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok();
    for (const auto &view : reader.readMessages()) {
        ++out.count_by_topic[view.channel->topic];
    }
    reader.close();

    std::ifstream stream(path, std::ios::binary);
    mcap::FileStreamReader data_source(stream);
    mcap::TypedRecordReader record_reader(data_source, 8);
    record_reader.onMetadata = [&out](const mcap::Metadata &metadata, mcap::ByteOffset) {
        for (const auto &[key, value] : metadata.metadata) {
            out.metadata[key] = value;
            ++out.metadata_count[key];
        }
    };
    while (record_reader.next() && record_reader.status().ok()) {
    }
    return out;
}

void write_one(McapRecorder &recorder, McapRecorder::ChannelId id, uint64_t log_time_ns) {
    const std::byte payload[] = {std::byte{'{'}, std::byte{'}'}};
    recorder.write(id, payload, sizeof(payload), log_time_ns);
}

VizSchema empty_json_schema() {
    VizSchema schema;
    schema.name = "test.Empty";
    schema.encoding = "jsonschema";
    schema.data = "{}";
    return schema;
}

class HookSink : public spdlog::sinks::sink {
   public:
    explicit HookSink(std::function<void(const std::string &)> hook) : hook_(std::move(hook)) {}
    void log(const spdlog::details::log_msg &msg) override {
        hook_(std::string(msg.payload.data(), msg.payload.size()));
    }
    void flush() override {}
    void set_pattern(const std::string &) override {}
    void set_formatter(std::unique_ptr<spdlog::formatter>) override {}

   private:
    std::function<void(const std::string &)> hook_;
};

class ScopedHookLogger {
   public:
    explicit ScopedHookLogger(std::function<void(const std::string &)> hook)
        : previous_(spdlog::default_logger()) {
        spdlog::set_default_logger(
            std::make_shared<spdlog::logger>("hook", std::make_shared<HookSink>(std::move(hook))));
    }
    ~ScopedHookLogger() { spdlog::set_default_logger(previous_); }

   private:
    std::shared_ptr<spdlog::logger> previous_;
};

}  // namespace

struct McapRecorderTestAccess {
    static std::filesystem::path make_file_path(const std::filesystem::path &dir,
                                                const std::string &profile, int segment) {
        return McapRecorder::make_file_path(dir, profile, segment);
    }
    static void hold_writer(McapRecorder &recorder, bool hold) {
        {
            std::lock_guard<std::mutex> lock(recorder.mutex_);
            recorder.paused_for_test_ = hold;
        }
        recorder.work_cv_.notify_all();
    }
    static void fail_channel_writes(McapRecorder &recorder, foxglove::FoxgloveError error) {
        recorder.channel_log_ = [error](foxglove::RawChannel &, const McapRecorder::WriteOp &) {
            return error;
        };
    }
};

namespace {

class McapRecorderToggleTest : public ::testing::Test {
   protected:
    void SetUp() override { dir_ = make_scratch_recordings_dir("toggle_test"); }
    void TearDown() override {
        std::error_code ec;
        for (const auto &path : paths_) std::filesystem::remove(path, ec);
        std::filesystem::remove_all(dir_, ec);
    }
    std::filesystem::path dir_;
    std::vector<std::filesystem::path> paths_;
};

TEST_F(McapRecorderToggleTest, OffClosesTheFileAndOnStartsANewOne) {
    McapRecorder recorder("toggle_test", dir_);
    recorder.write_metadata("camera_calibration", "calibration");
    const auto early = recorder.open_channel("/early", "json", empty_json_schema());
    write_one(recorder, early, 1);
    const std::filesystem::path first = recorder.file_path();
    paths_.push_back(first);

    std::future<void> closed_first;
    ASSERT_TRUE(recorder.set_enabled(false, &closed_first));
    EXPECT_FALSE(recorder.is_enabled());
    write_one(recorder, early, 2);
    closed_first.wait();

    FileContents closed = read_file(first);
    EXPECT_TRUE(closed.has_summary) << "the file recording-off closed is not finalized";
    EXPECT_EQ(closed.count_by_topic["/early"], 1);

    ASSERT_TRUE(recorder.set_enabled(true));
    const std::filesystem::path second = recorder.file_path();
    paths_.push_back(second);
    EXPECT_NE(second, first) << "the new file reused the closed one's name";
    write_one(recorder, early, 3);
    recorder.close();

    FileContents reopened = read_file(second);
    EXPECT_TRUE(reopened.has_summary);
    EXPECT_EQ(reopened.count_by_topic["/early"], 1);
    EXPECT_EQ(reopened.metadata["camera_calibration"], "calibration");
    EXPECT_EQ(reopened.metadata["active_profile"], "toggle_test");
    EXPECT_EQ(read_file(first).count_by_topic["/early"], 1);
}

TEST_F(McapRecorderToggleTest, TheFileExistsOnceTheConstructorReturns) {
    McapRecorder recorder("toggle_ctor_test", dir_);
    paths_.push_back(recorder.file_path());
    EXPECT_TRUE(std::filesystem::exists(recorder.file_path()));
}

TEST_F(McapRecorderToggleTest, TheEnableFutureIsReadyOnceTheNewFileExists) {
    McapRecorder recorder("toggle_enable_future_test", dir_);
    paths_.push_back(recorder.file_path());
    ASSERT_TRUE(recorder.set_enabled(false));
    std::future<void> opened;
    ASSERT_TRUE(recorder.set_enabled(true, &opened));
    paths_.push_back(recorder.file_path());
    opened.wait();
    EXPECT_TRUE(std::filesystem::exists(recorder.file_path()));
}

TEST_F(McapRecorderToggleTest, ChannelOpenedWhileOffRecordsInTheNextFile) {
    McapRecorder recorder("toggle_late_test", dir_);
    ASSERT_TRUE(recorder.set_enabled(false));
    paths_.push_back(recorder.file_path());
    const auto late = recorder.open_channel("/late", "json", empty_json_schema());
    EXPECT_NE(late, 0u);
    ASSERT_TRUE(recorder.set_enabled(true));
    paths_.push_back(recorder.file_path());
    write_one(recorder, late, 1);
    recorder.close();

    EXPECT_EQ(read_file(paths_.back()).count_by_topic["/late"], 1);
}

TEST_F(McapRecorderToggleTest, TogglingWithTheMcapLogSinkDoesNotDeadlock) {
    const auto previous_logger = spdlog::default_logger();
    auto recorder = std::make_shared<McapRecorder>("toggle_log_test", dir_);
    setup_logging(recorder);
    paths_.push_back(recorder->file_path());

    recorder->set_enabled(false);
    recorder->set_enabled(true);
    spdlog::info("after reopen");
    const std::filesystem::path second = recorder->file_path();
    paths_.push_back(second);
    recorder->close();
    spdlog::set_default_logger(previous_logger);

    FileContents reopened = read_file(second);
    EXPECT_GE(reopened.count_by_topic["/log"], 1) << "log lines stopped reaching the MCAP";
}

TEST_F(McapRecorderToggleTest, LoggingFromAnotherThreadWhileTogglingDoesNotDeadlock) {
    const auto previous_logger = spdlog::default_logger();
    auto recorder = std::make_shared<McapRecorder>("toggle_abba_test", dir_);
    setup_logging(recorder);
    spdlog::default_logger()->sinks().front()->set_level(spdlog::level::off);

    auto stop = std::make_shared<std::atomic<bool>>(false);
    std::thread logger_thread([stop] {
        while (!stop->load()) spdlog::info("control loop log line");
    });

    std::vector<std::filesystem::path> paths{recorder->file_path()};
    for (int i = 0; i < 200; ++i) {
        recorder->set_enabled(false);
        recorder->set_enabled(true);
        if (paths.back() != recorder->file_path()) paths.push_back(recorder->file_path());
    }
    paths_ = paths;
    stop->store(true);
    logger_thread.join();
    spdlog::set_default_logger(previous_logger);
    recorder->close();
    EXPECT_GT(paths_.size(), 1u);
}

TEST_F(McapRecorderToggleTest, QuickTogglesEachGetTheirOwnFile) {
    McapRecorder recorder("toggle_quick_test", dir_);
    const auto id = recorder.open_channel("/topic", "json", empty_json_schema());
    std::vector<std::filesystem::path> paths{recorder.file_path()};
    write_one(recorder, id, 1);
    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(recorder.set_enabled(false));
        ASSERT_TRUE(recorder.set_enabled(true));
        paths.push_back(recorder.file_path());
        write_one(recorder, id, 2 + static_cast<uint64_t>(i));
    }
    recorder.close();
    paths_ = paths;

    std::set<std::filesystem::path> unique(paths.begin(), paths.end());
    EXPECT_EQ(unique.size(), paths.size()) << "two toggles wrote to the same file";
    for (const auto &path : paths) {
        FileContents contents = read_file(path);
        EXPECT_TRUE(contents.has_summary) << path;
        EXPECT_EQ(contents.count_by_topic["/topic"], 1) << path;
    }
}

TEST_F(McapRecorderToggleTest, WritesDoNotWaitForTheWriterThreadWhileTogglesAreQueued) {
    McapRecorder recorder("toggle_nowait_test", dir_);
    const auto id = recorder.open_channel("/topic", "json", empty_json_schema());
    std::vector<std::filesystem::path> paths{recorder.file_path()};
    constexpr size_t kPayloadBytes = 1 << 20;
    const std::vector<std::byte> payload(kPayloadBytes, std::byte{'x'});
    McapRecorderTestAccess::hold_writer(recorder, true);
    for (int i = 0; i < 10; ++i) {
        recorder.set_enabled(false);
        recorder.set_enabled(true);
        paths.push_back(recorder.file_path());
        for (int j = 0; j < 5; ++j) {
            recorder.write(id, payload.data(), payload.size(), 1 + static_cast<uint64_t>(j));
        }
    }
    EXPECT_EQ(recorder.dropped_messages(), 0u);
    McapRecorderTestAccess::hold_writer(recorder, false);
    recorder.close();
    paths_ = paths;
}

TEST_F(McapRecorderToggleTest, FlushWaitsForQueuedWrites) {
    McapRecorder recorder("toggle_flush_test", dir_);
    const auto id = recorder.open_channel("/topic", "json", empty_json_schema());
    const std::filesystem::path path = recorder.file_path();
    paths_.push_back(path);
    for (uint64_t i = 1; i <= 100; ++i) write_one(recorder, id, i);
    recorder.set_enabled(false);
    recorder.flush();
    FileContents contents = read_file(path);
    EXPECT_TRUE(contents.has_summary);
    EXPECT_EQ(contents.count_by_topic["/topic"], 100);
    recorder.close();
}

TEST_F(McapRecorderToggleTest, FileAgeRestartsWithEachFile) {
    McapRecorder recorder("toggle_test", dir_);
    paths_.push_back(recorder.file_path());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ASSERT_TRUE(recorder.file_age_s().has_value());
    EXPECT_GE(*recorder.file_age_s(), 0.2);

    ASSERT_TRUE(recorder.set_enabled(false));
    EXPECT_FALSE(recorder.file_age_s().has_value());
    ASSERT_TRUE(recorder.set_enabled(true));
    paths_.push_back(recorder.file_path());
    ASSERT_TRUE(recorder.file_age_s().has_value());
    EXPECT_LT(*recorder.file_age_s(), 0.2);
    recorder.close();
}

TEST_F(McapRecorderToggleTest, CloseIsFinal) {
    McapRecorder recorder("toggle_test", dir_);
    paths_.push_back(recorder.file_path());
    recorder.close();
    EXPECT_FALSE(recorder.set_enabled(true));
    EXPECT_EQ(recorder.file_path(), paths_.front());
}

TEST_F(McapRecorderToggleTest, FullQueueDropsVideoAndKeepsStatus) {
    McapRecorder recorder("toggle_cap_test", dir_);
    const auto video = recorder.open_channel("/camera/video", "json", empty_json_schema());
    const auto status = recorder.open_channel("/status/system", "json", empty_json_schema());
    const std::filesystem::path path = recorder.file_path();
    paths_.push_back(path);

    McapRecorderTestAccess::hold_writer(recorder, true);
    const std::vector<std::byte> frame(1 << 20, std::byte{0x11});
    const std::string small = "{}";
    constexpr int kFrames = 200;
    constexpr int kStatusPerFrame = 5;
    for (int i = 0; i < kFrames; ++i) {
        recorder.write(video, frame.data(), frame.size(), 1 + i);
        for (int j = 0; j < kStatusPerFrame; ++j) {
            recorder.write(status, reinterpret_cast<const std::byte *>(small.data()), small.size(),
                           1 + i);
        }
    }
    const uint64_t kFramesThatFitBesideStatus =
        McapRecorder::kMaxQueuedLargeBytes / frame.size() - 1;
    EXPECT_EQ(recorder.dropped_messages(), kFrames - kFramesThatFitBesideStatus);

    McapRecorderTestAccess::hold_writer(recorder, false);
    recorder.set_enabled(false);
    recorder.flush();
    FileContents contents = read_file(path);
    EXPECT_EQ(contents.count_by_topic["/camera/video"],
              static_cast<int>(kFramesThatFitBesideStatus));
    EXPECT_EQ(contents.count_by_topic["/status/system"], kFrames * kStatusPerFrame);
    recorder.close();
}

TEST_F(McapRecorderToggleTest, PastSizeLimitIsFalseWhileTheNewFileIsNotCreatedYet) {
    McapRecorder recorder("toggle_limit_test", dir_);
    recorder.set_size_limits(1, std::nullopt);
    recorder.set_enabled(false);
    recorder.flush();
    McapRecorderTestAccess::hold_writer(recorder, true);
    recorder.set_enabled(true);
    paths_.push_back(recorder.file_path());
    EXPECT_FALSE(std::filesystem::exists(recorder.file_path()));
    EXPECT_FALSE(recorder.past_size_limit());
    McapRecorderTestAccess::hold_writer(recorder, false);
    recorder.close();
}

TEST_F(McapRecorderToggleTest, DroppingAMessageNeverLogs) {
    McapRecorder recorder("toggle_drop_log_test", dir_);
    const auto video = recorder.open_channel("/camera/video", "json", empty_json_schema());
    paths_.push_back(recorder.file_path());
    recorder.flush();
    McapRecorderTestAccess::hold_writer(recorder, true);
    std::atomic<int> logged{0};
    {
        ScopedHookLogger hook([&logged](const std::string &) { ++logged; });
        const std::vector<std::byte> frame(1 << 20, std::byte{0x11});
        for (int i = 0; i < 200; ++i) recorder.write(video, frame.data(), frame.size(), 1 + i);
        EXPECT_GT(recorder.dropped_messages(), 0u);
        EXPECT_EQ(logged.load(), 0);
    }
    McapRecorderTestAccess::hold_writer(recorder, false);
    recorder.close();
}

TEST_F(McapRecorderToggleTest, QueueCapCountsBytesUntilTheBatchIsWritten) {
    McapRecorder recorder("toggle_inflight_test", dir_);
    const auto video = recorder.open_channel("/camera/video", "json", empty_json_schema());
    recorder.set_enabled(false);
    recorder.flush();
    McapRecorderTestAccess::hold_writer(recorder, true);
    recorder.set_enabled(true);
    paths_.push_back(recorder.file_path());

    const std::vector<std::byte> frame(1 << 20, std::byte{0x11});
    for (int i = 0; i < 100; ++i) recorder.write(video, frame.data(), frame.size(), 1 + i);
    ASSERT_EQ(recorder.dropped_messages(), 0u);

    std::promise<void> writer_inside_batch;
    std::promise<void> release_writer;
    std::atomic<bool> blocked{false};
    auto release = release_writer.get_future().share();
    {
        ScopedHookLogger hook([&](const std::string &message) {
            if (message.find("Recording to") == std::string::npos || blocked.exchange(true)) return;
            writer_inside_batch.set_value();
            release.wait();
        });
        auto inside = writer_inside_batch.get_future();
        McapRecorderTestAccess::hold_writer(recorder, false);
        ASSERT_EQ(inside.wait_for(std::chrono::seconds(10)), std::future_status::ready);
        for (int i = 0; i < 100; ++i) recorder.write(video, frame.data(), frame.size(), 200 + i);
        EXPECT_GT(recorder.dropped_messages(), 0u);
        release_writer.set_value();
        recorder.flush();
    }
    recorder.close();
}

TEST_F(McapRecorderToggleTest, AFailedOpenOfAnOlderFileLeavesTheNewerRecordingOn) {
    McapRecorder recorder("toggle_late_failure_test", dir_);
    recorder.set_enabled(false);
    recorder.flush();
    McapRecorderTestAccess::hold_writer(recorder, true);
    recorder.set_enabled(true);
    const std::filesystem::path doomed = recorder.file_path();
    std::filesystem::create_directories(doomed);
    paths_.push_back(doomed);
    recorder.set_enabled(false);
    recorder.set_enabled(true);
    ASSERT_NE(recorder.file_path(), doomed);
    paths_.push_back(recorder.file_path());
    McapRecorderTestAccess::hold_writer(recorder, false);
    recorder.flush();
    EXPECT_TRUE(recorder.is_enabled());
    recorder.close();
}

TEST_F(McapRecorderToggleTest, EvictsOnlyFromItsOwnDirectoryAndNeverFromAnother) {
    const std::filesystem::path other = make_scratch_recordings_dir("toggle_other");
    const std::filesystem::path sentinel = other / "sentinel.mcap";
    const std::filesystem::path stale = dir_ / "stale.mcap";
    constexpr uintmax_t kOverOneGb = 1100ULL * 1024 * 1024;
    for (const auto &path : {sentinel, stale}) {
        std::ofstream(path).close();
        std::filesystem::resize_file(path, kOverOneGb);
        std::filesystem::last_write_time(
            path, std::filesystem::file_time_type::clock::now() - std::chrono::hours(24 * 365));
    }
    McapRecorder recorder("toggle_evict_test", dir_);
    EXPECT_EQ(recorder.file_path().parent_path(), dir_);
    paths_.push_back(recorder.file_path());
    recorder.set_size_limits(std::nullopt, 1);
    recorder.set_enabled(false);
    recorder.set_enabled(true);
    paths_.push_back(recorder.file_path());
    recorder.flush();
    recorder.close();
    EXPECT_FALSE(std::filesystem::exists(stale)) << "the oldest file in the recorder's directory";
    EXPECT_TRUE(std::filesystem::exists(sentinel)) << "a file outside the recorder's directory";
    std::error_code ec;
    std::filesystem::remove_all(other, ec);
}

TEST_F(McapRecorderToggleTest, HoldingDirectoryScanSurvivesAnUnreadableEntry) {
    McapRecorder recorder("toggle_scan_test", dir_);
    const std::filesystem::path loop = recorder.file_path().parent_path() / "zz_symlink_loop.mcap";
    std::error_code ec;
    std::filesystem::create_symlink(loop.filename(), loop, ec);
    ASSERT_FALSE(ec);
    paths_.push_back(loop);
    recorder.set_size_limits(std::nullopt, 1);
    recorder.set_enabled(false);
    recorder.set_enabled(true);
    paths_.push_back(recorder.file_path());
    recorder.flush();
    EXPECT_TRUE(recorder.is_enabled());
    recorder.close();
}

TEST_F(McapRecorderToggleTest, SetEnabledRefusesAPathACameraIsReplaying) {
    McapRecorder recorder("toggle_replay_test", dir_);
    paths_.push_back(recorder.file_path());
    ASSERT_TRUE(recorder.set_enabled(false));
    recorder.flush();
    bool refused = false;
    for (int attempt = 0; attempt < 3 && !refused; ++attempt) {
        for (int part = 0; part < 10; ++part) {
            McapRecorder::reserve_input_path(
                McapRecorderTestAccess::make_file_path(dir_, "toggle_replay_test", part));
        }
        refused = !recorder.set_enabled(true);
        if (!refused) {
            paths_.push_back(recorder.file_path());
            ASSERT_TRUE(recorder.set_enabled(false));
        }
    }
    EXPECT_TRUE(refused) << "recording opened a file a camera is replaying from";
    EXPECT_FALSE(recorder.is_enabled());
}

TEST_F(McapRecorderToggleTest, PartSuffixSkipsPathsTheWriterHasNotCreatedYet) {
    McapRecorder recorder("toggle_part_test", dir_);
    paths_.push_back(recorder.file_path());
    recorder.flush();
    McapRecorderTestAccess::hold_writer(recorder, true);
    std::set<std::filesystem::path> unique{recorder.file_path()};
    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(recorder.set_enabled(false));
        ASSERT_TRUE(recorder.set_enabled(true));
        EXPECT_TRUE(unique.insert(recorder.file_path()).second)
            << "a path was handed out twice before the writer created the first";
    }
    McapRecorderTestAccess::hold_writer(recorder, false);
    recorder.close();
    paths_.insert(paths_.end(), unique.begin(), unique.end());
}

TEST_F(McapRecorderToggleTest, MetadataAddedAfterToggleIsWrittenOnce) {
    McapRecorder recorder("toggle_metadata_test", dir_);
    paths_.push_back(recorder.file_path());
    recorder.flush();
    ASSERT_TRUE(recorder.set_enabled(false));
    recorder.write_metadata("before_toggle", "a");
    McapRecorderTestAccess::hold_writer(recorder, true);
    ASSERT_TRUE(recorder.set_enabled(true));
    const std::filesystem::path second = recorder.file_path();
    paths_.push_back(second);
    recorder.write_metadata("after_toggle", "b");
    McapRecorderTestAccess::hold_writer(recorder, false);
    recorder.close();

    FileContents contents = read_file(second);
    EXPECT_EQ(contents.metadata_count["before_toggle"], 1);
    EXPECT_EQ(contents.metadata_count["after_toggle"], 1);
}

TEST_F(McapRecorderToggleTest, FailedChannelLogsOncePerFile) {
    McapRecorder recorder("toggle_bad_channel_test", dir_);
    paths_.push_back(recorder.file_path());
    const auto bad = recorder.open_channel("/bad\xff", "json", empty_json_schema());
    recorder.flush();
    std::atomic<int> logged{0};
    {
        ScopedHookLogger hook([&logged](const std::string &message) {
            if (message.find("Failed to open channel") != std::string::npos) ++logged;
        });
        for (uint64_t i = 1; i <= 20; ++i) write_one(recorder, bad, i);
        recorder.flush();
    }
    EXPECT_EQ(logged.load(), 1) << "a channel that cannot be created logged once per message";
    recorder.close();
}

TEST_F(McapRecorderToggleTest, WriteFailureLogsOncePerFile) {
    McapRecorder recorder("toggle_write_failure_test", dir_);
    paths_.push_back(recorder.file_path());
    const auto channel = recorder.open_channel("/failing", "json", empty_json_schema());
    recorder.flush();
    McapRecorderTestAccess::fail_channel_writes(recorder, foxglove::FoxgloveError::IoError);
    std::atomic<int> logged{0};
    ScopedHookLogger hook([&logged](const std::string &message) {
        if (message.find("Write to") != std::string::npos) ++logged;
    });

    for (uint64_t i = 1; i <= 20; ++i) write_one(recorder, channel, i);
    recorder.flush();
    EXPECT_EQ(logged.load(), 1) << "a disk error logged once per message";

    ASSERT_TRUE(recorder.set_enabled(false));
    ASSERT_TRUE(recorder.set_enabled(true));
    paths_.push_back(recorder.file_path());
    for (uint64_t i = 21; i <= 40; ++i) write_one(recorder, channel, i);
    recorder.flush();
    EXPECT_EQ(logged.load(), 2) << "the second file did not log its own first failure";
    recorder.close();
}

}  // namespace
}  // namespace auto_battlebot
