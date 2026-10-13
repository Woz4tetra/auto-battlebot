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
#include <future>
#include <map>
#include <mcap/reader.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "logging/logging.hpp"
#include "mcap_recorder/mcap_recorder.hpp"

namespace auto_battlebot {
namespace {

struct FileContents {
    bool has_summary = false;
    std::map<std::string, int> count_by_topic;
    std::map<std::string, std::string> metadata;
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
        for (const auto &[key, value] : metadata.metadata) out.metadata[key] = value;
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

class McapRecorderToggleTest : public ::testing::Test {
   protected:
    void TearDown() override {
        std::error_code ec;
        for (const auto &path : paths_) std::filesystem::remove(path, ec);
    }
    std::vector<std::filesystem::path> paths_;
};

// Turning recording off finishes the file (summary written, readable while the app runs); turning
// it back on starts a new one in which the channel ids producers already hold still record.
TEST_F(McapRecorderToggleTest, OffClosesTheFileAndOnStartsANewOne) {
    McapRecorder recorder("toggle_test");
    recorder.write_metadata("camera_calibration", "calibration");
    const auto early = recorder.open_channel("/early", "json", empty_json_schema());
    write_one(recorder, early, 1);
    const std::filesystem::path first = recorder.file_path();
    paths_.push_back(first);

    ASSERT_TRUE(recorder.set_enabled(false));
    EXPECT_FALSE(recorder.is_enabled());
    // Registered while off: it has to exist in the next file.
    const auto late = recorder.open_channel("/late", "json", empty_json_schema());
    EXPECT_NE(late, 0u);
    write_one(recorder, early, 2);  // dropped, recording is off
    recorder.flush();               // off only queues the close

    FileContents closed = read_file(first);
    EXPECT_TRUE(closed.has_summary) << "the file recording-off closed is not finalized";
    EXPECT_EQ(closed.count_by_topic["/early"], 1);

    ASSERT_TRUE(recorder.set_enabled(true));
    const std::filesystem::path second = recorder.file_path();
    paths_.push_back(second);
    EXPECT_NE(second, first) << "the new file reused the closed one's name";
    write_one(recorder, early, 3);
    write_one(recorder, late, 4);
    recorder.close();

    FileContents reopened = read_file(second);
    EXPECT_TRUE(reopened.has_summary);
    EXPECT_EQ(reopened.count_by_topic["/early"], 1);
    EXPECT_EQ(reopened.count_by_topic["/late"], 1);
    // Replay needs the calibration from the file itself, so every file carries it.
    EXPECT_EQ(reopened.metadata["camera_calibration"], "calibration");
    EXPECT_EQ(reopened.metadata["active_profile"], "toggle_test");
    // The first file is untouched by the second.
    EXPECT_EQ(read_file(first).count_by_topic["/early"], 1);
}

// The app's logger copies every line into the MCAP. Toggling recording logs from inside the
// recorder's lock, which came straight back into write() on the same thread and deadlocked the
// app on the ZED Box. Run the toggles on a detached thread so a regression fails instead of
// hanging the suite.
TEST_F(McapRecorderToggleTest, TogglingWithTheMcapLogSinkDoesNotDeadlock) {
    const auto previous_logger = spdlog::default_logger();
    auto recorder = std::make_shared<McapRecorder>("toggle_log_test");
    setup_logging(recorder);
    paths_.push_back(recorder->file_path());

    auto done = std::make_shared<std::promise<std::filesystem::path>>();
    auto finished = done->get_future();
    std::thread([recorder, done] {
        recorder->set_enabled(false);
        recorder->set_enabled(true);
        spdlog::info("after reopen");
        done->set_value(recorder->file_path());
    }).detach();
    if (finished.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        ADD_FAILURE() << "set_enabled deadlocked against the MCAP log sink";
        std::abort();  // the stuck thread holds the recorder; nothing can clean up after it
    }
    const std::filesystem::path second = finished.get();
    paths_.push_back(second);
    spdlog::default_logger()->flush();  // the logger is asynchronous
    recorder->close();
    spdlog::set_default_logger(previous_logger);

    FileContents reopened = read_file(second);
    EXPECT_GE(reopened.count_by_topic["/log"], 1) << "log lines stopped reaching the MCAP";
}

// The cross-thread version, seen on the ZED Box after the same-thread one was fixed: another
// thread logs (holding McapLogSink's lock, waiting for the recorder's) while the runner toggles
// (holding the recorder's lock, logging "Recording to ..."). Log from one thread and toggle from
// another until a deadlock would have shown up.
TEST_F(McapRecorderToggleTest, LoggingFromAnotherThreadWhileTogglingDoesNotDeadlock) {
    const auto previous_logger = spdlog::default_logger();
    auto recorder = std::make_shared<McapRecorder>("toggle_abba_test");
    setup_logging(recorder);
    spdlog::default_logger()->sinks().front()->set_level(spdlog::level::off);  // quiet console

    auto stop = std::make_shared<std::atomic<bool>>(false);
    std::thread([stop] {
        while (!stop->load()) spdlog::info("control loop log line");
    }).detach();

    auto done = std::make_shared<std::promise<std::vector<std::filesystem::path>>>();
    auto finished = done->get_future();
    std::thread([recorder, done] {
        std::vector<std::filesystem::path> paths{recorder->file_path()};
        for (int i = 0; i < 200; ++i) {
            recorder->set_enabled(false);
            recorder->set_enabled(true);
            if (paths.back() != recorder->file_path()) paths.push_back(recorder->file_path());
        }
        done->set_value(paths);
    }).detach();
    if (finished.wait_for(std::chrono::seconds(20)) != std::future_status::ready) {
        ADD_FAILURE() << "toggling deadlocked against a thread that was logging";
        std::abort();
    }
    paths_ = finished.get();
    stop->store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // let the logger loop exit
    spdlog::set_default_logger(previous_logger);
    recorder->close();
    EXPECT_GT(paths_.size(), 1u);
}

// Two toggles inside one second both name a file before the writer thread creates either, so
// the second cannot rely on the first existing on disk to pick a new name.
TEST_F(McapRecorderToggleTest, QuickTogglesEachGetTheirOwnFile) {
    McapRecorder recorder("toggle_quick_test");
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

// Toggling closes and opens files on the writer thread. A writer that holds the queue while it
// does so would make every producer wait out each close.
TEST_F(McapRecorderToggleTest, WritesDoNotWaitForTogglesToFinish) {
    McapRecorder recorder("toggle_nowait_test");
    const auto id = recorder.open_channel("/topic", "json", empty_json_schema());
    std::vector<std::filesystem::path> paths{recorder.file_path()};
    // A megabyte a message, so each close has real data to write out.
    const std::vector<std::byte> payload(1 << 20, std::byte{'x'});
    double worst_ms = 0.0;
    for (int i = 0; i < 20; ++i) {
        recorder.set_enabled(false);
        recorder.set_enabled(true);
        paths.push_back(recorder.file_path());
        for (int j = 0; j < 5; ++j) {
            const auto start = std::chrono::steady_clock::now();
            recorder.write(id, payload.data(), payload.size(), 1 + static_cast<uint64_t>(j));
            worst_ms = std::max(worst_ms, std::chrono::duration<double, std::milli>(
                                              std::chrono::steady_clock::now() - start)
                                              .count());
        }
    }
    recorder.close();
    paths_ = paths;
    // A memcpy of 1 MB is well under a millisecond; a write that waited on a close is not.
    EXPECT_LT(worst_ms, 50.0);
}

TEST_F(McapRecorderToggleTest, FlushWaitsForQueuedWrites) {
    McapRecorder recorder("toggle_flush_test");
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

TEST_F(McapRecorderToggleTest, CloseIsFinal) {
    McapRecorder recorder("toggle_test");
    paths_.push_back(recorder.file_path());
    recorder.close();
    EXPECT_FALSE(recorder.set_enabled(true));
    EXPECT_EQ(recorder.file_path(), paths_.front());
}

}  // namespace
}  // namespace auto_battlebot
