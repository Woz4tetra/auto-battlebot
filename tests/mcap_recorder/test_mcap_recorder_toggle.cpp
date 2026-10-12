#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <map>
#include <mcap/reader.hpp>
#include <string>
#include <vector>

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

TEST_F(McapRecorderToggleTest, CloseIsFinal) {
    McapRecorder recorder("toggle_test");
    paths_.push_back(recorder.file_path());
    recorder.close();
    EXPECT_FALSE(recorder.set_enabled(true));
    EXPECT_EQ(recorder.file_path(), paths_.front());
}

}  // namespace
}  // namespace auto_battlebot
