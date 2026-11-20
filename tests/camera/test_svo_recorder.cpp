#ifdef BUILD_WITH_ZED
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rgbd_camera/svo_recorder.hpp"

namespace auto_battlebot {
namespace {

class FakeTarget : public SvoRecordingTarget {
   public:
    struct State {
        bool open = true;
        std::vector<std::string> started;
        int stopped = 0;
    };

    explicit FakeTarget(std::shared_ptr<State> state) : state_(std::move(state)) {}

    bool is_open() const override { return state_->open; }
    bool start_recording(const std::string &path, std::string &) override {
        std::ofstream(path) << std::string(4096, 'x');
        state_->started.push_back(path);
        return true;
    }
    void stop_recording() override { ++state_->stopped; }

   private:
    std::shared_ptr<State> state_;
};

struct RecorderFixture {
    RecorderFixture(uint64_t max_size_bytes)
        : dir(std::filesystem::temp_directory_path() / "svo_recorder_fake_target_test"),
          state(std::make_shared<FakeTarget::State>()),
          recorder(std::make_unique<FakeTarget>(state), dir, max_size_bytes, 1ull << 30) {
        std::filesystem::remove_all(dir);
    }
    ~RecorderFixture() { std::filesystem::remove_all(dir); }

    std::filesystem::path dir;
    std::shared_ptr<FakeTarget::State> state;
    SvoRecorder recorder;
};

TEST(SvoRecorderTest, SameSecondRestartGetsPartSuffixInsteadOfOverwriting) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "svo_recorder_part_suffix_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    const std::filesystem::path first = unused_svo_path(dir, "2026-01-01T00-00-00");
    EXPECT_EQ(first.filename(), "2026-01-01T00-00-00.svo2");
    std::ofstream(first).put('x');

    const std::filesystem::path second = unused_svo_path(dir, "2026-01-01T00-00-00");
    EXPECT_EQ(second.filename(), "2026-01-01T00-00-00_part01.svo2");
    std::ofstream(second).put('x');

    EXPECT_EQ(unused_svo_path(dir, "2026-01-01T00-00-00").filename(),
              "2026-01-01T00-00-00_part02.svo2");
    std::filesystem::remove_all(dir);
}

TEST(SvoRecorderTest, FramesClaimConsecutiveIndicesFromZeroAfterStart) {
    RecorderFixture fixture(1ull << 40);
    EXPECT_EQ(fixture.recorder.on_frame_grabbed().index, -1);
    ASSERT_TRUE(fixture.recorder.start());
    const auto first = fixture.recorder.on_frame_grabbed();
    const auto second = fixture.recorder.on_frame_grabbed();
    EXPECT_EQ(first.index, 0);
    EXPECT_EQ(second.index, 1);
    EXPECT_EQ(first.path, fixture.recorder.current_path());
}

TEST(SvoRecorderTest, StartFailsWhileTheCameraIsClosed) {
    RecorderFixture fixture(1ull << 40);
    fixture.state->open = false;
    EXPECT_FALSE(fixture.recorder.start());
    EXPECT_FALSE(fixture.recorder.is_recording());
    EXPECT_TRUE(fixture.state->started.empty());
}

TEST(SvoRecorderTest, FramesAfterStopClaimNoIndexAndNeverRollOver) {
    RecorderFixture fixture(1);
    ASSERT_TRUE(fixture.recorder.start());
    fixture.recorder.stop();
    for (int i = 0; i < 300; ++i) {
        const auto frame = fixture.recorder.on_frame_grabbed();
        EXPECT_EQ(frame.index, -1);
        EXPECT_TRUE(frame.path.empty());
    }
    EXPECT_EQ(fixture.state->started.size(), 1u);
    EXPECT_FALSE(fixture.recorder.desired());
}

TEST(SvoRecorderTest, OversizedFileRollsOverToANewPathAndRestartsTheIndex) {
    RecorderFixture fixture(1);
    ASSERT_TRUE(fixture.recorder.start());
    const std::string first_path = fixture.recorder.current_path();
    int64_t last_index = -1;
    for (int i = 0; i < 100; ++i) last_index = fixture.recorder.on_frame_grabbed().index;
    EXPECT_EQ(last_index, 99);
    ASSERT_EQ(fixture.state->started.size(), 2u);
    EXPECT_EQ(fixture.state->stopped, 1);
    EXPECT_NE(fixture.recorder.current_path(), first_path);
    EXPECT_EQ(fixture.recorder.on_frame_grabbed().index, 0);
}

}  // namespace
}  // namespace auto_battlebot
#endif
