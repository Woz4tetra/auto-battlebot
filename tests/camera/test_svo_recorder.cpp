#ifdef BUILD_WITH_ZED
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "rgbd_camera/svo_recorder.hpp"

namespace auto_battlebot {
namespace {

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

}  // namespace
}  // namespace auto_battlebot
#endif
