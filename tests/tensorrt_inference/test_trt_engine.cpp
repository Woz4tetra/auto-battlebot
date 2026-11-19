#include <gtest/gtest.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
namespace {

std::filesystem::path write_garbage_engine() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "auto_battlebot_garbage.engine";
    std::ofstream(path, std::ios::binary) << std::string(4096, 'x');
    return path;
}

std::filesystem::path find_small_engine() {
    const std::filesystem::path models =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "data" /
        "models";
    if (!std::filesystem::exists(models)) return {};
    for (const auto &entry : std::filesystem::directory_iterator(models)) {
        if (entry.path().extension() == ".engine" && entry.file_size() < 40 * 1024 * 1024) {
            return entry.path();
        }
    }
    return {};
}

size_t count_error_lines_during_garbage_load(bool quiet) {
    const std::filesystem::path garbage = write_garbage_engine();
    auto sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(64);
    spdlog::default_logger()->sinks().push_back(sink);
    {
        TrtEngine engine;
        if (quiet) {
            TrtEngine::ScopedQuietLogging guard;
            engine.load(garbage.string());
        } else {
            engine.load(garbage.string());
        }
    }
    auto &sinks = spdlog::default_logger()->sinks();
    sinks.erase(std::remove(sinks.begin(), sinks.end(), sink), sinks.end());
    std::filesystem::remove(garbage);
    size_t errors = 0;
    for (const auto &message : sink->last_raw()) {
        if (message.level >= spdlog::level::err) ++errors;
    }
    return errors;
}

TEST(TrtEngineTest, ProbeRejectionIsNotLoggedAsError) {
    EXPECT_GT(count_error_lines_during_garbage_load(false), 0u) << "unguarded load reports errors";
    EXPECT_EQ(count_error_lines_during_garbage_load(true), 0u);
}

TEST(TrtEngineTest, FailedLoadLeavesObjectClean) {
    const std::filesystem::path garbage = write_garbage_engine();
    {
        TrtEngine engine;
        TrtEngine::ScopedQuietLogging quiet;
        EXPECT_FALSE(engine.load("/nonexistent/auto_battlebot.engine"));
        EXPECT_FALSE(engine.load(garbage.string()));
        EXPECT_FALSE(engine.isLoaded());
        EXPECT_EQ(engine.getInputNumElements(), 0);
        EXPECT_EQ(engine.getOutputNumElements(), 0);
        EXPECT_EQ(engine.stream(), nullptr);
        std::vector<float> small(4);
        EXPECT_FALSE(engine.execute(small, small));
    }
    std::filesystem::remove(garbage);
}

TEST(TrtEngineTest, FailedLoadLeavesObjectReusable) {
    const std::filesystem::path engine_path = find_small_engine();
    if (engine_path.empty()) GTEST_SKIP() << "no engine under data/models";
    const std::filesystem::path garbage = write_garbage_engine();
    TrtEngine engine;
    {
        TrtEngine::ScopedQuietLogging quiet;
        ASSERT_FALSE(engine.load(garbage.string()));
        if (!engine.load(engine_path.string())) GTEST_SKIP() << "engine does not load here";
    }
    EXPECT_TRUE(engine.isLoaded());
    std::vector<float> input(static_cast<size_t>(engine.getInputNumElements()), 0.0f);
    std::vector<float> output(static_cast<size_t>(engine.getOutputNumElements()));
    EXPECT_TRUE(engine.execute(input, output));
    std::vector<float> one(1);
    EXPECT_FALSE(engine.execute(one, output)) << "undersized input accepted";
    EXPECT_FALSE(engine.execute(input, one)) << "undersized output accepted";
    std::filesystem::remove(garbage);
}

}  // namespace
}  // namespace auto_battlebot
