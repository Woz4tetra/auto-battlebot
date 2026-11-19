#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "config/config_parser.hpp"
#include "engine_selector/config.hpp"
#include "engine_selector/engine_selector.hpp"
#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
namespace {

EngineSelectorConfiguration parse_engine(const std::string &toml_text) {
    toml::table table = toml::parse(toml_text);
    ConfigParser parser(table, "model");
    EngineSelectorConfiguration config;
    config.parse(parser, "engine");
    return config;
}

class InvalidPlanFile {
   public:
    explicit InvalidPlanFile(const std::string &name)
        : path_(std::filesystem::temp_directory_path() / name) {
        std::ofstream out(path_, std::ios::binary);
        out << "not a tensorrt plan";
    }
    ~InvalidPlanFile() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }

    InvalidPlanFile(const InvalidPlanFile &) = delete;
    InvalidPlanFile &operator=(const InvalidPlanFile &) = delete;

    std::string path() const { return path_.string(); }

   private:
    std::filesystem::path path_;
};

}  // namespace

TEST(EngineSelectorConfigTest, ParsesCandidateList) {
    auto config = parse_engine(R"(
[engine]
candidates = ["a.engine", "b.engine"]
)");

    ASSERT_EQ(config.candidates.size(), 2u);
    EXPECT_EQ(config.candidates[0], "a.engine");
    EXPECT_EQ(config.candidates[1], "b.engine");
}

TEST(EngineSelectorConfigTest, MissingTableThrows) {
    EXPECT_THROW(parse_engine(R"(
other = 1
)"),
                 ConfigValidationError);
}

TEST(EngineSelectorConfigTest, EmptyCandidateListThrows) {
    EXPECT_THROW(parse_engine(R"(
[engine]
candidates = []
)"),
                 ConfigValidationError);
}

TEST(EngineSelectorConfigTest, UnknownKeyInsideSubconfigThrows) {
    EXPECT_THROW(parse_engine(R"(
[engine]
candidates = ["a.engine"]
typo_field = 3
)"),
                 ConfigValidationError);
}

size_t count_occurrences(const std::string &text, const std::string &needle) {
    size_t count = 0;
    for (size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + 1)) {
        ++count;
    }
    return count;
}

std::string select_capturing_log(EngineSelector &selector) {
    std::ostringstream captured;
    auto logger = std::make_shared<spdlog::logger>(
        "engine_selector_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(captured));
    const auto previous = spdlog::default_logger();
    spdlog::set_default_logger(logger);
    TrtEngine engine;
    EXPECT_FALSE(selector.select(engine).has_value());
    spdlog::set_default_logger(previous);
    return captured.str();
}

TEST(EngineSelectorTest, ReturnsNulloptWhenNoCandidateExists) {
    EngineSelectorConfiguration config;
    config.candidates = {"/nonexistent/one.engine", "/nonexistent/two.engine"};
    EngineSelector selector(config, "TestModel");

    TrtEngine engine;
    EXPECT_FALSE(selector.select(engine).has_value());
}

TEST(EngineSelectorTest, ReturnsNulloptWhenCandidateFailsDeserialization) {
    InvalidPlanFile bogus("auto_battlebot_bogus.engine");
    EngineSelectorConfiguration config;
    config.candidates = {bogus.path()};
    EngineSelector selector(config, "TestModel");

    TrtEngine engine;
    EXPECT_FALSE(selector.select(engine).has_value());
}

TEST(EngineSelectorTest, MissingCandidateDoesNotStopTheSearch) {
    InvalidPlanFile bogus("auto_battlebot_bogus_second.engine");
    EngineSelectorConfiguration config;
    config.candidates = {"/nonexistent/first.engine", bogus.path()};
    EngineSelector selector(config, "TestModel");

    const std::string log = select_capturing_log(selector);
    EXPECT_EQ(count_occurrences(log, bogus.path() + " -> rejected by TensorRT"), 1u) << log;
}

TEST(EngineSelectorTest, SelectLogsOutcomeOnce) {
    InvalidPlanFile bogus("auto_battlebot_bogus_logged.engine");
    EngineSelectorConfiguration config;
    config.candidates = {"/nonexistent/first.engine", bogus.path()};
    EngineSelector selector(config, "TestModel");

    const std::string log = select_capturing_log(selector);
    EXPECT_EQ(count_occurrences(log, "no candidate engine loads"), 1u) << log;
    EXPECT_EQ(count_occurrences(log, "/nonexistent/first.engine -> not present"), 1u) << log;
    EXPECT_EQ(count_occurrences(log, bogus.path() + " -> rejected by TensorRT"), 1u) << log;
}

TEST(EngineSelectorTest, DescribeCudaDeviceIsNeverEmpty) {
    EXPECT_FALSE(describe_cuda_device().empty());
}

}  // namespace auto_battlebot

TEST(ScopedQuietLoggingTest, QuietsOnlyTheThreadHoldingTheGuard) {
    auto_battlebot::TrtEngine::ScopedQuietLogging quiet;
    EXPECT_TRUE(auto_battlebot::TrtEngine::ScopedQuietLogging::active_on_this_thread());
    bool other_thread_quiet = true;
    std::thread other([&] {
        other_thread_quiet = auto_battlebot::TrtEngine::ScopedQuietLogging::active_on_this_thread();
    });
    other.join();
    EXPECT_FALSE(other_thread_quiet);
}

TEST(ScopedQuietLoggingTest, NestedGuardsStayQuietUntilTheOuterOneEnds) {
    using Quiet = auto_battlebot::TrtEngine::ScopedQuietLogging;
    {
        Quiet outer;
        { Quiet inner; }
        EXPECT_TRUE(Quiet::active_on_this_thread());
    }
    EXPECT_FALSE(Quiet::active_on_this_thread());
}
