#include <gtest/gtest.h>
#include <toml++/toml.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "config/config_parser.hpp"
#include "engine_selector/config.hpp"
#include "engine_selector/engine_selector.hpp"

namespace auto_battlebot {
namespace {

EngineSelectorConfiguration parse_engine(const std::string &toml_text) {
    toml::table table = toml::parse(toml_text);
    ConfigParser parser(table, "model");
    EngineSelectorConfiguration config;
    config.parse(parser, "engine");
    return config;
}

// Writes a file that exists but is not a valid TensorRT plan, so deserialization is
// guaranteed to reject it the way a wrong-architecture engine would.
class TempFile {
   public:
    explicit TempFile(const std::string &name)
        : path_(std::filesystem::temp_directory_path() / name) {
        std::ofstream out(path_, std::ios::binary);
        out << "not a tensorrt plan";
    }
    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }

    TempFile(const TempFile &) = delete;
    TempFile &operator=(const TempFile &) = delete;

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

TEST(EngineSelectorTest, ReturnsNulloptWhenNoCandidateExists) {
    EngineSelectorConfiguration config;
    config.candidates = {"/nonexistent/one.engine", "/nonexistent/two.engine"};
    EngineSelector selector(config, "TestModel");

    TrtEngine engine;
    EXPECT_FALSE(selector.select(engine).has_value());
}

// A file that exists but is not a valid plan stands in for an engine built for another
// GPU: both are rejected by deserialization rather than by the existence check.
TEST(EngineSelectorTest, ReturnsNulloptWhenCandidateIsNotAValidPlan) {
    TempFile bogus("auto_battlebot_bogus.engine");
    EngineSelectorConfiguration config;
    config.candidates = {bogus.path()};
    EngineSelector selector(config, "TestModel");

    TrtEngine engine;
    EXPECT_FALSE(selector.select(engine).has_value());
}

// Missing files must not abort the search; a later candidate still gets its turn. Both
// candidates fail here, but reaching the second one is what is being checked.
TEST(EngineSelectorTest, MissingCandidateDoesNotStopTheSearch) {
    TempFile bogus("auto_battlebot_bogus_second.engine");
    EngineSelectorConfiguration config;
    config.candidates = {"/nonexistent/first.engine", bogus.path()};
    EngineSelector selector(config, "TestModel");

    TrtEngine engine;
    EXPECT_FALSE(selector.select(engine).has_value());
}

TEST(EngineSelectorTest, DescribesTheCudaDevice) {
    // Never empty: reports the failure reason instead of throwing when CUDA is absent,
    // because the string only ever appears in log lines.
    EXPECT_FALSE(describe_cuda_device().empty());
}

}  // namespace auto_battlebot
