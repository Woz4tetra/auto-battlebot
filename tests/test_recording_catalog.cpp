#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "config/config_parser.hpp"
#include "config/recording_catalog.hpp"
#include "directories.hpp"

namespace auto_battlebot {
namespace {
class RecordingCatalogTest : public ::testing::Test {
   protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() / "auto_battlebot_test_recording_catalog";
        std::filesystem::remove_all(dir_);
        std::filesystem::create_directories(dir_);
    }

    void TearDown() override { std::filesystem::remove_all(dir_); }

    RecordingCatalog load(const std::string &content) {
        const std::filesystem::path file = dir_ / "recordings.toml";
        std::ofstream(file) << content;
        return RecordingCatalog::load(file);
    }

    void expect_load_error(const std::string &content, const std::string &needle) {
        try {
            load(content);
            FAIL() << "expected ConfigValidationError";
        } catch (const ConfigValidationError &e) {
            EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
        }
    }

    std::filesystem::path dir_;
};

TEST_F(RecordingCatalogTest, FindsEntryByKey) {
    RecordingCatalog catalog = load(
        "[recordings.\"2026-05-02T11-45-08\"]\npath = \"data/svo/a/2026-05-02T11-45-08.svo2\"\n"
        "start_frame = 13000\nnote = \"vs Sphinx\"\n");

    const CatalogRecording *entry = catalog.find("2026-05-02T11-45-08");
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->path, "data/svo/a/2026-05-02T11-45-08.svo2");
    EXPECT_EQ(entry->start_frame, 13000);
    EXPECT_EQ(entry->note, "vs Sphinx");
    EXPECT_EQ(catalog.find("2026-05-02T11-45"), nullptr);
}

TEST_F(RecordingCatalogTest, FindsEntryByPathStem) {
    RecordingCatalog catalog = load("[recordings.match]\npath = \"x/match.svo2\"\n");

    EXPECT_NE(catalog.find_by_stem("/elsewhere/match.svo2"), nullptr);
    EXPECT_EQ(catalog.find_by_stem("/elsewhere/other.svo2"), nullptr);
}

TEST_F(RecordingCatalogTest, MissingStartFrameDefaultsToZero) {
    RecordingCatalog catalog = load("[recordings.match]\npath = \"x/match.svo2\"\n");

    ASSERT_EQ(catalog.entries().size(), 1u);
    EXPECT_EQ(catalog.entries()[0].start_frame, 0);
    EXPECT_EQ(catalog.entries()[0].note, "");
}

TEST_F(RecordingCatalogTest, NonStringPathThrows) {
    expect_load_error("[recordings.match]\npath = 3\n", "`path` must be a string");
}

TEST_F(RecordingCatalogTest, MissingPathThrows) {
    expect_load_error("[recordings.match]\nstart_frame = 3\n", "missing `path`");
}

TEST_F(RecordingCatalogTest, NegativeStartFrameThrows) {
    expect_load_error("[recordings.match]\npath = \"a.svo2\"\nstart_frame = -1\n",
                      "non-negative integer");
}

TEST_F(RecordingCatalogTest, UnknownKeyThrows) {
    expect_load_error("[recordings.match]\npath = \"a.svo2\"\nstart = 3\n", "unknown key 'start'");
}

TEST_F(RecordingCatalogTest, ListMarksWhichFilesArePresent) {
    const std::filesystem::path here = dir_ / "here.svo2";
    std::ofstream(here) << "x";
    RecordingCatalog catalog = load("[recordings.here]\npath = \"" + here.generic_string() +
                                    "\"\nstart_frame = 7\nnote = \"kept\"\n\n"
                                    "[recordings.gone]\npath = \"" +
                                    (dir_ / "gone.svo2").generic_string() + "\"\n");

    const std::string listing = format_recording_list(catalog);

    EXPECT_NE(listing.find("gone  0       no"), std::string::npos) << listing;
    EXPECT_NE(listing.find("here  7       yes   kept"), std::string::npos) << listing;
}

TEST(CheckedInRecordingCatalog, LoadsAndEveryKeyIsItsPathStem) {
    RecordingCatalog catalog = RecordingCatalog::load(recording_catalog_path(get_config_dir()));

    ASSERT_FALSE(catalog.entries().empty());
    for (const auto &entry : catalog.entries()) {
        EXPECT_EQ(entry.path.stem().string(), entry.name);
        EXPECT_TRUE(entry.path.is_relative()) << entry.path;
    }
    const CatalogRecording *sphinx = catalog.find("2026-05-02T11-45-08");
    ASSERT_NE(sphinx, nullptr);
    EXPECT_EQ(sphinx->start_frame, 13000);
}
}  // namespace
}  // namespace auto_battlebot
