#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "viz/static_files.hpp"

namespace auto_battlebot::viz {
namespace {

class StaticFilesTest : public ::testing::Test {
   protected:
    void SetUp() override {
        base_ = std::filesystem::temp_directory_path() /
                ("static_files_test_" + std::to_string(::getpid()));
        std::filesystem::remove_all(base_);
        root_ = base_ / "dist";
        std::filesystem::create_directories(root_ / "assets");
        std::ofstream(root_ / "index.html") << "<html></html>";
        std::ofstream(root_ / "assets" / "index-abc123.js") << "console.log(1)";
        std::ofstream(base_ / "secret.txt") << "outside";
    }
    void TearDown() override { std::filesystem::remove_all(base_); }

    std::filesystem::path base_;
    std::filesystem::path root_;
};

TEST_F(StaticFilesTest, ServesIndexAndAssets) {
    auto index = resolve_static_path(root_, "/");
    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(index->filename(), "index.html");
    auto asset = resolve_static_path(root_, "/assets/index-abc123.js");
    ASSERT_TRUE(asset.has_value());
    EXPECT_EQ(content_type_for(*asset), "text/javascript; charset=utf-8");
}

TEST_F(StaticFilesTest, RejectsTraversal) {
    EXPECT_FALSE(resolve_static_path(root_, "/../secret.txt"));
    EXPECT_FALSE(resolve_static_path(root_, "/assets/../../secret.txt"));
    EXPECT_FALSE(resolve_static_path(root_, "/%2e%2e/secret.txt"));
    EXPECT_FALSE(resolve_static_path(root_, "/%2E%2E%2fsecret.txt"));
    EXPECT_FALSE(resolve_static_path(root_, "/assets/./index-abc123.js"));
    EXPECT_FALSE(resolve_static_path(root_, "/..%5csecret.txt"));
    EXPECT_FALSE(resolve_static_path(root_, "/%00"));
    EXPECT_FALSE(resolve_static_path(root_, "/%zz"));
}

TEST_F(StaticFilesTest, RejectsAbsoluteAndEmptySegments) {
    EXPECT_FALSE(resolve_static_path(root_, (base_ / "secret.txt").string()));
    EXPECT_FALSE(resolve_static_path(root_, "/" + (base_ / "secret.txt").string()));
    EXPECT_FALSE(resolve_static_path(root_, "//etc/passwd"));
    EXPECT_FALSE(resolve_static_path(root_, "index.html"));
}

TEST_F(StaticFilesTest, RejectsSymlinksOutOfTheRoot) {
    std::filesystem::create_symlink(base_ / "secret.txt", root_ / "leak.txt");
    std::filesystem::create_directory_symlink(base_, root_ / "up");
    EXPECT_FALSE(resolve_static_path(root_, "/leak.txt"));
    EXPECT_FALSE(resolve_static_path(root_, "/up/secret.txt"));
}

TEST_F(StaticFilesTest, MissingFilesAndDirectoriesAreNotFound) {
    EXPECT_FALSE(resolve_static_path(root_, "/nope.js"));
    EXPECT_FALSE(resolve_static_path(root_, "/assets"));
}

}  // namespace
}  // namespace auto_battlebot::viz
