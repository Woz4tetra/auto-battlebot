#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "config/camera_profile_store.hpp"
#include "config/config_parser.hpp"

namespace auto_battlebot {
namespace {

class CameraProfileStoreTest : public ::testing::Test {
   protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() / "auto_battlebot_camera_profile_test";
        std::filesystem::remove_all(dir_);
        path_ = dir_ / "state" / "camera_profiles.toml";
    }
    void TearDown() override { std::filesystem::remove_all(dir_); }

    void write_file(const std::string &content) {
        std::filesystem::create_directories(path_.parent_path());
        std::ofstream(path_) << content;
    }
    std::string read_file() {
        std::ifstream in(path_);
        std::stringstream text;
        text << in.rdbuf();
        return text.str();
    }

    static CameraSettings cage() {
        CameraSettings s;
        s.auto_exposure = false;
        s.exposure_us = 3000;
        s.analog_gain_mdb = 14000;
        s.digital_gain = 2;
        s.auto_white_balance = false;
        s.white_balance_k = 5600;
        s.saturation = 5;
        s.gamma = 6;
        s.sharpness = 3;
        s.anti_banding = AntiBanding::HZ_60;
        return s;
    }

    std::filesystem::path dir_;
    std::filesystem::path path_;
};

TEST_F(CameraProfileStoreTest, MissingFileOffersOnlyDefault) {
    CameraProfileStore store(path_);
    store.load();
    EXPECT_EQ(store.names(), std::vector<std::string>{"DEFAULT"});
    EXPECT_TRUE(store.contains("DEFAULT"));
    EXPECT_FALSE(store.settings("DEFAULT").has_value());
}

TEST_F(CameraProfileStoreTest, SaveRoundTripsAndKeepsDefaultFirst) {
    {
        CameraProfileStore store(path_);
        store.load();
        store.save("nhrl_cage_2", cage());
        store.save("pit", CameraSettings{});
    }
    CameraProfileStore reloaded(path_);
    reloaded.load();
    EXPECT_EQ(reloaded.names(), (std::vector<std::string>{"DEFAULT", "nhrl_cage_2", "pit"}));
    ASSERT_TRUE(reloaded.settings("nhrl_cage_2").has_value());
    EXPECT_EQ(*reloaded.settings("nhrl_cage_2"), cage());
    EXPECT_EQ(*reloaded.settings("pit"), CameraSettings{});
    EXPECT_FALSE(std::filesystem::exists(path_.string() + ".tmp"));
}

TEST_F(CameraProfileStoreTest, SaveOverwritesInPlace) {
    CameraProfileStore store(path_);
    store.load();
    store.save("a", CameraSettings{});
    store.save("b", CameraSettings{});
    store.save("a", cage());
    EXPECT_EQ(store.names(), (std::vector<std::string>{"DEFAULT", "a", "b"}));
    EXPECT_EQ(*store.settings("a"), cage());
}

TEST_F(CameraProfileStoreTest, DefaultCannotBeSavedOrDeleted) {
    CameraProfileStore store(path_);
    store.load();
    EXPECT_THROW(store.save("DEFAULT", cage()), std::runtime_error);
    EXPECT_THROW(store.save("default", cage()), std::runtime_error);
    EXPECT_THROW(store.remove("DEFAULT"), std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(path_));
}

TEST_F(CameraProfileStoreTest, RejectsBadNames) {
    CameraProfileStore store(path_);
    store.load();
    EXPECT_THROW(store.save("", cage()), std::runtime_error);
    EXPECT_THROW(store.save("has space", cage()), std::runtime_error);
    EXPECT_THROW(store.save("quote\"", cage()), std::runtime_error);
    EXPECT_THROW(store.save(std::string(33, 'a'), cage()), std::runtime_error);
}

TEST_F(CameraProfileStoreTest, RemoveDeletesAndUnknownThrows) {
    CameraProfileStore store(path_);
    store.load();
    store.save("a", cage());
    store.remove("a");
    EXPECT_EQ(store.names(), std::vector<std::string>{"DEFAULT"});
    EXPECT_THROW(store.remove("a"), std::runtime_error);
}

TEST_F(CameraProfileStoreTest, BrokenFileIsNeverOverwritten) {
    const std::string broken = "[[profile]]\nname = \"a\"\nexposure_uss = 3000\n";
    write_file(broken);
    CameraProfileStore store(path_);
    store.load();
    EXPECT_EQ(store.names(), std::vector<std::string>{"DEFAULT"});
    EXPECT_THROW(store.save("b", cage()), std::runtime_error);
    EXPECT_EQ(read_file(), broken);
}

TEST(CameraProfileParse, RejectsUnknownFieldsAndBadEnums) {
    EXPECT_THROW(parse_camera_profiles("[[profile]]\nname = \"a\"\ngain = 1\n", "t"),
                 ConfigValidationError);
    EXPECT_THROW(
        parse_camera_profiles("[[profile]]\nname = \"a\"\nanti_banding = \"HZ_70\"\n", "t"),
        ConfigValidationError);
    EXPECT_THROW(parse_camera_profiles("active = \"a\"\n", "t"), ConfigValidationError);
    EXPECT_THROW(
        parse_camera_profiles("[[profile]]\nname = \"a\"\n[[profile]]\nname = \"a\"\n", "t"),
        ConfigValidationError);
    EXPECT_THROW(parse_camera_profiles("[[profile]]\nname = \"DEFAULT\"\n", "t"),
                 ConfigValidationError);
}

TEST(CameraProfileParse, MissingFieldsTakeStructDefaults) {
    const auto profiles = parse_camera_profiles("[[profile]]\nname = \"bare\"\n", "t");
    ASSERT_EQ(profiles.size(), 1u);
    EXPECT_EQ(profiles[0].settings, CameraSettings{});
}

}  // namespace
}  // namespace auto_battlebot
