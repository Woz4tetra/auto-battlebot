#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include "config/config.hpp"
#include "config/config_parser.hpp"
#include "config/profile_selection.hpp"
#include "directories.hpp"

namespace auto_battlebot {
namespace {
std::vector<std::string> offered_profiles() {
    const ProfileSelectorConfig selector = load_profile_selector(get_config_dir());
    return list_available_profiles(get_config_dir(), selector.pattern);
}

TEST(ProfilePattern, OffersExactlyTheDeployableProfiles) {
    const std::vector<std::string> expected = {
        "mr_stabs_mk2_aer_zed_box", "mr_stabs_mk2_fid_zed_box", "mr_stabs_mk2_sysid_zed_box",
        "mrs_buff_mk3_aer_zed_box", "mrs_buff_mk3_zed_box",
    };
    EXPECT_EQ(offered_profiles(), expected);
}

TEST(ProfilePattern, ExcludesBasesSectionFilesAndDesktopProfiles) {
    const std::vector<std::string> offered = offered_profiles();
    for (const std::string name :
         {"_zed_box", "_common", "mr_stabs_mk2_desktop", "playback/_playback",
          "transmitters/mr_stabs_mk2_espnow", "robots/mr_stabs_mk2", "profiles"}) {
        EXPECT_EQ(std::count(offered.begin(), offered.end(), name), 0) << name;
    }
}

TEST(ProfilePattern, EveryOfferedProfileLoads) {
    for (const std::string &profile : offered_profiles()) {
        EXPECT_NO_THROW(load_classes_from_config(get_config_dir() / (profile + ".toml")))
            << profile;
    }
}

TEST(ExpandHomeTest, ExpandsTildeAndHomeVariableAgainstTheGivenHome) {
    EXPECT_EQ(expand_home("~/a/b", "/home/x"), std::filesystem::path("/home/x/a/b"));
    EXPECT_EQ(expand_home("$HOME/a", "/home/x"), std::filesystem::path("/home/x/a"));
    EXPECT_EQ(expand_home("/abs/path", ""), std::filesystem::path("/abs/path"));
}

TEST(ExpandHomeTest, ThrowsWhenHomeIsUnresolvable) {
    EXPECT_THROW(expand_home("~/a", ""), ConfigValidationError);
    EXPECT_THROW(expand_home("$HOME/a", ""), ConfigValidationError);
}

}  // namespace
}  // namespace auto_battlebot
