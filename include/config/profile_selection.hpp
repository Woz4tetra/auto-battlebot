#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace auto_battlebot {

struct ProfileSelectorConfig {
    std::string pattern;
    std::filesystem::path selection_file;
};

ProfileSelectorConfig load_profile_selector(const std::filesystem::path &config_dir);

std::vector<std::string> list_available_profiles(const std::filesystem::path &config_dir,
                                                 const std::string &pattern);

std::optional<std::string> read_selection_file(const ProfileSelectorConfig &selector);

std::optional<std::string> read_default_profile(const std::filesystem::path &config_dir);

std::string resolve_active_profile(const std::filesystem::path &config_dir,
                                   const ProfileSelectorConfig &selector);

std::filesystem::path expand_home(const std::string &raw);

std::filesystem::path expand_home(const std::string &raw, const std::filesystem::path &home);

void write_selection_file(const ProfileSelectorConfig &selector, const std::string &profile_name);

}  // namespace auto_battlebot
