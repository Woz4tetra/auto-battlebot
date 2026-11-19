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

/**
 * Expand a leading "$HOME" or "~" against the current user's home directory. Leaves other paths
 * as-is. Throws if a home-relative path cannot be resolved, rather than silently returning the
 * unexpanded literal as a relative path: under systemd with a stripped environment that literal
 * would be written under the working directory (inside the repo) and wiped by the next deploy.
 */
std::filesystem::path expand_home(const std::string &raw);

void write_selection_file(const ProfileSelectorConfig &selector, const std::string &profile_name);

}  // namespace auto_battlebot
