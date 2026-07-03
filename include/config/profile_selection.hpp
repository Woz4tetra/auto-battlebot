#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace auto_battlebot {

/**
 * Configuration for the UI profile switcher, read from config/profiles.toml.
 * `pattern` filters which config profiles the UI offers; `selection_file` is where the current
 * choice is persisted (outside the repo so uploads/installs do not clobber it).
 */
struct ProfileSelectorConfig {
    std::string pattern;
    std::filesystem::path selection_file;
};

/**
 * Load config/profiles.toml. A missing file or missing keys degrade gracefully to an empty
 * pattern (no profiles offered) and the default selection-file path.
 */
ProfileSelectorConfig load_profile_selector(const std::filesystem::path &config_dir);

/**
 * Enumerate every *.toml under `config_dir` recursively and return those whose path relative to
 * `config_dir` (without the .toml extension, '/'-separated) matches `pattern`. Sorted and unique.
 * An empty pattern matches nothing.
 */
std::vector<std::string> list_available_profiles(const std::filesystem::path &config_dir,
                                                 const std::string &pattern);

/** Read the persisted selection (trimmed single line), or nullopt if the file is absent/empty. */
std::optional<std::string> read_selection_file(const ProfileSelectorConfig &selector);

/** Read config/default_profile (trimmed single line). Returns empty string if absent/empty. */
std::string read_default_profile(const std::filesystem::path &config_dir);

/**
 * Resolve the profile to launch: the persisted selection if it is present in the available set,
 * otherwise the default profile. Throws ConfigValidationError if neither resolves to an existing
 * config file.
 */
std::string resolve_active_profile(const std::filesystem::path &config_dir,
                                   const ProfileSelectorConfig &selector);

/** Persist `profile_name` to the selection file, creating parent directories as needed. */
void write_selection_file(const ProfileSelectorConfig &selector, const std::string &profile_name);

}  // namespace auto_battlebot
