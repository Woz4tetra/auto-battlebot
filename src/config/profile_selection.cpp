#include "config/profile_selection.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <regex>

#include "config/config_parser.hpp"

namespace auto_battlebot {
namespace {

constexpr const char *kProfilesFileName = "profiles.toml";
constexpr const char *kDefaultProfileFileName = "default_profile";
constexpr const char *kDefaultSelectionFile = "$HOME/.local/state/auto_battlebot/selected_profile";

// Expand a leading "$HOME" or "~" against the HOME environment variable. Leaves other paths as-is.
std::filesystem::path expand_home(const std::string &raw) {
    const bool tilde = !raw.empty() && raw.front() == '~';
    const bool home_var = raw.rfind("$HOME", 0) == 0;
    if (tilde || home_var) {
        const char *home = std::getenv("HOME");
        if (home != nullptr && *home != '\0') {
            std::string suffix = raw.substr(tilde ? 1 : 5);
            if (!suffix.empty() && suffix.front() == '/') suffix.erase(0, 1);
            return std::filesystem::path(home) / suffix;
        }
    }
    return std::filesystem::path(raw);
}

std::string trim(const std::string &s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

// Read the first non-empty, trimmed line of a file, or nullopt if the file is absent/empty.
std::optional<std::string> read_first_line(const std::filesystem::path &path) {
    std::ifstream file(path);
    if (!file) return std::nullopt;
    std::string line;
    while (std::getline(file, line)) {
        std::string trimmed = trim(line);
        if (!trimmed.empty()) return trimmed;
    }
    return std::nullopt;
}

// Profile identifier: path relative to config_dir, without .toml, '/'-separated.
std::string profile_id(const std::filesystem::path &config_dir, const std::filesystem::path &file) {
    std::filesystem::path rel = std::filesystem::relative(file, config_dir);
    rel.replace_extension();
    return rel.generic_string();
}

}  // namespace

ProfileSelectorConfig load_profile_selector(const std::filesystem::path &config_dir) {
    ProfileSelectorConfig selector;
    selector.selection_file = expand_home(kDefaultSelectionFile);

    const std::filesystem::path path = config_dir / kProfilesFileName;
    if (!std::filesystem::exists(path)) {
        spdlog::warn("Profile selector config not found at {}; profile switching disabled",
                     path.string());
        return selector;
    }
    try {
        toml::table root = toml::parse_file(path.string());
        const toml::table *section = root["profiles"].as_table();
        if (section == nullptr) {
            spdlog::error("{} is missing the [profiles] section", path.string());
            return selector;
        }
        ConfigParser parser(*section, "profiles");
        selector.pattern = parser.get_optional_string("pattern", "");
        const std::string selection_file =
            parser.get_optional_string("selection_file", kDefaultSelectionFile);
        parser.validate_no_extra_fields();
        selector.selection_file = expand_home(selection_file);
    } catch (const toml::parse_error &e) {
        spdlog::error("Failed to parse {}: {}", path.string(), std::string(e.description()));
    } catch (const ConfigValidationError &e) {
        spdlog::error("Invalid {}: {}", path.string(), e.what());
    }
    return selector;
}

std::vector<std::string> list_available_profiles(const std::filesystem::path &config_dir,
                                                 const std::string &pattern) {
    std::vector<std::string> profiles;
    if (pattern.empty() || !std::filesystem::is_directory(config_dir)) {
        return profiles;
    }
    std::regex regex;
    try {
        regex = std::regex(pattern);
    } catch (const std::regex_error &e) {
        spdlog::error("Invalid profile regex '{}': {}", pattern, e.what());
        return profiles;
    }
    std::error_code ec;
    for (const auto &entry : std::filesystem::recursive_directory_iterator(config_dir, ec)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".toml") continue;
        const std::string id = profile_id(config_dir, entry.path());
        if (std::regex_match(id, regex)) profiles.push_back(id);
    }
    std::sort(profiles.begin(), profiles.end());
    profiles.erase(std::unique(profiles.begin(), profiles.end()), profiles.end());
    return profiles;
}

std::optional<std::string> read_selection_file(const ProfileSelectorConfig &selector) {
    if (selector.selection_file.empty()) return std::nullopt;
    return read_first_line(selector.selection_file);
}

std::string read_default_profile(const std::filesystem::path &config_dir) {
    return read_first_line(config_dir / kDefaultProfileFileName).value_or("");
}

std::string resolve_active_profile(const std::filesystem::path &config_dir,
                                   const ProfileSelectorConfig &selector) {
    const std::vector<std::string> available =
        list_available_profiles(config_dir, selector.pattern);

    const std::optional<std::string> selected = read_selection_file(selector);
    if (selected && std::find(available.begin(), available.end(), *selected) != available.end()) {
        spdlog::info("Using selected profile '{}'", *selected);
        return *selected;
    }
    if (selected) {
        spdlog::warn("Selected profile '{}' is not available; falling back to default profile",
                     *selected);
    }

    const std::string fallback = read_default_profile(config_dir);
    if (fallback.empty()) {
        throw ConfigValidationError("No profile selected and no default in " +
                                    (config_dir / kDefaultProfileFileName).string());
    }
    const std::filesystem::path fallback_path = config_dir / (fallback + ".toml");
    if (!std::filesystem::exists(fallback_path)) {
        throw ConfigValidationError("Default profile '" + fallback + "' not found at " +
                                    fallback_path.string());
    }
    spdlog::info("Using default profile '{}'", fallback);
    return fallback;
}

void write_selection_file(const ProfileSelectorConfig &selector, const std::string &profile_name) {
    if (selector.selection_file.empty()) {
        spdlog::error("Cannot persist profile selection: no selection_file configured");
        return;
    }
    std::error_code ec;
    const std::filesystem::path parent = selector.selection_file.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            spdlog::error("Failed to create {}: {}", parent.string(), ec.message());
            return;
        }
    }
    std::ofstream file(selector.selection_file, std::ios::trunc);
    if (!file) {
        spdlog::error("Failed to open {} for writing", selector.selection_file.string());
        return;
    }
    file << profile_name << "\n";
    spdlog::info("Persisted profile selection '{}' to {}", profile_name,
                 selector.selection_file.string());
}

}  // namespace auto_battlebot
