#include "config/camera_profile_store.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <magic_enum.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "config/config_parser.hpp"

namespace auto_battlebot {
namespace {

constexpr std::size_t kMaxNameLength = 32;

bool is_default(const std::string &name) {
    std::string upper = name;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return upper == CameraProfileStore::kDefaultName;
}

void check_name(const std::string &name) {
    if (name.empty() || name.size() > kMaxNameLength) {
        throw std::runtime_error("Profile names are 1 to 32 characters");
    }
    const bool valid = std::all_of(name.begin(), name.end(), [](unsigned char c) {
        return std::isalnum(c) != 0 || c == '_' || c == '-';
    });
    if (!valid) {
        throw std::runtime_error("Profile names use only letters, digits, '_' and '-'");
    }
    if (is_default(name)) {
        throw std::runtime_error("DEFAULT is built in and cannot be changed");
    }
}

int get_int(ConfigParser &parser, const std::string &key, int fallback) {
    return static_cast<int>(parser.get_optional_int(key, fallback));
}

CameraProfile parse_profile(const toml::table &table, const std::string &section) {
    ConfigParser parser(table, section);
    CameraProfile profile;
    profile.name = parser.get_required_string("name");
    try {
        check_name(profile.name);
    } catch (const std::runtime_error &e) {
        throw ConfigValidationError(section + ": '" + profile.name + "': " + e.what());
    }
    CameraSettings &s = profile.settings;
    s.auto_exposure = parser.get_optional_bool("auto_exposure", s.auto_exposure);
    s.exposure_us = get_int(parser, "exposure_us", s.exposure_us);
    s.analog_gain_mdb = get_int(parser, "analog_gain_mdb", s.analog_gain_mdb);
    s.digital_gain = get_int(parser, "digital_gain", s.digital_gain);
    s.auto_white_balance = parser.get_optional_bool("auto_white_balance", s.auto_white_balance);
    s.white_balance_k = get_int(parser, "white_balance_k", s.white_balance_k);
    s.saturation = get_int(parser, "saturation", s.saturation);
    s.gamma = get_int(parser, "gamma", s.gamma);
    s.sharpness = get_int(parser, "sharpness", s.sharpness);
    const std::string banding = parser.get_optional_string(
        "anti_banding", std::string(magic_enum::enum_name(s.anti_banding)));
    const auto parsed = magic_enum::enum_cast<AntiBanding>(banding);
    if (!parsed) {
        std::string valid;
        for (auto name : magic_enum::enum_names<AntiBanding>()) {
            valid += (valid.empty() ? "" : ", ") + std::string(name);
        }
        throw ConfigValidationError(section + ": invalid anti_banding '" + banding +
                                    "' (valid: " + valid + ")");
    }
    s.anti_banding = *parsed;
    parser.validate_no_extra_fields();
    return profile;
}

}  // namespace

std::vector<CameraProfile> parse_camera_profiles(const std::string &toml_text,
                                                 const std::string &source_name) {
    toml::table root;
    try {
        root = toml::parse(toml_text, source_name);
    } catch (const toml::parse_error &e) {
        throw ConfigValidationError(source_name + ": " + std::string(e.description()));
    }
    validate_no_extra_sections(root, {"profile"}, source_name);

    std::vector<CameraProfile> profiles;
    const toml::array *entries = root["profile"].as_array();
    if (entries == nullptr) {
        if (root.contains("profile")) {
            throw ConfigValidationError(source_name + ": 'profile' must be [[profile]] tables");
        }
        return profiles;
    }
    for (std::size_t i = 0; i < entries->size(); ++i) {
        const toml::table *table = (*entries)[i].as_table();
        const std::string section = "profile " + std::to_string(i + 1);
        if (table == nullptr) {
            throw ConfigValidationError(source_name + ": " + section + " is not a table");
        }
        CameraProfile profile = parse_profile(*table, section);
        const bool duplicate = std::any_of(profiles.begin(), profiles.end(),
                                           [&](const auto &p) { return p.name == profile.name; });
        if (duplicate) {
            throw ConfigValidationError(source_name + ": profile '" + profile.name +
                                        "' appears twice");
        }
        profiles.push_back(std::move(profile));
    }
    return profiles;
}

std::string format_camera_profiles(const std::vector<CameraProfile> &profiles) {
    std::ostringstream out;
    out << "# Camera profiles, written by the dashboard. DEFAULT is built in and not listed.\n"
           "# Values are ZED SDK units: exposure in microseconds, analog gain in milli-dB.\n";
    for (const CameraProfile &p : profiles) {
        const CameraSettings &s = p.settings;
        out << "\n[[profile]]\n"
            << "name = \"" << p.name << "\"\n"
            << "auto_exposure = " << (s.auto_exposure ? "true" : "false") << "\n"
            << "exposure_us = " << s.exposure_us << "\n"
            << "analog_gain_mdb = " << s.analog_gain_mdb << "\n"
            << "digital_gain = " << s.digital_gain << "\n"
            << "auto_white_balance = " << (s.auto_white_balance ? "true" : "false") << "\n"
            << "white_balance_k = " << s.white_balance_k << "\n"
            << "saturation = " << s.saturation << "\n"
            << "gamma = " << s.gamma << "\n"
            << "sharpness = " << s.sharpness << "\n"
            << "anti_banding = \"" << magic_enum::enum_name(s.anti_banding) << "\"\n";
    }
    return out.str();
}

CameraProfileStore::CameraProfileStore(std::filesystem::path path) : path_(std::move(path)) {}

void CameraProfileStore::load() {
    profiles_.clear();
    load_error_.clear();
    std::ifstream file(path_);
    if (!file) {
        spdlog::info("[CameraProfileStore] No saved camera profiles at {}", path_.string());
        return;
    }
    std::stringstream text;
    text << file.rdbuf();
    try {
        profiles_ = parse_camera_profiles(text.str(), path_.string());
    } catch (const ConfigValidationError &e) {
        load_error_ = e.what();
        spdlog::error(
            "[CameraProfileStore] {}. Saved camera profiles are unavailable until it is "
            "fixed or removed.",
            load_error_);
        return;
    }
    spdlog::info("[CameraProfileStore] Loaded {} camera profile(s) from {}", profiles_.size(),
                 path_.string());
}

std::vector<std::string> CameraProfileStore::names() const {
    std::vector<std::string> out{kDefaultName};
    for (const CameraProfile &p : profiles_) out.push_back(p.name);
    return out;
}

bool CameraProfileStore::contains(const std::string &name) const {
    return name == kDefaultName || settings(name).has_value();
}

std::optional<CameraSettings> CameraProfileStore::settings(const std::string &name) const {
    for (const CameraProfile &p : profiles_) {
        if (p.name == name) return p.settings;
    }
    return std::nullopt;
}

void CameraProfileStore::save(const std::string &name, const CameraSettings &settings) {
    check_name(name);
    require_writable();
    std::vector<CameraProfile> before = profiles_;
    auto it = std::find_if(profiles_.begin(), profiles_.end(),
                           [&](const CameraProfile &p) { return p.name == name; });
    if (it != profiles_.end()) {
        it->settings = settings;
    } else {
        profiles_.push_back({name, settings});
    }
    try {
        write();
    } catch (...) {
        profiles_ = std::move(before);
        throw;
    }
}

void CameraProfileStore::remove(const std::string &name) {
    check_name(name);
    require_writable();
    auto it = std::find_if(profiles_.begin(), profiles_.end(),
                           [&](const CameraProfile &p) { return p.name == name; });
    if (it == profiles_.end()) {
        throw std::runtime_error("No camera profile named " + name);
    }
    std::vector<CameraProfile> before = profiles_;
    profiles_.erase(it);
    try {
        write();
    } catch (...) {
        profiles_ = std::move(before);
        throw;
    }
}

void CameraProfileStore::require_writable() const {
    if (!load_error_.empty()) {
        throw std::runtime_error(path_.filename().string() +
                                 " did not load, so it is not overwritten: " + load_error_);
    }
}

void CameraProfileStore::write() const {
    std::error_code ec;
    std::filesystem::create_directories(path_.parent_path(), ec);
    // Write beside the target and rename over it, so a power cut mid-save leaves the old file.
    std::filesystem::path tmp = path_;
    tmp += ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        out << format_camera_profiles(profiles_);
        out.flush();
        if (!out) {
            throw std::runtime_error("Could not write " + tmp.string());
        }
    }
    std::filesystem::rename(tmp, path_, ec);
    if (ec) {
        throw std::runtime_error("Could not replace " + path_.string() + ": " + ec.message());
    }
}

}  // namespace auto_battlebot
