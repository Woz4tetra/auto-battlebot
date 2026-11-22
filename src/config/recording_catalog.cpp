#include "config/recording_catalog.hpp"

#include <algorithm>
#include <sstream>

#include "config/config_parser.hpp"

namespace auto_battlebot {
namespace {
constexpr const char *kSvoPlaybackType = "ZedSvoPlaybackCamera";
constexpr const char *kVideoPlaybackType = "VideoPlaybackCamera";
constexpr const char *kSvoPlaybackBase = "playback/_playback";
constexpr const char *kVideoPlaybackBase = "playback/_video_playback";

bool is_svo(const std::filesystem::path &path) {
    const std::string ext = path.extension().string();
    return ext == ".svo" || ext == ".svo2";
}

CatalogRecording parse_entry(const std::filesystem::path &file, const std::string &name,
                             const toml::node &node) {
    const std::string where = file.string() + " [recordings.\"" + name + "\"]";
    const toml::table *table = node.as_table();
    if (!table) {
        throw ConfigValidationError(where + " must be a table");
    }
    CatalogRecording entry;
    entry.name = name;
    for (auto &&[key, value] : *table) {
        if (key.str() == "path") {
            if (!value.is_string()) {
                throw ConfigValidationError(where + ": `path` must be a string");
            }
            entry.path = *value.value<std::string>();
        } else if (key.str() == "start_frame") {
            if (!value.is_integer() || *value.value<int64_t>() < 0) {
                throw ConfigValidationError(where +
                                            ": `start_frame` must be a non-negative integer");
            }
            entry.start_frame = *value.value<int64_t>();
        } else if (key.str() == "note") {
            if (!value.is_string()) {
                throw ConfigValidationError(where + ": `note` must be a string");
            }
            entry.note = *value.value<std::string>();
        } else {
            throw ConfigValidationError(where + ": unknown key '" + std::string(key.str()) +
                                        "' (valid: path, start_frame, note)");
        }
    }
    if (entry.path.empty()) {
        throw ConfigValidationError(where + ": missing `path`");
    }
    return entry;
}

std::string quoted(const std::string &text) {
    std::ostringstream os;
    os << toml::value<std::string>(text);
    return os.str();
}
}  // namespace

RecordingCatalog RecordingCatalog::load(const std::filesystem::path &file) {
    if (!std::filesystem::exists(file)) {
        throw ConfigValidationError("Recording catalog not found at " + file.string());
    }
    const toml::table data = toml::parse_file(file.string());
    RecordingCatalog catalog;
    for (auto &&[key, value] : data) {
        if (key.str() != "recordings") {
            throw ConfigValidationError(file.string() + ": unknown top-level key '" +
                                        std::string(key.str()) + "' (valid: recordings)");
        }
    }
    const toml::table *recordings = data["recordings"].as_table();
    if (!recordings) {
        return catalog;
    }
    for (auto &&[key, value] : *recordings) {
        catalog.entries_.push_back(parse_entry(file, std::string(key.str()), value));
    }
    return catalog;
}

const CatalogRecording *RecordingCatalog::find(const std::string &name) const {
    auto it = std::find_if(entries_.begin(), entries_.end(),
                           [&](const CatalogRecording &entry) { return entry.name == name; });
    return it == entries_.end() ? nullptr : &*it;
}

const CatalogRecording *RecordingCatalog::find_by_stem(const std::filesystem::path &path) const {
    return find(path.stem().string());
}

std::filesystem::path recording_catalog_path(const std::filesystem::path &config_root) {
    return config_root / "playback" / "recordings.toml";
}

ResolvedRecording resolve_recording(const RecordingCatalog &catalog,
                                    const RecordingOverride &request) {
    ResolvedRecording resolved;
    if (const CatalogRecording *entry = catalog.find(request.recording)) {
        resolved.path = entry->path;
        resolved.start_frame = entry->start_frame;
    } else {
        resolved.path = request.recording;
        if (const CatalogRecording *by_stem = catalog.find_by_stem(resolved.path)) {
            resolved.start_frame = by_stem->start_frame;
        }
    }
    if (request.start_frame) {
        resolved.start_frame = *request.start_frame;
    }
    if (!std::filesystem::exists(resolved.path)) {
        throw ConfigValidationError("--recording '" + request.recording + "': no file at " +
                                    resolved.path.string());
    }
    return resolved;
}

void apply_recording_override(toml::table &merged, const std::string &profile,
                              const RecordingCatalog &catalog, const RecordingOverride &request) {
    toml::table *camera = merged["rgbd_camera"].as_table();
    const std::string type = camera ? (*camera)["type"].value_or(std::string{}) : std::string{};
    const bool svo_profile = type == kSvoPlaybackType;
    const bool video_profile = type == kVideoPlaybackType;
    if (!svo_profile && !video_profile) {
        throw ConfigValidationError("--recording needs a playback profile, but '" + profile +
                                    "' has [rgbd_camera] type = \"" + type + "\" (expected " +
                                    kSvoPlaybackType + " or " + kVideoPlaybackType + ")");
    }
    const ResolvedRecording recording = resolve_recording(catalog, request);
    if (svo_profile && !is_svo(recording.path)) {
        throw ConfigValidationError("--recording " + recording.path.string() +
                                    " is not an SVO, but '" + profile + "' is a " +
                                    kSvoPlaybackType + " profile; use a profile that extends " +
                                    kVideoPlaybackBase);
    }
    if (video_profile && is_svo(recording.path)) {
        throw ConfigValidationError("--recording " + recording.path.string() + " is an SVO, but '" +
                                    profile + "' is a " + kVideoPlaybackType +
                                    " profile; use a profile that extends " + kSvoPlaybackBase);
    }
    const char *path_key = svo_profile ? "svo_file_path" : "video_file_path";
    const char *frame_key = svo_profile ? "svo_start_frame" : "start_frame";
    const std::string text = std::string(path_key) + " = " +
                             quoted(recording.path.generic_string()) + "\n" + frame_key + " = " +
                             std::to_string(recording.start_frame) + "\n";
    toml::table overrides = toml::parse(text, std::string_view(kRecordingOverrideSource));
    for (auto &&[key, value] : overrides) {
        value.visit([&](auto &&node) { camera->insert_or_assign(key.str(), std::move(node)); });
    }
}

std::string format_recording_list(const RecordingCatalog &catalog) {
    size_t name_width = 4;
    for (const auto &entry : catalog.entries()) {
        name_width = std::max(name_width, entry.name.size());
    }
    std::ostringstream os;
    auto pad = [](const std::string &text, size_t width) {
        return text + std::string(width > text.size() ? width - text.size() : 0, ' ');
    };
    os << pad("name", name_width) << "  " << pad("start", 6) << "  " << pad("here", 4)
       << "  note\n";
    for (const auto &entry : catalog.entries()) {
        const bool present = std::filesystem::exists(entry.path);
        os << pad(entry.name, name_width) << "  " << pad(std::to_string(entry.start_frame), 6)
           << "  " << pad(present ? "yes" : "no", 4) << "  " << entry.note << "\n";
    }
    return os.str();
}
}  // namespace auto_battlebot
