#pragma once

#include <toml++/toml.h>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace auto_battlebot {
inline constexpr const char *kRecordingOverrideSource = "cli --recording";

struct CatalogRecording {
    std::string name;
    std::filesystem::path path;
    int64_t start_frame = 0;
    std::string note;
};

class RecordingCatalog {
   public:
    static RecordingCatalog load(const std::filesystem::path &file);

    const CatalogRecording *find(const std::string &name) const;
    const CatalogRecording *find_by_stem(const std::filesystem::path &path) const;
    const std::vector<CatalogRecording> &entries() const { return entries_; }

   private:
    std::vector<CatalogRecording> entries_;
};

std::filesystem::path recording_catalog_path(const std::filesystem::path &config_root);

struct RecordingOverride {
    std::string recording;
    std::optional<int64_t> start_frame;
};

struct ResolvedRecording {
    std::filesystem::path path;
    int64_t start_frame = 0;
};

ResolvedRecording resolve_recording(const RecordingCatalog &catalog,
                                    const RecordingOverride &request);

void apply_recording_override(toml::table &merged, const std::string &profile,
                              const RecordingCatalog &catalog, const RecordingOverride &request);

std::string format_recording_list(const RecordingCatalog &catalog);
}  // namespace auto_battlebot
