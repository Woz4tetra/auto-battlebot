#pragma once

#include <toml++/toml.h>

#include <filesystem>
#include <string>
#include <vector>

namespace auto_battlebot {
inline constexpr const char *kDefaultSource = "default";
inline constexpr const char *kResolvedConfigMetadataKey = "resolved_config";
inline constexpr const char *kUnsetValue = "";

struct ResolvedField {
    std::string section;
    std::string key;
    std::string value;
    std::string source;
};

struct ResolvedConfig {
    std::vector<ResolvedField> fields;

    const ResolvedField *find(const std::string &section, const std::string &key) const;
};

std::string format_resolved_toml(const ResolvedConfig &resolved);
std::string format_toml_value(const toml::node &node);

enum class RecorderMode { RECORD, PROBE_SCHEMA };

class ConfigDefaultsRecorder {
   public:
    struct RecordedField {
        const toml::table *table;
        std::string section_name;
        std::string key;
        std::string value;
        bool required = false;
    };

    explicit ConfigDefaultsRecorder(RecorderMode mode = RecorderMode::RECORD);
    ~ConfigDefaultsRecorder();
    ConfigDefaultsRecorder(const ConfigDefaultsRecorder &) = delete;
    ConfigDefaultsRecorder &operator=(const ConfigDefaultsRecorder &) = delete;

    void record(const toml::table &table, const std::string &section_name, const std::string &key,
                std::string value, bool required = false);

    RecorderMode mode() const { return mode_; }
    const std::vector<RecordedField> &recorded() const { return defaults_; }

    ResolvedConfig resolve(const toml::table &root, const std::filesystem::path &config_root) const;

   private:
    RecorderMode mode_;
    std::vector<RecordedField> defaults_;
    ConfigDefaultsRecorder *previous_;
};

void record_config_default(const toml::table &table, const std::string &section_name,
                           const std::string &key, std::string value);

bool probe_required_field(const toml::table &table, const std::string &section_name,
                          const std::string &key);
}  // namespace auto_battlebot
