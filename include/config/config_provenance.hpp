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

class ConfigDefaultsRecorder {
   public:
    ConfigDefaultsRecorder();
    ~ConfigDefaultsRecorder();
    ConfigDefaultsRecorder(const ConfigDefaultsRecorder &) = delete;
    ConfigDefaultsRecorder &operator=(const ConfigDefaultsRecorder &) = delete;

    void record(const toml::table &table, const std::string &section_name, const std::string &key,
                std::string value);

    ResolvedConfig resolve(const toml::table &root, const std::filesystem::path &config_root) const;

   private:
    struct RecordedDefault {
        const toml::table *table;
        std::string section_name;
        std::string key;
        std::string value;
    };

    std::vector<RecordedDefault> defaults_;
    ConfigDefaultsRecorder *previous_;
};

void record_config_default(const toml::table &table, const std::string &section_name,
                           const std::string &key, std::string value);
}  // namespace auto_battlebot
