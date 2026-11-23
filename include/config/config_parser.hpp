#pragma once

#include <toml++/toml.h>

#include <cstdint>
#include <iostream>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/config_provenance.hpp"

namespace auto_battlebot {
class ConfigValidationError : public std::runtime_error {
   public:
    explicit ConfigValidationError(const std::string &message) : std::runtime_error(message) {}
};

inline void validate_no_extra_sections(const toml::table &toml_data,
                                       const std::vector<std::string> &expected_sections,
                                       const std::string &config_name) {
    std::vector<std::string> extra_sections;
    std::set<std::string> expected_set(expected_sections.begin(), expected_sections.end());

    for (const auto &[key, value] : toml_data) {
        std::string key_str(key.str());
        if (expected_set.find(key_str) == expected_set.end()) {
            extra_sections.push_back(key_str);
        }
    }

    if (!extra_sections.empty()) {
        std::string error_msg = "Unknown sections in " + config_name + ": ";
        for (size_t i = 0; i < extra_sections.size(); ++i) {
            error_msg += "[" + extra_sections[i] + "]";
            if (i < extra_sections.size() - 1) {
                error_msg += ", ";
            }
        }
        throw ConfigValidationError(error_msg);
    }
}

inline void require_positive_if_set(const std::optional<double> &value, const std::string &key) {
    if (value && *value <= 0.0) {
        throw ConfigValidationError("'" + key +
                                    "' must be > 0 when set; leave it out to turn it off");
    }
}

class ConfigParser {
   public:
    ConfigParser(const toml::table &table, const std::string &section_name)
        : table_(table), section_name_(section_name) {}

    std::string get_required_string(const std::string &key, const std::string &probe_value = "") {
        mark_accessed(key);
        auto value = table_[key].value<std::string>();
        if (!value) {
            if (probe_required_field(table_, section_name_, key)) return probe_value;
            throw ConfigValidationError("Missing required field '" + key + "' in section [" +
                                        section_name_ + "]");
        }
        return *value;
    }

    std::string get_optional_string(const std::string &key, const std::string &default_value) {
        mark_accessed(key);
        record_default_if_absent(key, toml::value<std::string>(default_value));
        return table_[key].value_or(default_value);
    }

    int64_t get_required_int(const std::string &key) {
        mark_accessed(key);
        auto value = table_[key].value<int64_t>();
        if (!value) {
            if (probe_required_field(table_, section_name_, key)) return int64_t{};
            throw ConfigValidationError("Missing required field '" + key + "' in section [" +
                                        section_name_ + "]");
        }
        return *value;
    }

    int64_t get_optional_int(const std::string &key, int64_t default_value) {
        mark_accessed(key);
        record_default_if_absent(key, toml::value<int64_t>(default_value));
        return table_[key].value_or(default_value);
    }

    double get_required_double(const std::string &key) {
        mark_accessed(key);
        auto value = table_[key].value<double>();
        if (!value) {
            if (probe_required_field(table_, section_name_, key)) return double{};
            throw ConfigValidationError("Missing required field '" + key + "' in section [" +
                                        section_name_ + "]");
        }
        return *value;
    }

    double get_optional_double(const std::string &key, double default_value) {
        mark_accessed(key);
        record_default_if_absent(key, toml::value<double>(default_value));
        return table_[key].value_or(default_value);
    }

    bool get_required_bool(const std::string &key) {
        mark_accessed(key);
        auto value = table_[key].value<bool>();
        if (!value) {
            if (probe_required_field(table_, section_name_, key)) return bool{};
            throw ConfigValidationError("Missing required field '" + key + "' in section [" +
                                        section_name_ + "]");
        }
        return *value;
    }

    bool get_optional_bool(const std::string &key, bool default_value) {
        mark_accessed(key);
        record_default_if_absent(key, toml::value<bool>(default_value));
        return table_[key].value_or(default_value);
    }

    std::optional<double> get_nullable_double(const std::string &key,
                                              std::optional<double> default_value) {
        return get_nullable<double>(key, default_value, "a number");
    }

    std::optional<int64_t> get_nullable_int(const std::string &key,
                                            std::optional<int64_t> default_value) {
        return get_nullable<int64_t>(key, default_value, "an integer");
    }

    void reject_renamed(const std::string &old_key, const std::string &new_key) const {
        if (table_.contains(old_key)) {
            throw ConfigValidationError("Field '" + old_key + "' in section [" + section_name_ +
                                        "] is renamed to '" + new_key + "'");
        }
    }

    template <typename T>
    std::vector<T> get_required_vector(const std::string &key) {
        mark_accessed(key);
        const toml::array *arr = table_[key].as_array();
        if (!arr) {
            if (probe_required_field(table_, section_name_, key)) return {};
            throw ConfigValidationError("Missing required field '" + key + "' in section [" +
                                        section_name_ + "] (expected array)");
        }
        std::vector<T> result;
        result.reserve(arr->size());
        for (size_t i = 0; i < arr->size(); ++i) {
            auto value = (*arr)[i].template value<T>();
            if (!value) {
                throw ConfigValidationError("Invalid element type in field '" + key +
                                            "' at index " + std::to_string(i) + " in section [" +
                                            section_name_ + "]");
            }
            result.push_back(*value);
        }
        return result;
    }

    template <typename T>
    std::vector<T> get_optional_vector(const std::string &key, std::vector<T> default_value = {}) {
        mark_accessed(key);
        const toml::array *arr = table_[key].as_array();
        if (!arr) {
            toml::array recorded;
            for (const auto &element : default_value) {
                recorded.push_back(element);
            }
            record_default_if_absent(key, recorded);
            return default_value;
        }
        std::vector<T> result;
        result.reserve(arr->size());
        for (size_t i = 0; i < arr->size(); ++i) {
            auto value = (*arr)[i].template value<T>();
            if (!value) {
                throw ConfigValidationError("Invalid element type in field '" + key +
                                            "' at index " + std::to_string(i) + " in section [" +
                                            section_name_ + "]");
            }
            result.push_back(*value);
        }
        return result;
    }

    const toml::table *get_table(const std::string &key) {
        mark_accessed(key);
        return table_[key].as_table();
    }

    const toml::array *get_array(const std::string &key) {
        mark_accessed(key);
        return table_[key].as_array();
    }

    void validate_no_extra_fields() {
        std::vector<std::string> extra_fields;
        for (const auto &[key, value] : table_) {
            std::string key_str(key.str());
            if (accessed_keys_.find(key_str) == accessed_keys_.end()) {
                extra_fields.push_back(key_str);
            }
        }

        if (!extra_fields.empty()) {
            std::string error_msg = "Unknown fields in section [" + section_name_ + "]: ";
            for (size_t i = 0; i < extra_fields.size(); ++i) {
                error_msg += "'" + extra_fields[i] + "'";
                if (i < extra_fields.size() - 1) {
                    error_msg += ", ";
                }
            }
            throw ConfigValidationError(error_msg);
        }
    }

   private:
    void mark_accessed(const std::string &key) { accessed_keys_.insert(key); }

    void record_default_if_absent(const std::string &key, const toml::node &value) {
        if (!table_.contains(key)) {
            record_config_default(table_, section_name_, key, format_toml_value(value));
        }
    }

    template <typename T>
    std::optional<T> get_nullable(const std::string &key, std::optional<T> default_value,
                                  const char *expected) {
        mark_accessed(key);
        const toml::node *node = table_.get(key);
        if (!node) {
            record_config_default(table_, section_name_, key,
                                  default_value ? format_toml_value(toml::value<T>(*default_value))
                                                : std::string(kUnsetValue));
            return default_value;
        }
        std::optional<T> value = node->value<T>();
        if (!value) {
            throw ConfigValidationError("Field '" + key + "' in section [" + section_name_ +
                                        "] must be " + expected);
        }
        return value;
    }

    const toml::table &table_;
    std::string section_name_;
    std::set<std::string> accessed_keys_;
};

}  // namespace auto_battlebot
