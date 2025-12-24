#pragma once

#include <toml++/toml.h>
#include <string>
#include <vector>
#include <set>
#include <stdexcept>
#include <iostream>

namespace auto_battlebot
{
    class ConfigValidationError : public std::runtime_error
    {
    public:
        explicit ConfigValidationError(const std::string &message)
            : std::runtime_error(message) {}
    };

    /**
     * Validate that a TOML table only contains expected sections
     */
    inline void validate_no_extra_sections(
        const toml::table &toml_data,
        const std::vector<std::string> &expected_sections,
        const std::string &config_name)
    {
        std::vector<std::string> extra_sections;
        std::set<std::string> expected_set(expected_sections.begin(), expected_sections.end());

        for (const auto &[key, value] : toml_data)
        {
            std::string key_str(key.str());
            if (expected_set.find(key_str) == expected_set.end())
            {
                extra_sections.push_back(key_str);
            }
        }

        if (!extra_sections.empty())
        {
            std::string error_msg = "Unknown sections in " + config_name + ": ";
            for (size_t i = 0; i < extra_sections.size(); ++i)
            {
                error_msg += "[" + extra_sections[i] + "]";
                if (i < extra_sections.size() - 1)
                {
                    error_msg += ", ";
                }
            }
            throw ConfigValidationError(error_msg);
        }
    }

    /**
     * Helper class for parsing and validating TOML configuration sections
     */
    class ConfigParser
    {
    public:
        ConfigParser(const toml::table &table, const std::string &section_name)
            : table_(table), section_name_(section_name) {}

        /**
         * Get a required string field
         */
        std::string get_required_string(const std::string &key)
        {
            mark_accessed(key);
            auto value = table_[key].value<std::string>();
            if (!value)
            {
                throw ConfigValidationError(
                    "Missing required field '" + key + "' in section [" + section_name_ + "]");
            }
            return *value;
        }

        /**
         * Get an optional string field with default value
         */
        std::string get_optional_string(const std::string &key, const std::string &default_value)
        {
            mark_accessed(key);
            return table_[key].value_or(default_value);
        }

        /**
         * Get a required integer field
         */
        int64_t get_required_int(const std::string &key)
        {
            mark_accessed(key);
            auto value = table_[key].value<int64_t>();
            if (!value)
            {
                throw ConfigValidationError(
                    "Missing required field '" + key + "' in section [" + section_name_ + "]");
            }
            return *value;
        }

        /**
         * Get an optional integer field with default value
         */
        int64_t get_optional_int(const std::string &key, int64_t default_value)
        {
            mark_accessed(key);
            return table_[key].value_or(default_value);
        }

        /**
         * Get a required double field
         */
        double get_required_double(const std::string &key)
        {
            mark_accessed(key);
            auto value = table_[key].value<double>();
            if (!value)
            {
                throw ConfigValidationError(
                    "Missing required field '" + key + "' in section [" + section_name_ + "]");
            }
            return *value;
        }

        /**
         * Get an optional double field with default value
         */
        double get_optional_double(const std::string &key, double default_value)
        {
            mark_accessed(key);
            return table_[key].value_or(default_value);
        }

        /**
         * Get a required boolean field
         */
        bool get_required_bool(const std::string &key)
        {
            mark_accessed(key);
            auto value = table_[key].value<bool>();
            if (!value)
            {
                throw ConfigValidationError(
                    "Missing required field '" + key + "' in section [" + section_name_ + "]");
            }
            return *value;
        }

        /**
         * Get an optional boolean field with default value
         */
        bool get_optional_bool(const std::string &key, bool default_value)
        {
            mark_accessed(key);
            return table_[key].value_or(default_value);
        }

        /**
         * Get a nested table (for nested configuration sections)
         */
        const toml::table *get_table(const std::string &key)
        {
            mark_accessed(key);
            return table_[key].as_table();
        }

        /**
         * Get a nested array
         */
        const toml::array *get_array(const std::string &key)
        {
            mark_accessed(key);
            return table_[key].as_array();
        }

        /**
         * Check if all fields in the table were accessed
         * Throws if there are unrecognized fields
         */
        void validate_no_extra_fields()
        {
            std::vector<std::string> extra_fields;
            for (const auto &[key, value] : table_)
            {
                std::string key_str(key.str());
                if (accessed_keys_.find(key_str) == accessed_keys_.end())
                {
                    extra_fields.push_back(key_str);
                }
            }

            if (!extra_fields.empty())
            {
                std::string error_msg = "Unknown fields in section [" + section_name_ + "]: ";
                for (size_t i = 0; i < extra_fields.size(); ++i)
                {
                    error_msg += "'" + extra_fields[i] + "'";
                    if (i < extra_fields.size() - 1)
                    {
                        error_msg += ", ";
                    }
                }
                throw ConfigValidationError(error_msg);
            }
        }

    private:
        void mark_accessed(const std::string &key)
        {
            accessed_keys_.insert(key);
        }

        const toml::table &table_;
        std::string section_name_;
        std::set<std::string> accessed_keys_;
    };

} // namespace auto_battlebot
