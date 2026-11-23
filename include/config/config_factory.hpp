#pragma once

#include <algorithm>
#include <functional>
#include <magic_enum.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "config/config_parser.hpp"

namespace auto_battlebot {
template <typename BaseConfig>
class ConfigFactory {
   public:
    using Creator = std::function<std::unique_ptr<BaseConfig>()>;
    using Parser = std::function<void(BaseConfig &, ConfigParser &)>;

    static ConfigFactory &instance() {
        static ConfigFactory factory;
        return factory;
    }

    void register_type(const std::string &type_name, Creator creator, Parser parser) {
        creators_[type_name] = creator;
        parsers_[type_name] = parser;
    }

    std::vector<std::string> type_names() const {
        std::vector<std::string> names;
        for (const auto &[name, creator] : creators_) names.push_back(name);
        std::sort(names.begin(), names.end());
        return names;
    }

    std::unique_ptr<BaseConfig> create(const std::string &type_name) const {
        auto it = creators_.find(type_name);
        if (it == creators_.end()) {
            throw std::invalid_argument("Unknown configuration type: " + type_name);
        }
        return it->second();
    }

    void parse(BaseConfig &config, ConfigParser &parser) const {
        auto it = parsers_.find(config.type);
        if (it == parsers_.end()) {
            throw std::invalid_argument("No parser registered for type: " + config.type);
        }
        it->second(config, parser);
    }

    std::unique_ptr<BaseConfig> create_and_parse(ConfigParser &parser) const {
        std::string type = parser.get_required_string("type");
        auto config = create(type);
        parse(*config, parser);
        parser.validate_no_extra_fields();
        return config;
    }

   private:
    std::unordered_map<std::string, Creator> creators_;
    std::unordered_map<std::string, Parser> parsers_;
};

template <typename BaseConfig, typename DerivedConfig>
class ConfigRegistrar {
   public:
    explicit ConfigRegistrar(const std::string &type_name) {
        ConfigFactory<BaseConfig>::instance().register_type(
            type_name,
            []() -> std::unique_ptr<BaseConfig> { return std::make_unique<DerivedConfig>(); },
            [](BaseConfig &base_config, ConfigParser &parser) {
                auto &config = static_cast<DerivedConfig &>(base_config);
                config.parse_fields(parser);
            });
    }
};

}  // namespace auto_battlebot

#define REGISTER_CONFIG(BaseType, DerivedType, TypeName) \
    static auto_battlebot::ConfigRegistrar<BaseType, DerivedType> registrar_##DerivedType(TypeName);

#define PARSE_CONFIG_FIELDS(...) \
    void parse_fields([[maybe_unused]] auto_battlebot::ConfigParser &parser) { __VA_ARGS__ }

#define PARSE_FIELD(field_name) field_name = parser.get_optional_int(#field_name, field_name);

#define PARSE_FIELD_STRING(field_name) \
    field_name = parser.get_optional_string(#field_name, field_name);

#define PARSE_FIELD_DOUBLE(field_name) \
    field_name = parser.get_optional_double(#field_name, field_name);

#define PARSE_FIELD_BOOL(field_name) field_name = parser.get_optional_bool(#field_name, field_name);

#define PARSE_FIELD_NULLABLE_DOUBLE(field_name)                       \
    field_name = parser.get_nullable_double(#field_name, field_name); \
    require_positive_if_set(field_name, #field_name);

#define PARSE_RENAMED(old_name, new_name) parser.reject_renamed(#old_name, #new_name);

#define PARSE_FIELD_REQUIRED(field_name) field_name = parser.get_required_int(#field_name);

#define PARSE_FIELD_STRING_REQUIRED(field_name) \
    field_name = parser.get_required_string(#field_name);

#define PARSE_FIELD_DOUBLE_REQUIRED(field_name) \
    field_name = parser.get_required_double(#field_name);

#define PARSE_FIELD_BOOL_REQUIRED(field_name) field_name = parser.get_required_bool(#field_name);

#define PARSE_ENUM(field_name, EnumType)                                         \
    {                                                                            \
        auto default_str = std::string(magic_enum::enum_name(field_name));       \
        auto str = parser.get_optional_string(#field_name, default_str);         \
        auto enum_val = magic_enum::enum_cast<EnumType>(str);                    \
        if (!enum_val.has_value()) {                                             \
            throw std::invalid_argument("Invalid " #EnumType " value: '" + str + \
                                        "' for field '" #field_name "'");        \
        }                                                                        \
        field_name = enum_val.value();                                           \
    }

#define PARSE_ENUM_REQUIRED(field_name, EnumType)                                  \
    {                                                                              \
        auto str = parser.get_required_string(                                     \
            #field_name, std::string(magic_enum::enum_names<EnumType>().front())); \
        auto enum_val = magic_enum::enum_cast<EnumType>(str);                      \
        if (!enum_val.has_value()) {                                               \
            throw std::invalid_argument("Invalid " #EnumType " value: '" + str +   \
                                        "' for field '" #field_name "'");          \
        }                                                                          \
        field_name = enum_val.value();                                             \
    }
