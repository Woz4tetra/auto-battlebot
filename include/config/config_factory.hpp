#pragma once

#include "config/config_parser.hpp"
#include <memory>
#include <unordered_map>
#include <functional>
#include <string>
#include <magic_enum.hpp>

namespace auto_battlebot
{
    /**
     * Generic factory for configuration types with automatic registration
     */
    template <typename BaseConfig>
    class ConfigFactory
    {
    public:
        using Creator = std::function<std::unique_ptr<BaseConfig>()>;
        using Parser = std::function<void(BaseConfig &, ConfigParser &)>;

        static ConfigFactory &instance()
        {
            static ConfigFactory factory;
            return factory;
        }

        void register_type(const std::string &type_name, Creator creator, Parser parser)
        {
            creators_[type_name] = creator;
            parsers_[type_name] = parser;
        }

        std::unique_ptr<BaseConfig> create(const std::string &type_name) const
        {
            auto it = creators_.find(type_name);
            if (it == creators_.end())
            {
                throw std::invalid_argument("Unknown configuration type: " + type_name);
            }
            return it->second();
        }

        void parse(BaseConfig &config, ConfigParser &parser) const
        {
            auto it = parsers_.find(config.type);
            if (it == parsers_.end())
            {
                throw std::invalid_argument("No parser registered for type: " + config.type);
            }
            it->second(config, parser);
        }

        std::unique_ptr<BaseConfig> create_and_parse(ConfigParser &parser) const
        {
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

    /**
     * Helper class for automatic registration at static initialization time
     */
    template <typename BaseConfig, typename DerivedConfig>
    class ConfigRegistrar
    {
    public:
        ConfigRegistrar(const std::string &type_name)
        {
            ConfigFactory<BaseConfig>::instance().register_type(
                type_name,
                []() -> std::unique_ptr<BaseConfig>
                {
                    return std::make_unique<DerivedConfig>();
                },
                [](BaseConfig &base_config, ConfigParser &parser)
                {
                    auto &config = static_cast<DerivedConfig &>(base_config);
                    config.parse_fields(parser);
                });
        }
    };

} // namespace auto_battlebot

/**
 * Macro to simplify configuration registration
 * Usage: REGISTER_CONFIG(BaseType, DerivedType, "TypeName")
 */
#define REGISTER_CONFIG(BaseType, DerivedType, TypeName)          \
    static auto_battlebot::ConfigRegistrar<BaseType, DerivedType> \
        registrar_##DerivedType(TypeName);

/**
 * Macro to define the parse_fields method with automatic field parsing
 * Usage in derived config struct:
 *
 * struct ZedRgbdCameraConfiguration : public RgbdCameraConfiguration {
 *     int camera_fps = 30;
 *     PARSE_CONFIG_FIELDS(
 *         PARSE_FIELD(camera_fps)
 *     )
 * };
 */
#define PARSE_CONFIG_FIELDS(...)                                             \
    void parse_fields([[maybe_unused]] auto_battlebot::ConfigParser &parser) \
    {                                                                        \
        __VA_ARGS__                                                          \
    }

#define PARSE_FIELD(field_name) \
    field_name = parser.get_optional_int(#field_name, field_name);

#define PARSE_FIELD_STRING(field_name) \
    field_name = parser.get_optional_string(#field_name, field_name);

#define PARSE_FIELD_DOUBLE(field_name) \
    field_name = parser.get_optional_double(#field_name, field_name);

#define PARSE_FIELD_BOOL(field_name) \
    field_name = parser.get_optional_bool(#field_name, field_name);

#define PARSE_FIELD_REQUIRED(field_name) \
    field_name = parser.get_required_int(#field_name);

#define PARSE_FIELD_STRING_REQUIRED(field_name) \
    field_name = parser.get_required_string(#field_name);

#define PARSE_FIELD_DOUBLE_REQUIRED(field_name) \
    field_name = parser.get_required_double(#field_name);

#define PARSE_FIELD_BOOL_REQUIRED(field_name) \
    field_name = parser.get_required_bool(#field_name);

/**
 * Enum parsing macros using magic_enum for automatic string conversion
 * Enums are represented as strings in TOML and automatically converted
 * No manual string conversion functions needed!
 *
 * Usage:
 * PARSE_ENUM(robot_label, Label)
 * PARSE_ENUM_REQUIRED(robot_group, Group)
 */
#define PARSE_ENUM(field_name, EnumType)                                                                           \
    {                                                                                                              \
        auto default_str = std::string(magic_enum::enum_name(field_name));                                         \
        auto str = parser.get_optional_string(#field_name, default_str);                                           \
        auto enum_val = magic_enum::enum_cast<EnumType>(str);                                                      \
        if (!enum_val.has_value())                                                                                 \
        {                                                                                                          \
            throw std::invalid_argument("Invalid " #EnumType " value: '" + str + "' for field '" #field_name "'"); \
        }                                                                                                          \
        field_name = enum_val.value();                                                                             \
    }

#define PARSE_ENUM_REQUIRED(field_name, EnumType)                                                                  \
    {                                                                                                              \
        auto str = parser.get_required_string(#field_name);                                                        \
        auto enum_val = magic_enum::enum_cast<EnumType>(str);                                                      \
        if (!enum_val.has_value())                                                                                 \
        {                                                                                                          \
            throw std::invalid_argument("Invalid " #EnumType " value: '" + str + "' for field '" #field_name "'"); \
        }                                                                                                          \
        field_name = enum_val.value();                                                                             \
    }
