#pragma once

#include <cstdint>
#include <magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace auto_battlebot::remote {

class SchemaFields;

namespace detail {

template <typename T>
struct is_optional : std::false_type {};
template <typename T>
struct is_optional<std::optional<T>> : std::true_type {};

template <typename T>
struct is_vector : std::false_type {};
template <typename T>
struct is_vector<std::vector<T>> : std::true_type {};

template <typename T>
concept Message = requires(SchemaFields& f) { T::describe(f); };

std::string lowercase(std::string_view text);

/** Every supported field kind, checked once here so encode, decode, and schema_of agree. */
template <typename T>
constexpr void check_field_type() {
    if constexpr (is_optional<T>::value || is_vector<T>::value) {
        check_field_type<typename T::value_type>();
    } else {
        static_assert(!std::is_same_v<T, uint64_t>,
                      "uint64_t does not fit a JSON number above 2^53; send it as a string");
        static_assert(std::is_same_v<T, bool> || std::is_enum_v<T> || std::is_integral_v<T> ||
                          std::is_floating_point_v<T> || std::is_same_v<T, std::string> ||
                          Message<T>,
                      "unsupported remote message field type");
    }
}

template <typename T>
nlohmann::json encode(const T& value) {
    if constexpr (std::is_enum_v<T>) {
        return lowercase(magic_enum::enum_name(value));
    } else if constexpr (is_vector<T>::value) {
        nlohmann::json out = nlohmann::json::array();
        for (const auto& item : value) out.push_back(encode(item));
        return out;
    } else {
        return nlohmann::json(value);
    }
}

[[noreturn]] void throw_type_error(std::string_view expected, const nlohmann::json& got);

/** Stricter than nlohmann's get<T>: a string never becomes a number, 1.5 never becomes 1, and a
 *  number never becomes a bool. A command with a wrong type is refused, not coerced. */
template <typename T>
void decode(const nlohmann::json& j, T& out) {
    if constexpr (std::is_same_v<T, bool>) {
        if (!j.is_boolean()) throw_type_error("boolean", j);
        out = j.get<bool>();
    } else if constexpr (std::is_enum_v<T>) {
        if (!j.is_string()) throw_type_error("string", j);
        auto value = magic_enum::enum_cast<T>(j.get<std::string>(), magic_enum::case_insensitive);
        if (!value) throw std::invalid_argument("unknown value '" + j.get<std::string>() + "'");
        out = *value;
    } else if constexpr (std::is_integral_v<T>) {
        if (!j.is_number_integer()) throw_type_error("integer", j);
        out = j.get<T>();
    } else if constexpr (std::is_floating_point_v<T>) {
        if (!j.is_number()) throw_type_error("number", j);
        out = j.get<T>();
    } else if constexpr (std::is_same_v<T, std::string>) {
        if (!j.is_string()) throw_type_error("string", j);
        out = j.get<std::string>();
    } else if constexpr (is_vector<T>::value) {
        if (!j.is_array()) throw_type_error("array", j);
        out.clear();
        for (const auto& item : j) decode(item, out.emplace_back());
    } else {
        if (!j.is_object()) throw_type_error("object", j);
        from_json(j, out);
    }
}

template <typename T>
void write_field(nlohmann::json& j, const char* name, const T& value) {
    check_field_type<T>();
    if constexpr (is_optional<T>::value) {
        if (value) j[name] = encode(*value);
    } else {
        j[name] = encode(value);
    }
}

template <typename T>
void read_field(const nlohmann::json& j, const char* name, T& value) {
    if constexpr (is_optional<T>::value) {
        auto it = j.find(name);
        if (it == j.end() || it->is_null()) {
            value.reset();
        } else {
            decode(*it, value.emplace());
        }
    } else {
        auto it = j.find(name);
        if (it == j.end()) throw std::invalid_argument(std::string("missing field '") + name + "'");
        try {
            decode(*it, value);
        } catch (const std::exception& e) {
            throw std::invalid_argument(std::string("field '") + name + "': " + e.what());
        }
    }
}

}  // namespace detail

/** JSON Schema for one field type. Mirrors detail::encode. */
template <typename T>
nlohmann::json schema_of();

/** Collects one message's properties for its JSON Schema; filled by the struct's describe(). */
class SchemaFields {
   public:
    template <typename T>
    void add(const char* name) {
        detail::check_field_type<T>();
        if constexpr (detail::is_optional<T>::value) {
            properties_[name] = schema_of<typename T::value_type>();
        } else {
            properties_[name] = schema_of<T>();
            required_.emplace_back(name);
        }
    }

    nlohmann::json object_schema() const;

   private:
    nlohmann::json properties_ = nlohmann::json::object();
    std::vector<std::string> required_;
};

template <typename T>
nlohmann::json schema_of() {
    detail::check_field_type<T>();
    if constexpr (std::is_same_v<T, bool>) {
        return {{"type", "boolean"}};
    } else if constexpr (std::is_enum_v<T>) {
        nlohmann::json names = nlohmann::json::array();
        for (auto name : magic_enum::enum_names<T>()) names.push_back(detail::lowercase(name));
        return {{"type", "string"}, {"enum", names}};
    } else if constexpr (std::is_integral_v<T>) {
        return {{"type", "integer"}};
    } else if constexpr (std::is_floating_point_v<T>) {
        return {{"type", "number"}};
    } else if constexpr (std::is_same_v<T, std::string>) {
        return {{"type", "string"}};
    } else if constexpr (detail::is_vector<T>::value) {
        return {{"type", "array"}, {"items", schema_of<typename T::value_type>()}};
    } else {
        SchemaFields fields;
        T::describe(fields);
        nlohmann::json out = fields.object_schema();
        out["title"] = std::string(T::kSchemaName);
        return out;
    }
}

/** Schema text as a channel advertises it. */
template <detail::Message T>
std::string schema_for() {
    return schema_of<T>().dump();
}

template <detail::Message T>
std::string to_json_string(const T& message) {
    nlohmann::json j;
    to_json(j, message);
    return j.dump();
}

}  // namespace auto_battlebot::remote

#define AB_JSON_DETAIL_TO(field) \
    ::auto_battlebot::remote::detail::write_field(ab_json_j, #field, ab_json_t.field);
#define AB_JSON_DETAIL_FROM(field) \
    ::auto_battlebot::remote::detail::read_field(ab_json_j, #field, ab_json_t.field);
#define AB_JSON_DETAIL_DESCRIBE(field) ab_json_f.add<decltype(field)>(#field);

/**
 * Declares a remote message's fields once. Expands to the schema name, to_json/from_json, and a
 * describe() that feeds the JSON Schema builder. Missing required fields throw on parse, so a
 * command with no `count` is refused instead of silently using the default.
 */
#define AB_JSON_MESSAGE(Type, schema_name, ...)                                         \
    static constexpr std::string_view kSchemaName = schema_name;                        \
    friend void to_json(nlohmann::json& ab_json_j, const Type& ab_json_t) {             \
        ab_json_j = nlohmann::json::object();                                           \
        NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(AB_JSON_DETAIL_TO, __VA_ARGS__))       \
    }                                                                                   \
    friend void from_json(const nlohmann::json& ab_json_j, Type& ab_json_t) {           \
        if (!ab_json_j.is_object())                                                     \
            ::auto_battlebot::remote::detail::throw_type_error("object", ab_json_j);    \
        NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(AB_JSON_DETAIL_FROM, __VA_ARGS__))     \
    }                                                                                   \
    static void describe(::auto_battlebot::remote::SchemaFields& ab_json_f) {           \
        NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(AB_JSON_DETAIL_DESCRIBE, __VA_ARGS__)) \
    }

/** A message with no fields. Writes `{}` and accepts any object. */
#define AB_JSON_EMPTY_MESSAGE(Type, schema_name)                                     \
    static constexpr std::string_view kSchemaName = schema_name;                     \
    friend void to_json(nlohmann::json& ab_json_j, const Type&) {                    \
        ab_json_j = nlohmann::json::object();                                        \
    }                                                                                \
    friend void from_json(const nlohmann::json& ab_json_j, Type&) {                  \
        if (!ab_json_j.is_object())                                                  \
            ::auto_battlebot::remote::detail::throw_type_error("object", ab_json_j); \
    }                                                                                \
    static void describe(::auto_battlebot::remote::SchemaFields&) {}
