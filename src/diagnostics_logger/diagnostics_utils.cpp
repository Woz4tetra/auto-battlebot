#include "diagnostics_logger/diagnostics_utils.hpp"
#include <diagnostic_msgs/KeyValue.hxx>
#include <sstream>

namespace auto_battlebot
{
    namespace
    {
        // Helper to convert DiagnosticsValue to string
        std::string value_to_string(const DiagnosticsValue &value)
        {
            return std::visit([](auto &&arg) -> std::string
                              {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, int>) {
                    return std::to_string(arg);
                } else if constexpr (std::is_same_v<T, double>) {
                    return std::to_string(arg);
                } else if constexpr (std::is_same_v<T, std::string>) {
                    return arg;
                } else if constexpr (std::is_same_v<T, std::vector<int>>) {
                    // Should not reach here as vectors are handled separately
                    return "";
                } else if constexpr (std::is_same_v<T, std::vector<double>>) {
                    return "";
                } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
                    return "";
                } else if constexpr (std::is_same_v<T, DiagnosticsData>) {
                    // Should not reach here as nested maps are handled separately
                    return "";
                } }, value);
        }

        // Helper to check if value is a vector type
        bool is_vector_type(const DiagnosticsValue &value)
        {
            return std::holds_alternative<std::vector<int>>(value) ||
                   std::holds_alternative<std::vector<double>>(value) ||
                   std::holds_alternative<std::vector<std::string>>(value);
        }

        // Helper to check if value is a nested map
        bool is_nested_map(const DiagnosticsValue &value)
        {
            return std::holds_alternative<DiagnosticsData>(value);
        }

        // Helper to get vector size
        size_t get_vector_size(const DiagnosticsValue &value)
        {
            if (std::holds_alternative<std::vector<int>>(value))
            {
                return std::get<std::vector<int>>(value).size();
            }
            else if (std::holds_alternative<std::vector<double>>(value))
            {
                return std::get<std::vector<double>>(value).size();
            }
            else if (std::holds_alternative<std::vector<std::string>>(value))
            {
                return std::get<std::vector<std::string>>(value).size();
            }
            return 0;
        }

        // Helper to get vector element as string
        std::string get_vector_element(const DiagnosticsValue &value, size_t index)
        {
            if (std::holds_alternative<std::vector<int>>(value))
            {
                return std::to_string(std::get<std::vector<int>>(value)[index]);
            }
            else if (std::holds_alternative<std::vector<double>>(value))
            {
                return std::to_string(std::get<std::vector<double>>(value)[index]);
            }
            else if (std::holds_alternative<std::vector<std::string>>(value))
            {
                return std::get<std::vector<std::string>>(value)[index];
            }
            return "";
        }

        void flatten_recursive(
            const DiagnosticsData &data,
            std::map<std::string, std::string> &result,
            const std::string &parent_key,
            const std::string &separator)
        {
            for (const auto &[key, value] : data)
            {
                std::string new_key = parent_key.empty() ? key : parent_key + separator + key;

                if (is_nested_map(value))
                {
                    // Recursively flatten nested map
                    const auto &nested_data = std::get<DiagnosticsData>(value);
                    flatten_recursive(nested_data, result, new_key, separator);
                }
                else if (is_vector_type(value))
                {
                    // Flatten vector/array elements
                    size_t size = get_vector_size(value);
                    for (size_t i = 0; i < size; ++i)
                    {
                        std::string element_key = new_key + separator + std::to_string(i);
                        result[element_key] = get_vector_element(value, i);
                    }
                }
                else
                {
                    // Simple value
                    result[new_key] = value_to_string(value);
                }
            }
        }
    }

    std::map<std::string, std::string> flatten_diagnostics_data(
        const DiagnosticsData &data,
        const std::string &parent_key,
        const std::string &separator)
    {
        std::map<std::string, std::string> result;
        flatten_recursive(data, result, parent_key, separator);
        return result;
    }

    diagnostic_msgs::DiagnosticStatus dict_to_diagnostics(
        const DiagnosticsData &data,
        int8_t level,
        const std::string &name,
        const std::string &message,
        const std::string &hardware_id)
    {
        diagnostic_msgs::DiagnosticStatus status;
        status.level = level;
        status.name = name;
        status.message = message;
        status.hardware_id = hardware_id;

        // Flatten the data and convert to KeyValue pairs
        auto flattened = flatten_diagnostics_data(data);
        for (const auto &[key, value] : flattened)
        {
            diagnostic_msgs::KeyValue kv;
            kv.key = key;
            kv.value = value;
            status.values.push_back(kv);
        }

        return status;
    }

} // namespace auto_battlebot
