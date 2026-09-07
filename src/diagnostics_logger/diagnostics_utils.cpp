#include "diagnostics_logger/diagnostics_utils.hpp"

namespace auto_battlebot {

std::string diagnostic_scalar_to_string(const DiagnosticScalar &value) {
    return std::visit(
        [](auto &&arg) -> std::string {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, std::string>) {
                return arg;
            } else {
                return std::to_string(arg);
            }
        },
        value);
}

namespace {

void flatten_recursive(const DiagnosticsData &data, std::map<std::string, DiagnosticScalar> &result,
                       const std::string &parent_key, const std::string &separator) {
    for (const auto &[key, value] : data) {
        const std::string new_key = parent_key.empty() ? key : parent_key + separator + key;
        std::visit(
            [&](auto &&arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, DiagnosticsData>) {
                    flatten_recursive(arg, result, new_key, separator);
                } else if constexpr (std::is_same_v<T, std::vector<int>> ||
                                     std::is_same_v<T, std::vector<double>> ||
                                     std::is_same_v<T, std::vector<std::string>>) {
                    for (size_t i = 0; i < arg.size(); ++i) {
                        result[new_key + separator + std::to_string(i)] = arg[i];
                    }
                } else {
                    result[new_key] = arg;
                }
            },
            static_cast<const DiagnosticsValue::variant &>(value));
    }
}

}  // namespace

std::map<std::string, DiagnosticScalar> flatten_diagnostics_data(const DiagnosticsData &data,
                                                                 const std::string &parent_key,
                                                                 const std::string &separator) {
    std::map<std::string, DiagnosticScalar> result;
    flatten_recursive(data, result, parent_key, separator);
    return result;
}

}  // namespace auto_battlebot
