#pragma once

namespace auto_battlebot {
template <typename DerivedConfig, typename BaseConfig>
const DerivedConfig &config_cast(const BaseConfig &config) {
    return static_cast<const DerivedConfig &>(config);
}
}  // namespace auto_battlebot
