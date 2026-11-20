#pragma once

namespace auto_battlebot {
template <typename DerivedConfig, typename BaseConfig>
DerivedConfig &config_cast(const BaseConfig &config) {
    const auto &derived = static_cast<const DerivedConfig &>(config);
    return const_cast<DerivedConfig &>(derived);
}
}  // namespace auto_battlebot
