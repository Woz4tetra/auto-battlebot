#pragma once

namespace auto_battlebot
{
    /**
     * Safely cast a const reference to a base configuration to a non-const reference
     * to a derived configuration type.
     *
     * This is used when a factory function receives a const reference but needs to
     * pass a non-const reference to a constructor that stores the configuration.
     *
     * @tparam DerivedConfig The derived configuration type to cast to
     * @tparam BaseConfig The base configuration type
     * @param config The base configuration reference to cast
     * @return A non-const reference to the derived configuration
     */
    template <typename DerivedConfig, typename BaseConfig>
    DerivedConfig &config_cast(const BaseConfig &config)
    {
        const auto &derived = static_cast<const DerivedConfig &>(config);
        return const_cast<DerivedConfig &>(derived);
    }
} // namespace auto_battlebot
