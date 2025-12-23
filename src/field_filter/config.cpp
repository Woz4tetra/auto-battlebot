#include "field_filter/config.hpp"
#include "field_filter/noop_field_filter.hpp"
#include "field_filter/point_cloud_field_filter.hpp"
#include "config/config_parser.hpp"
#include "config/config_cast.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(FieldFilterConfiguration, NoopFieldFilterConfiguration, "NoopFieldFilter")
    REGISTER_CONFIG(FieldFilterConfiguration, PointCloudFieldFilterConfiguration, "PointCloudFieldFilter")

    std::unique_ptr<FieldFilterConfiguration> parse_field_filter_config(ConfigParser &parser)
    {
        return ConfigFactory<FieldFilterConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<FieldFilterInterface> make_field_filter(const FieldFilterConfiguration &config)
    {
        std::cout << "Selected " + config.type + " for FieldFilter" << std::endl;
        if (config.type == "NoopFieldFilter")
        {
            return std::make_shared<NoopFieldFilter>();
        }
        else if (config.type == "PointCloudFieldFilter")
        {
            return std::make_shared<PointCloudFieldFilter>(
                config_cast<PointCloudFieldFilterConfiguration>(config));
        }
        throw std::invalid_argument("Failed to load FieldFilter of type " + config.type);
    }
} // namespace auto_battlebot
