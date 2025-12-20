#include "directories.hpp"

namespace auto_battlebot
{
    std::filesystem::path get_project_root()
    {
        return std::filesystem::current_path();
    }

    std::filesystem::path get_config_dir()
    {
        return get_project_root() / "config";
    }

    std::filesystem::path get_project_path(const std::string &relative_path)
    {
        return get_project_root() / relative_path;
    }

} // namespace auto_battlebot
