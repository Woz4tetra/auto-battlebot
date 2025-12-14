#pragma once

#include <string>
#include <filesystem>

namespace auto_battlebot
{

    /**
     * Get the project root directory.
     * Searches upward from the executable location for a directory containing CMakeLists.txt
     * @return Absolute path to the project root directory
     */
    std::filesystem::path get_project_root();

    /**
     * Get the config directory path.
     * @return Absolute path to the config directory (project_root/config)
     */
    std::filesystem::path get_config_dir();

    /**
     * Get a path relative to the project root.
     * @param relative_path Path relative to project root
     * @return Absolute path
     */
    std::filesystem::path get_project_path(const std::string &relative_path);

} // namespace auto_battlebot
