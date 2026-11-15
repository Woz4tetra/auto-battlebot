#pragma once

#include <filesystem>
#include <string>

namespace auto_battlebot {

std::filesystem::path get_project_root();

std::filesystem::path get_config_dir();

std::filesystem::path get_project_path(const std::string &relative_path);

}  // namespace auto_battlebot
