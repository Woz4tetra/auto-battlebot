#pragma once

#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <string>

namespace auto_battlebot {

inline std::filesystem::path make_scratch_recordings_dir(const std::string &tag) {
    static std::atomic<int> counter{0};
    const std::filesystem::path dir = std::filesystem::temp_directory_path() /
                                      ("auto_battlebot_" + tag + "_" + std::to_string(::getpid()) +
                                       "_" + std::to_string(counter.fetch_add(1)));
    std::filesystem::create_directories(dir);
    return dir;
}

}  // namespace auto_battlebot
