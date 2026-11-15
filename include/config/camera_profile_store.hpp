#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "rgbd_camera/camera_settings.hpp"

namespace auto_battlebot {

struct CameraProfile {
    std::string name;
    CameraSettings settings;
};

class CameraProfileStore {
   public:
    static constexpr const char *kDefaultName = "DEFAULT";
    static constexpr const char *kDefaultPath =
        "$HOME/.local/state/auto_battlebot/camera_profiles.toml";

    explicit CameraProfileStore(std::filesystem::path path);

    /** Read the file. A missing file means no saved profiles. A file that fails to parse is
     *  logged and left untouched on disk, and save/remove refuse until it is fixed, so a typo
     *  never costs the profiles in it. */
    void load();

    std::vector<std::string> names() const;
    bool contains(const std::string &name) const;
    std::optional<CameraSettings> settings(const std::string &name) const;

    void save(const std::string &name, const CameraSettings &settings);
    void remove(const std::string &name);

    const std::filesystem::path &path() const { return path_; }

   private:
    void require_writable() const;
    void write() const;

    std::filesystem::path path_;
    std::vector<CameraProfile> profiles_;
    std::string load_error_;
};

std::vector<CameraProfile> parse_camera_profiles(const std::string &toml_text,
                                                 const std::string &source_name);
std::string format_camera_profiles(const std::vector<CameraProfile> &profiles);

}  // namespace auto_battlebot
