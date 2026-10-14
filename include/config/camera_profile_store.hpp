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

/**
 * Named camera profiles saved on this box, in one TOML file under $HOME so a deploy never
 * clobbers venue tuning. DEFAULT is built in, always listed first, and means "what the camera
 * opened with"; it is never written to the file. Nothing records which profile was active: the
 * app starts on DEFAULT every time.
 */
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

    /** DEFAULT, then the saved profiles in file order. */
    std::vector<std::string> names() const;
    bool contains(const std::string &name) const;
    /** Saved settings for `name`; nullopt for DEFAULT and unknown names. */
    std::optional<CameraSettings> settings(const std::string &name) const;

    /** Create or overwrite `name` and write the file. Throws std::runtime_error with a message
     *  fit for the dashboard on a bad name, DEFAULT, an unloadable file, or a failed write. */
    void save(const std::string &name, const CameraSettings &settings);
    /** Delete `name` and write the file. Throws like save(), and on an unknown name. */
    void remove(const std::string &name);

    const std::filesystem::path &path() const { return path_; }

   private:
    void require_writable() const;
    void write() const;

    std::filesystem::path path_;
    std::vector<CameraProfile> profiles_;
    std::string load_error_;
};

/** Parse a camera profiles file's text. Throws ConfigValidationError on any bad field. */
std::vector<CameraProfile> parse_camera_profiles(const std::string &toml_text,
                                                 const std::string &source_name);
/** Serialize profiles in the format parse_camera_profiles reads. */
std::string format_camera_profiles(const std::vector<CameraProfile> &profiles);

}  // namespace auto_battlebot
