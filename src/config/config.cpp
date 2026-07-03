#include "config/config.hpp"

#include <spdlog/spdlog.h>

#include <filesystem>
#include <system_error>

namespace auto_battlebot {
namespace {
// Deep-copy an overlay node into dst[key], replacing whatever is there.
void assign_copy(toml::table &dst, std::string_view key, const toml::node &src) {
    src.visit([&](auto &&node) { dst.insert_or_assign(key, node); });
}

// Recursively merge `overlay` onto `base` in place.
// - Tables whose `type` field differs are replaced wholesale (a different implementation has been
//   selected, so the base section's fields are meaningless). This lets a variant swap e.g. a
//   YoloKeypointModel for a NoopKeypointModel without inheriting stale fields.
// - Otherwise tables are merged key-by-key; scalars and arrays from the overlay replace the base.
void merge_into(toml::table &base, const toml::table &overlay) {
    for (auto &&[key, value] : overlay) {
        const toml::table *overlay_sub = value.as_table();
        toml::node *base_node = base.get(key.str());
        toml::table *base_sub = base_node ? base_node->as_table() : nullptr;

        if (overlay_sub && base_sub) {
            auto base_type = (*base_sub)["type"].value<std::string>();
            auto overlay_type = (*overlay_sub)["type"].value<std::string>();
            if (base_type && overlay_type && *base_type != *overlay_type) {
                assign_copy(base, key.str(), value);  // type changed -> replace section
            } else {
                merge_into(*base_sub, *overlay_sub);
            }
        } else {
            assign_copy(base, key.str(), value);
        }
    }
}

// Resolve an `extends` target to a concrete path via a single deterministic join against the
// config root directory (never a recursive/name-based search), appending `.toml` if needed.
// Extends values are always written relative to the config root, so a base like `_desktop` or
// `playback/_playback` resolves the same way regardless of which subdirectory declares it.
std::filesystem::path resolve_extends(const std::filesystem::path &config_root,
                                      const std::string &value) {
    std::filesystem::path resolved = config_root / value;
    if (resolved.extension() != ".toml") {
        resolved += ".toml";
    }
    return resolved;
}

// Parse `path` and, if it declares `extends`, recursively load+merge its base config underneath it.
// `config_root` is the directory that `extends` values are resolved against.
// `chain` holds the canonical paths currently being resolved, for cycle detection.
toml::table load_and_merge_config(const std::filesystem::path &path,
                                  const std::filesystem::path &config_root,
                                  std::vector<std::filesystem::path> &chain) {
    std::error_code ec;
    std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
    if (ec) {
        canonical = path;
    }

    for (const auto &visited : chain) {
        if (visited == canonical) {
            std::string msg = "Circular config inheritance detected: ";
            for (const auto &c : chain) {
                msg += c.filename().string() + " -> ";
            }
            msg += canonical.filename().string();
            throw ConfigValidationError(msg);
        }
    }
    chain.push_back(canonical);

    toml::table data = toml::parse_file(path.string());

    if (auto extends = data["extends"].value<std::string>()) {
        std::filesystem::path base_path = resolve_extends(config_root, *extends);
        if (!std::filesystem::exists(base_path)) {
            throw ConfigValidationError("Config '" + path.filename().string() + "' extends base '" +
                                        *extends + "' which was not found at " +
                                        base_path.string());
        }
        toml::table merged = load_and_merge_config(base_path, config_root, chain);
        merge_into(merged, data);
        merged.erase("extends");
        chain.pop_back();
        return merged;
    }

    chain.pop_back();
    return data;
}
}  // namespace

template <typename ConfigType>
ConfigType parse_config_section(const toml::table &toml_data, const std::string &section_name,
                                std::vector<std::string> &parsed_sections) {
    ConfigType config;

    auto section = toml_data[section_name].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [" + section_name + "]");
    }

    ConfigParser parser(*section, section_name);
    config.type = parser.get_required_string("type");
    parser.validate_no_extra_fields();

    parsed_sections.push_back(section_name);
    return config;
}

std::filesystem::path normalize_config_path(const std::string &config_path) {
    std::filesystem::path path;
    if (config_path.empty()) {
        path = get_config_dir() / "main.toml";
    } else {
        std::filesystem::path input_path(config_path);
        if (input_path.extension() == ".toml") {
            path = input_path;
        } else {
            path = input_path;
            path += ".toml";
        }
    }
    return path;
}

ClassConfiguration load_classes_from_config(const std::filesystem::path &path,
                                            const std::filesystem::path &config_root) {
    ClassConfiguration config;

    try {
        std::filesystem::path root = config_root.empty() ? get_config_dir() : config_root;
        std::vector<std::filesystem::path> chain;
        auto toml_data = load_and_merge_config(path, root, chain);

        std::vector<std::string> parsed_sections;

        config.camera = load_camera_from_toml(toml_data, parsed_sections);
        config.field_model = load_field_model_from_toml(toml_data, parsed_sections);
        config.robot_mask_model = load_robot_blob_model_from_toml(toml_data, parsed_sections);
        config.field_filter = load_field_filter_from_toml(toml_data, parsed_sections);
        config.keypoint_model = load_keypoint_model_from_toml(toml_data, parsed_sections);
        config.robot_filter = load_robot_filter_from_toml(toml_data, parsed_sections);
        config.target_selector = load_target_selector_from_toml(toml_data, parsed_sections);
        config.navigation = load_navigation_from_toml(toml_data, parsed_sections);
        config.transmitter = load_transmitter_from_toml(toml_data, parsed_sections);
        config.publisher = load_publisher_from_toml(toml_data, parsed_sections);
        config.ui = load_ui_from_toml(toml_data, parsed_sections);
        load_runner_from_toml(toml_data, parsed_sections, config.runner);
        config.health = load_health_from_toml(toml_data, parsed_sections);
        config.mcap_recorder = load_mcap_config_from_toml(toml_data, parsed_sections);
        config.clock = load_clock_from_toml(toml_data, parsed_sections);

        validate_no_extra_sections(toml_data, parsed_sections, path.stem());
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing classes config file: {}", path.string());
        spdlog::error("{}", e.description());
        throw;
    } catch (const ConfigValidationError &e) {
        spdlog::error("Configuration validation error in {}: {}", path.string(), e.what());
        throw;
    } catch (const std::exception &e) {
        spdlog::error("Error reading classes config file: {}", e.what());
        throw;
    }

    return config;
}
}  // namespace auto_battlebot
