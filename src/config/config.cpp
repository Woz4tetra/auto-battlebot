#include "config/config.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <filesystem>
#include <string_view>
#include <system_error>
#include <utility>

namespace auto_battlebot {
namespace {
std::filesystem::path resolve_extends(const std::filesystem::path &config_root,
                                      const std::string &value) {
    std::filesystem::path resolved = config_root / value;
    if (resolved.extension() != ".toml") {
        resolved += ".toml";
    }
    return resolved;
}

constexpr std::string_view kFromKey = "from";

using SectionPath = std::vector<std::string>;

std::string section_name(const SectionPath &path) {
    std::string name;
    for (const auto &part : path) {
        if (!name.empty()) {
            name += ".";
        }
        name += part;
    }
    return name;
}

void move_into(toml::table &dst, std::string_view key, toml::node &src) {
    src.visit([&](auto &&node) { dst.insert_or_assign(key, std::move(node)); });
}

bool selects_other_type(const toml::table &base, const toml::table &overlay) {
    auto base_type = base["type"].value<std::string>();
    auto overlay_type = overlay["type"].value<std::string>();
    return base_type && overlay_type && *base_type != *overlay_type;
}

void merge_into(toml::table &base, toml::table &overlay, SectionPath &path,
                const std::vector<SectionPath> &replaced) {
    for (auto &&[key, value] : overlay) {
        path.emplace_back(key.str());
        toml::table *overlay_sub = value.as_table();
        toml::node *base_node = base.get(key.str());
        toml::table *base_sub = base_node ? base_node->as_table() : nullptr;
        const bool replace = std::find(replaced.begin(), replaced.end(), path) != replaced.end();
        if (overlay_sub && base_sub && !replace && !selects_other_type(*base_sub, *overlay_sub)) {
            merge_into(*base_sub, *overlay_sub, path, replaced);
        } else {
            move_into(base, key.str(), value);
        }
        path.pop_back();
    }
}

template <typename Table>
Table *find_table(Table &root, const SectionPath &path) {
    Table *table = &root;
    for (const auto &part : path) {
        auto *node = table->get(part);
        if (!node) {
            return nullptr;
        }
        table = node->as_table();
        if (!table) {
            return nullptr;
        }
    }
    return table;
}

bool contains_from_key(const toml::table &table) {
    for (auto &&[key, value] : table) {
        if (key.str() == kFromKey) {
            return true;
        }
        const toml::table *sub = value.as_table();
        if (sub && contains_from_key(*sub)) {
            return true;
        }
    }
    return false;
}

toml::table load_section_file(const std::filesystem::path &referrer, const SectionPath &path,
                              const std::string &value, const std::filesystem::path &config_root) {
    const std::string where =
        "Config '" + referrer.string() + "' [" + section_name(path) + "] from = \"" + value + "\"";
    std::filesystem::path file = resolve_extends(config_root, value);
    if (!std::filesystem::exists(file)) {
        throw ConfigValidationError(where + ": no file at " + file.string());
    }
    toml::table data = toml::parse_file(file.string());
    if (data.contains("extends")) {
        throw ConfigValidationError(where + ": " + file.string() +
                                    " sets `extends`, which a section file may not");
    }
    if (contains_from_key(data)) {
        throw ConfigValidationError(where + ": " + file.string() +
                                    " uses `from`, which a section file may not");
    }
    toml::table *section = find_table(data, path);
    if (!section) {
        throw ConfigValidationError(where + ": " + file.string() + " has no [" +
                                    section_name(path) + "]");
    }
    return std::move(*section);
}

void expand_section_files(toml::table &table, SectionPath &path,
                          const std::filesystem::path &referrer,
                          const std::filesystem::path &config_root,
                          std::vector<SectionPath> &expanded) {
    for (auto &&[key, value] : table) {
        toml::table *sub = value.as_table();
        if (!sub) {
            continue;
        }
        path.emplace_back(key.str());
        if (const toml::node *from = sub->get(kFromKey)) {
            const std::string where =
                "Config '" + referrer.string() + "' [" + section_name(path) + "]";
            auto file = from->value<std::string>();
            if (!file) {
                throw ConfigValidationError(where + ": `from` must be a string");
            }
            if (sub->size() != 1) {
                throw ConfigValidationError(where + ": `from` must be the only key in the table");
            }
            *sub = load_section_file(referrer, path, *file, config_root);
            expanded.push_back(path);
        } else {
            expand_section_files(*sub, path, referrer, config_root, expanded);
        }
        path.pop_back();
    }
}

std::vector<SectionPath> expand_section_files(toml::table &data,
                                              const std::filesystem::path &referrer,
                                              const std::filesystem::path &config_root) {
    if (data.contains(kFromKey)) {
        throw ConfigValidationError("Config '" + referrer.string() +
                                    "': `from` is only valid inside a table");
    }
    std::vector<SectionPath> expanded;
    SectionPath path;
    expand_section_files(data, path, referrer, config_root, expanded);
    return expanded;
}

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
    const std::vector<SectionPath> expanded = expand_section_files(data, path, config_root);

    if (auto extends = data["extends"].value<std::string>()) {
        std::filesystem::path base_path = resolve_extends(config_root, *extends);
        if (!std::filesystem::exists(base_path)) {
            throw ConfigValidationError("Config '" + path.filename().string() + "' extends base '" +
                                        *extends + "' which was not found at " +
                                        base_path.string());
        }
        toml::table merged = load_and_merge_config(base_path, config_root, chain);
        data.erase("extends");
        SectionPath section_path;
        merge_into(merged, data, section_path, expanded);
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

toml::table load_merged_config(const std::filesystem::path &path,
                               const std::filesystem::path &config_root) {
    std::filesystem::path root = config_root.empty() ? get_config_dir() : config_root;
    std::vector<std::filesystem::path> chain;
    return load_and_merge_config(path, root, chain);
}

ClassConfiguration load_classes_from_config(const std::filesystem::path &path,
                                            const std::filesystem::path &config_root,
                                            ResolvedConfig *resolved) {
    ClassConfiguration config;

    try {
        const std::filesystem::path root = config_root.empty() ? get_config_dir() : config_root;
        auto toml_data = load_merged_config(path, root);
        ConfigDefaultsRecorder recorder;

        std::vector<std::string> parsed_sections;

        config.plant = load_plant_from_toml(toml_data, parsed_sections);
        config.camera = load_camera_from_toml(toml_data, parsed_sections);
        config.field_model = load_field_model_from_toml(toml_data, parsed_sections);
        config.field_filter = load_field_filter_from_toml(toml_data, parsed_sections);
        config.keypoint_model = load_keypoint_model_from_toml(toml_data, parsed_sections);
        config.keypoint_filter = load_keypoint_filter_from_toml(toml_data, parsed_sections);
        config.robot_filter = load_robot_filter_from_toml(toml_data, parsed_sections, config.plant);
        config.target_selector = load_target_selector_from_toml(toml_data, parsed_sections);
        config.transmitter = load_transmitter_from_toml(toml_data, parsed_sections);
        config.navigation = load_navigation_from_toml(toml_data, parsed_sections, config.plant,
                                                      *config.robot_filter, *config.transmitter);
        config.command_shaper =
            load_command_shaper_from_toml(toml_data, parsed_sections, config.plant);
        config.control_loop = load_control_loop_from_toml(toml_data, parsed_sections);
        config.publisher = load_publisher_from_toml(toml_data, parsed_sections);
        config.ui = load_ui_from_toml(toml_data, parsed_sections);
        load_runner_from_toml(toml_data, parsed_sections, config.runner);
        config.health = load_health_from_toml(toml_data, parsed_sections);
        config.performance_monitor = load_performance_monitor_from_toml(toml_data, parsed_sections);
        config.mcap_recorder = load_mcap_config_from_toml(toml_data, parsed_sections);
        config.clock = load_clock_from_toml(toml_data, parsed_sections);

        validate_no_extra_sections(toml_data, parsed_sections, path.stem());
        if (resolved) {
            *resolved = recorder.resolve(toml_data, root);
        }
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
