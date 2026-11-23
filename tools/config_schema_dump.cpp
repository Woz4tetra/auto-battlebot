#include <CLI/CLI.hpp>
#include <algorithm>
#include <exception>
#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "config/config.hpp"
#include "config/config_factory.hpp"
#include "directories.hpp"

namespace {

using auto_battlebot::ConfigDefaultsRecorder;
using auto_battlebot::ConfigFactory;
using auto_battlebot::ConfigParser;
using auto_battlebot::RecorderMode;
using nlohmann::json;

template <typename Base, typename Prepare>
json probe_type(const std::string &section, const std::string &type_name, Prepare prepare) {
    toml::table table{{"type", type_name}};
    ConfigDefaultsRecorder recorder(RecorderMode::PROBE_SCHEMA);
    json entry = {{"fields", json::array()}, {"error", nullptr}};
    try {
        ConfigParser parser(table, section);
        parser.get_required_string("type");
        auto &factory = ConfigFactory<Base>::instance();
        auto config = factory.create(type_name);
        prepare(*config, parser);
        factory.parse(*config, parser);
    } catch (const std::exception &e) {
        entry["error"] = e.what();
    }
    for (const auto &field : recorder.recorded()) {
        entry["fields"].push_back({{"section", field.section_name},
                                   {"key", field.key},
                                   {"default", field.value},
                                   {"required", field.required}});
    }
    return entry;
}

template <typename Base, typename Prepare>
void probe_section(json &out, const std::string &section, Prepare prepare) {
    json types = json::object();
    for (const auto &name : ConfigFactory<Base>::instance().type_names()) {
        types[name] = probe_type<Base>(section, name, prepare);
    }
    out[section] = {{"types", types}};
}

template <typename Base>
void probe_section(json &out, const std::string &section) {
    probe_section<Base>(out, section, [](Base &, ConfigParser &) {});
}

void collect_types(const toml::table &table, const std::string &prefix, json &out) {
    for (const auto &[key, value] : table) {
        const toml::table *sub = value.as_table();
        if (!sub) continue;
        const std::string name = prefix.empty() ? std::string(key.str())
                                                : prefix + "." + std::string(key.str());
        if (auto type = (*sub)["type"].value<std::string>()) out[name] = *type;
        collect_types(*sub, name, out);
    }
}

json profile_types(const std::filesystem::path &config_dir) {
    json profiles = json::object();
    std::vector<std::filesystem::path> files;
    for (const auto &entry : std::filesystem::directory_iterator(config_dir)) {
        const auto &path = entry.path();
        if (path.extension() == ".toml" && !path.stem().string().starts_with("_")) {
            files.push_back(path);
        }
    }
    std::sort(files.begin(), files.end());
    for (const auto &path : files) {
        json types = json::object();
        json fields = json::array();
        json error = nullptr;
        try {
            const toml::table merged = auto_battlebot::load_merged_config(path, config_dir);
            if (!merged.contains("rgbd_camera")) continue;
            collect_types(merged, "", types);
            auto_battlebot::ResolvedConfig resolved;
            auto_battlebot::load_classes_from_config(path, config_dir, &resolved);
            for (const auto &field : resolved.fields) {
                fields.push_back({{"section", field.section},
                                  {"key", field.key},
                                  {"value", field.value},
                                  {"source", field.source}});
            }
        } catch (const std::exception &e) {
            error = e.what();
        }
        profiles[path.stem().string()] = {{"types", types}, {"fields", fields}, {"error", error}};
    }
    return profiles;
}

}  // namespace

int main(int argc, char **argv) {
    using namespace auto_battlebot;
    CLI::App app{"Print every REGISTER_CONFIG type's fields and defaults, and each profile's types"};
    std::string config_dir = get_config_dir().string();
    app.add_option("--config-dir", config_dir, "Directory holding the top-level profiles");
    CLI11_PARSE(app, argc, argv);

    json sections = json::object();
    probe_section<RgbdCameraConfiguration>(sections, "rgbd_camera");
    probe_section<MaskModelConfiguration>(sections, "field_model");
    probe_section<FieldFilterConfiguration>(
        sections, "field_filter",
        [](FieldFilterConfiguration &config, ConfigParser &parser) {
            config.parse_common_fields(parser);
        });
    probe_section<KeypointModelConfiguration>(sections, "keypoint_model");
    probe_section<RobotFilterConfiguration>(sections, "robot_filter");
    probe_section<MotionEstimatorConfiguration>(sections, "robot_filter.motion_estimator");
    probe_section<TargetSelectorConfiguration>(sections, "target_selector");
    probe_section<TransmitterConfiguration>(sections, "transmitter");
    probe_section<NavigationConfiguration>(sections, "navigation");
    probe_section<CommandShaperConfiguration>(sections, "command_shaper");
    probe_section<PublisherConfiguration>(sections, "publisher");
    probe_section<ClockConfiguration>(sections, "clock");

    json out = {{"sections", sections}, {"profiles", profile_types(config_dir)}};
    std::cout << out.dump(2) << "\n";
    return 0;
}
