#include <CLI/CLI.hpp>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "config/config.hpp"
#include "health/mcap_grader.hpp"
#include "health/performance_monitor_config.hpp"

int main(int argc, char** argv) {
    using namespace auto_battlebot;
    CLI::App app{"Grade recordings with the PerformanceMonitor checks"};
    std::vector<std::string> files;
    std::string profile;
    app.add_option("files", files, "MCAP recordings")->required()->check(CLI::ExistingFile);
    app.add_option("-c,--config", profile,
                   "Profile whose [performance_monitor] thresholds to use (default: struct "
                   "defaults)");
    CLI11_PARSE(app, argc, argv);

    PerformanceMonitorConfiguration config;
    if (!profile.empty()) {
        try {
            std::vector<std::string> sections;
            config = load_performance_monitor_from_toml(
                load_merged_config(normalize_config_path(profile)), sections);
        } catch (const std::exception& e) {
            std::cerr << "cannot load " << profile << ": " << e.what() << "\n";
            return 3;
        }
    }

    bool any_red = false;
    for (const std::string& file : files) {
        try {
            const McapGrade grade = grade_mcap(file, config);
            std::cout << format_grade(grade);
            any_red = any_red || grade.red();
        } catch (const std::exception& e) {
            std::cerr << file << ": " << e.what() << "\n";
            return 3;
        }
    }
    return any_red ? 1 : 0;
}
