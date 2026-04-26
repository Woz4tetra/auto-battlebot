// Probe 1: ZED grab / retrieveImage / retrieveMeasure hang detection.
//
// Loops sl::Camera::grab + retrieveImage + retrieveMeasure with a per-stage watchdog
// so a hang attributes to the specific call. Mirrors InitParameters from
// src/rgbd_camera/zed_rgbd_camera.cpp.

#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <sl/Camera.hpp>
#include <string>
#include <vector>

#include "diag_common.hpp"
#include "spdlog/spdlog.h"

namespace ab = auto_battlebot::diag;

namespace {

sl::DEPTH_MODE parse_depth_mode(const std::string& s) {
    if (s == "neural") return sl::DEPTH_MODE::NEURAL;
    if (s == "performance") return sl::DEPTH_MODE::PERFORMANCE;
    if (s == "none") return sl::DEPTH_MODE::NONE;
    return sl::DEPTH_MODE::NEURAL;
}

struct Histogram {
    std::vector<double> samples;
    void add(double v) { samples.push_back(v); }
    void report(const char* tag) {
        if (samples.empty()) return;
        std::vector<double> sorted = samples;
        std::sort(sorted.begin(), sorted.end());
        const double p50 = sorted[sorted.size() / 2];
        const double p99 = sorted[static_cast<size_t>(sorted.size() * 0.99)];
        const double max = sorted.back();
        spdlog::info("{} count={} p50_ms={:.2f} p99_ms={:.2f} max_ms={:.2f}", tag, sorted.size(),
                     p50, p99, max);
        samples.clear();
    }
};

}  // namespace

int main(int argc, char** argv) {
    ab::install_sigsegv_backtrace();
    ab::configure_logger("diag_zed_grab");

    int iterations = 0;
    double duration_s = 60.0;
    int watchdog_s = 8;
    std::string svo_path;
    std::string depth_mode_str = "neural";
    std::string retrieve_str = "both";
    int fps = 30;

    CLI::App app{"ZED grab/retrieve hang probe"};
    app.add_option("--iterations", iterations, "Stop after N iterations (0 = use --duration-s)");
    app.add_option("--duration-s", duration_s, "Stop after this many seconds");
    app.add_option("--watchdog-s", watchdog_s, "Per-stage hang threshold (seconds)");
    app.add_option("--svo", svo_path, "Replay an SVO file instead of live camera");
    app.add_option("--depth-mode", depth_mode_str, "neural|performance|none");
    app.add_option("--retrieve", retrieve_str, "image|measure|both|none");
    app.add_option("--fps", fps, "Camera FPS");
    CLI11_PARSE(app, argc, argv);

    const bool retrieve_image = retrieve_str == "image" || retrieve_str == "both";
    const bool retrieve_measure = retrieve_str == "measure" || retrieve_str == "both";

    sl::Camera zed;
    sl::InitParameters params;
    params.camera_fps = fps;
    params.camera_resolution = sl::RESOLUTION::HD720;
    params.depth_mode = parse_depth_mode(depth_mode_str);
    params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    params.coordinate_units = sl::UNIT::METER;
    if (!svo_path.empty()) {
        params.input.setFromSVOFile(svo_path.c_str());
        params.svo_real_time_mode = true;
    }

    {
        ab::StageWatchdog wd(std::chrono::seconds(30), "diag_zed_grab");
        DIAG_STAGE(wd, "zed_open");
        const auto err = zed.open(params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            spdlog::error("zed.open failed: {}", sl::toString(err).c_str());
            return ab::kExitSetupFailure;
        }
    }

    ab::StageWatchdog wd(std::chrono::seconds(watchdog_s), "diag_zed_grab");

    sl::Mat rgb_mat;
    sl::Mat depth_mat;
    sl::RuntimeParameters rt;

    Histogram grab_hist;
    Histogram image_hist;
    Histogram measure_hist;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0));
    int errors = 0;
    int total = 0;
    char stage_buf[64];

    for (int i = 0; iterations == 0 || i < iterations; ++i) {
        if (iterations == 0 && std::chrono::steady_clock::now() > deadline) break;

        std::snprintf(stage_buf, sizeof(stage_buf), "grab_%d", i);
        wd.stage(stage_buf);
        const auto t0 = std::chrono::steady_clock::now();
        const auto err = zed.grab(rt);
        const double grab_ms = ab::elapsed_ms_since(t0);
        grab_hist.add(grab_ms);
        if (err != sl::ERROR_CODE::SUCCESS) {
            ++errors;
            spdlog::warn("grab error iter={} code={}", i, sl::toString(err).c_str());
            if (svo_path.empty()) continue;
            // SVO end of file
            break;
        }

        if (retrieve_image) {
            std::snprintf(stage_buf, sizeof(stage_buf), "retrieveImage_left_%d", i);
            wd.stage(stage_buf);
            const auto t1 = std::chrono::steady_clock::now();
            zed.retrieveImage(rgb_mat, sl::VIEW::LEFT);
            image_hist.add(ab::elapsed_ms_since(t1));
        }

        if (retrieve_measure && params.depth_mode != sl::DEPTH_MODE::NONE) {
            std::snprintf(stage_buf, sizeof(stage_buf), "retrieveMeasure_depth_%d", i);
            wd.stage(stage_buf);
            const auto t2 = std::chrono::steady_clock::now();
            zed.retrieveMeasure(depth_mat, sl::MEASURE::DEPTH);
            measure_hist.add(ab::elapsed_ms_since(t2));
        }

        ++total;
        if ((i + 1) % 100 == 0) {
            grab_hist.report("grab");
            image_hist.report("retrieveImage");
            measure_hist.report("retrieveMeasure");
        }
    }

    wd.stage("zed_close");
    zed.close();

    grab_hist.report("grab_final");
    image_hist.report("retrieveImage_final");
    measure_hist.report("retrieveMeasure_final");

    if (total == 0) {
        spdlog::error("no successful iterations");
        return ab::kExitSetupFailure;
    }
    const double err_ratio = static_cast<double>(errors) / static_cast<double>(total + errors);
    spdlog::info("DONE total={} errors={} err_ratio={:.3f}", total, errors, err_ratio);
    if (err_ratio > 0.05) return ab::kExitErrors;
    return ab::kExitPass;
}
