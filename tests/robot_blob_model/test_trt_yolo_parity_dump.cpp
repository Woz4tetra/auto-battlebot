#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <opencv2/imgcodecs.hpp>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "engine_selector/engine_selector.hpp"
#include "robot_blob_model/yolo_bbox_robot_blob_model.hpp"

namespace auto_battlebot {
namespace {

std::string env_or_empty(const char *name) {
    const char *value = std::getenv(name);
    return value ? value : "";
}

TEST(TrtYoloParityDumpTest, WritesTheDetectionsForTheEngineAndImageNamedInTheEnvironment) {
    const std::string engine = env_or_empty("ABT_PARITY_ENGINE");
    const std::string image_path = env_or_empty("ABT_PARITY_IMAGE");
    const std::string out_path = env_or_empty("ABT_PARITY_OUT");
    if (engine.empty() || image_path.empty() || out_path.empty()) {
        GTEST_SKIP() << "set ABT_PARITY_ENGINE, ABT_PARITY_IMAGE and ABT_PARITY_OUT";
    }
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});

    YoloBboxRobotBlobModelConfiguration config;
    config.engine.candidates = {engine};
    config.confidence_threshold = 0.5f;
    config.their_robot_labels = {Label::OPPONENT};
    config.label_indices = {Label::OPPONENT, Label::OPPONENT};
    auto selector = std::make_shared<EngineSelector>(config.engine, "parity");
    YoloBboxRobotBlobModel model(config, selector);
    ASSERT_TRUE(model.initialize());

    RgbImage image;
    image.image = cv::imread(image_path, cv::IMREAD_COLOR);
    ASSERT_FALSE(image.image.empty());
    model.update(image);

    std::ofstream out(out_path);
    for (const auto &d : model.last_detections_original_px().detections) {
        out << d.x1 << " " << d.y1 << " " << d.x2 << " " << d.y2 << " " << d.confidence << " "
            << d.class_id << "\n";
    }
}

}  // namespace
}  // namespace auto_battlebot
