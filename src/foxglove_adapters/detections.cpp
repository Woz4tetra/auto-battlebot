#include "foxglove_adapters/detections.hpp"

#include <cstdio>

#include "foxglove_adapters/common.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

std::string to_detections_json(const DetectionsStamped &detections) {
    std::string json;
    json.reserve(64 + detections.detections.size() * 128);

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "{\"stamp\":%.9f,\"w\":%d,\"h\":%d,\"dets\":[",
                  detections.header.stamp, detections.image_width, detections.image_height);
    json += buffer;

    bool first = true;
    for (const auto &det : detections.detections) {
        if (!first) json += ',';
        first = false;
        std::snprintf(buffer, sizeof(buffer),
                      "{\"x1\":%.1f,\"y1\":%.1f,\"x2\":%.1f,\"y2\":%.1f,\"conf\":%.4f,"
                      "\"class_id\":%d,\"label\":\"",
                      det.x1, det.y1, det.x2, det.y2, det.confidence, det.class_id);
        json += buffer;
        json += enum_to_string_lower(det.label);
        json += '"';
        if (!det.keypoints.empty()) {
            json += ",\"kps\":[";
            bool first_kp = true;
            for (const auto &kp : det.keypoints) {
                if (!first_kp) json += ',';
                first_kp = false;
                std::snprintf(buffer, sizeof(buffer), "[%.1f,%.1f,%.4f]", kp.x, kp.y,
                              kp.confidence);
                json += buffer;
            }
            json += ']';
        }
        json += '}';
    }
    json += "]}";
    return json;
}

foxglove::schemas::ImageAnnotations to_image_annotations(const DetectionsStamped &detections) {
    using namespace foxglove::schemas;
    ImageAnnotations annotations;
    const Timestamp stamp = to_timestamp(detections.header.stamp);
    annotations.timestamp = stamp;

    auto point2 = [](double x, double y) {
        Point2 p;
        p.x = x;
        p.y = y;
        return p;
    };

    for (const auto &det : detections.detections) {
        const Color color = to_color(get_color_for_index(det.label), 1.0f);

        PointsAnnotation box;
        box.timestamp = stamp;
        box.type = PointsAnnotation::PointsAnnotationType::LINE_LOOP;
        box.points = {point2(det.x1, det.y1), point2(det.x2, det.y1), point2(det.x2, det.y2),
                      point2(det.x1, det.y2)};
        box.outline_color = color;
        box.thickness = 2.0;
        annotations.points.push_back(std::move(box));

        for (const auto &kp : det.keypoints) {
            CircleAnnotation circle;
            circle.timestamp = stamp;
            circle.position = point2(kp.x, kp.y);
            circle.diameter = 8.0;
            circle.thickness = 2.0;
            circle.outline_color = color;
            circle.fill_color = to_color(color.r, color.g, color.b, 0.4f);
            annotations.circles.push_back(std::move(circle));
        }

        TextAnnotation label;
        label.timestamp = stamp;
        label.position = point2(det.x1, det.y1);
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), " %.2f", det.confidence);
        label.text = enum_to_string_lower(det.label) + buffer;
        label.font_size = 14.0;
        label.text_color = to_color(1.0f, 1.0f, 1.0f, 1.0f);
        label.background_color = to_color(color.r, color.g, color.b, 0.8f);
        annotations.texts.push_back(std::move(label));
    }
    return annotations;
}

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
