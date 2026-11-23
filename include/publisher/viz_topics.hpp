#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <string_view>

namespace auto_battlebot {

enum class TopicRate { PER_FRAME, EVENT, CONFIGURED };

enum class VizTopicId {
    CAMERA_IMAGE,
    CAMERA_CAMERA_INFO,
    CAMERA_FRAME_META,
    CAMERA_VIDEO,
    CAMERA_PREVIEW,
    CAMERA_PREVIEW_VIDEO,
    TF,
    FIELD_MASK,
    FIELD_MASK_CAMERA_INFO,
    FIELD_MARKERS,
    FIELD_POINTS,
    HAZARD_MARKERS,
    ROBOT_MARKERS,
    NAV_MARKERS,
    KEYPOINT_DETECTIONS,
    KEYPOINT_ANNOTATIONS,
    LOG,
    ESPNOW_TELEMETRY,
    APRILTAG_ROBOT_TAGS,
};

struct VizTopic {
    VizTopicId id;
    std::string_view topic;
    std::string_view encoding;
    bool latch;
    TopicRate rate;
    bool recorded;
};

inline constexpr std::string_view kDiagnosticsTopicPrefix = "/diagnostics/";

inline constexpr auto kVizTopics = std::array{
    VizTopic{VizTopicId::CAMERA_IMAGE, "/camera/image", "protobuf", false, TopicRate::PER_FRAME,
             true},
    VizTopic{VizTopicId::CAMERA_CAMERA_INFO, "/camera/camera_info", "protobuf", false,
             TopicRate::PER_FRAME, true},
    VizTopic{VizTopicId::CAMERA_FRAME_META, "/camera/frame_meta", "json", false,
             TopicRate::PER_FRAME, true},
    VizTopic{VizTopicId::CAMERA_VIDEO, "/camera/video", "protobuf", false, TopicRate::PER_FRAME,
             true},
    VizTopic{VizTopicId::CAMERA_PREVIEW, "/camera/preview", "protobuf", false,
             TopicRate::CONFIGURED, false},
    VizTopic{VizTopicId::CAMERA_PREVIEW_VIDEO, "/camera/preview_video", "protobuf", false,
             TopicRate::CONFIGURED, false},
    VizTopic{VizTopicId::TF, "/tf", "protobuf", false, TopicRate::PER_FRAME, true},
    VizTopic{VizTopicId::FIELD_MASK, "/field_mask", "protobuf", true, TopicRate::EVENT, true},
    VizTopic{VizTopicId::FIELD_MASK_CAMERA_INFO, "/field_mask/camera_info", "protobuf", true,
             TopicRate::EVENT, true},
    VizTopic{VizTopicId::FIELD_MARKERS, "/field_markers", "protobuf", true, TopicRate::EVENT, true},
    VizTopic{VizTopicId::FIELD_POINTS, "/field_points", "protobuf", true, TopicRate::EVENT, true},
    VizTopic{VizTopicId::HAZARD_MARKERS, "/hazard_markers", "protobuf", true, TopicRate::PER_FRAME,
             true},
    VizTopic{VizTopicId::ROBOT_MARKERS, "/robot_markers", "protobuf", true, TopicRate::PER_FRAME,
             true},
    VizTopic{VizTopicId::NAV_MARKERS, "/nav_markers", "protobuf", true, TopicRate::PER_FRAME, true},
    VizTopic{VizTopicId::KEYPOINT_DETECTIONS, "/keypoint_detections", "json", false,
             TopicRate::PER_FRAME, true},
    VizTopic{VizTopicId::KEYPOINT_ANNOTATIONS, "/keypoint_detections/annotations", "protobuf",
             false, TopicRate::PER_FRAME, false},
    VizTopic{VizTopicId::LOG, "/log", "protobuf", false, TopicRate::EVENT, true},
    VizTopic{VizTopicId::ESPNOW_TELEMETRY, "/robot/espnow_telemetry", "json", false,
             TopicRate::EVENT, true},
    VizTopic{VizTopicId::APRILTAG_ROBOT_TAGS, "/apriltag/robot_tags", "json", false,
             TopicRate::PER_FRAME, true},
};

namespace detail {

constexpr bool viz_topics_ordered() {
    for (std::size_t i = 0; i < kVizTopics.size(); ++i) {
        if (static_cast<std::size_t>(kVizTopics[i].id) != i) return false;
    }
    return true;
}

constexpr bool viz_topics_unique() {
    for (std::size_t i = 0; i < kVizTopics.size(); ++i) {
        for (std::size_t j = i + 1; j < kVizTopics.size(); ++j) {
            if (kVizTopics[i].topic == kVizTopics[j].topic) return false;
        }
        if (kVizTopics[i].topic.starts_with(kDiagnosticsTopicPrefix)) return false;
        if (kVizTopics[i].topic.starts_with("/status/")) return false;
        if (kVizTopics[i].topic.starts_with("/command/")) return false;
    }
    return true;
}

}  // namespace detail

static_assert(detail::viz_topics_ordered(), "kVizTopics rows follow VizTopicId order");
static_assert(detail::viz_topics_unique(),
              "viz topics are unique and stay out of the /diagnostics/, /status/ and /command/ "
              "namespaces");

constexpr const VizTopic &viz_topic(VizTopicId id) {
    return kVizTopics[static_cast<std::size_t>(id)];
}

inline std::string diagnostics_topic(std::string_view module) {
    return std::string(kDiagnosticsTopicPrefix) + std::string(module);
}

}  // namespace auto_battlebot
