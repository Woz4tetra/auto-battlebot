#include "robot_filter/filter_input_record.hpp"

#include <magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <ostream>

#include "enums.hpp"

namespace auto_battlebot {
namespace {
using Json = nlohmann::json;

// cv::Mat is serialized as {rows, cols, data: [row-major doubles]} rather than relying on any
// particular element type, since CameraInfo's intrinsics (3x3) and distortion (1x5) are the only
// matrices this format needs to round-trip and both are populated as CV_64F.
Json mat_to_json(const cv::Mat &mat) {
    std::vector<double> data;
    data.reserve(static_cast<size_t>(mat.rows) * static_cast<size_t>(mat.cols));
    for (int r = 0; r < mat.rows; ++r) {
        for (int c = 0; c < mat.cols; ++c) {
            data.push_back(mat.at<double>(r, c));
        }
    }
    return Json{{"rows", mat.rows}, {"cols", mat.cols}, {"data", data}};
}

cv::Mat mat_from_json(const Json &j) {
    int rows = j.at("rows").get<int>();
    int cols = j.at("cols").get<int>();
    auto data = j.at("data").get<std::vector<double>>();
    cv::Mat mat(rows, cols, CV_64F);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            mat.at<double>(r, c) = data[static_cast<size_t>(r * cols + c)];
        }
    }
    return mat;
}

Json header_to_json(const Header &h) {
    return Json{{"stamp", h.stamp}, {"frame_id", std::string(magic_enum::enum_name(h.frame_id))}};
}

Header header_from_json(const Json &j) {
    Header h;
    h.stamp = j.at("stamp").get<double>();
    if (auto v = magic_enum::enum_cast<FrameId>(j.at("frame_id").get<std::string>())) {
        h.frame_id = *v;
    }
    return h;
}

Json keypoint_to_json(const Keypoint &k) {
    return Json{
        {"label", std::string(magic_enum::enum_name(k.label))},
        {"keypoint_label", std::string(magic_enum::enum_name(k.keypoint_label))},
        {"x", k.x},
        {"y", k.y},
        {"confidence", k.confidence},
        {"detection_index", k.detection_index},
    };
}

Keypoint keypoint_from_json(const Json &j) {
    Keypoint k;
    if (auto v = magic_enum::enum_cast<Label>(j.at("label").get<std::string>())) {
        k.label = *v;
    }
    if (auto v = magic_enum::enum_cast<KeypointLabel>(j.at("keypoint_label").get<std::string>())) {
        k.keypoint_label = *v;
    }
    k.x = j.at("x").get<double>();
    k.y = j.at("y").get<double>();
    k.confidence = j.at("confidence").get<double>();
    k.detection_index = j.at("detection_index").get<int>();
    return k;
}

Json keypoints_stamped_to_json(const KeypointsStamped &ks) {
    Json points = Json::array();
    for (const auto &k : ks.keypoints) {
        points.push_back(keypoint_to_json(k));
    }
    return Json{{"header", header_to_json(ks.header)}, {"points", points}};
}

KeypointsStamped keypoints_stamped_from_json(const Json &j) {
    KeypointsStamped ks;
    ks.header = header_from_json(j.at("header"));
    for (const auto &pj : j.at("points")) {
        ks.keypoints.push_back(keypoint_from_json(pj));
    }
    return ks;
}

Json camera_info_to_json(const CameraInfo &ci) {
    return Json{
        {"header", header_to_json(ci.header)},
        {"width", ci.width},
        {"height", ci.height},
        {"intrinsics", mat_to_json(ci.intrinsics)},
        {"distortion", mat_to_json(ci.distortion)},
    };
}

CameraInfo camera_info_from_json(const Json &j) {
    CameraInfo ci;
    ci.header = header_from_json(j.at("header"));
    ci.width = j.at("width").get<int>();
    ci.height = j.at("height").get<int>();
    ci.intrinsics = mat_from_json(j.at("intrinsics"));
    ci.distortion = mat_from_json(j.at("distortion"));
    return ci;
}

// Flat row-major 16-element array; Transform is just a 4x4 homogeneous matrix (Eigen::Matrix4d).
Json transform_to_json(const Transform &t) {
    std::vector<double> flat;
    flat.reserve(16);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            flat.push_back(t.tf(r, c));
        }
    }
    return flat;
}

Transform transform_from_json(const Json &j) {
    Transform t;
    auto flat = j.get<std::vector<double>>();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            t.tf(r, c) = flat[static_cast<size_t>(r * 4 + c)];
        }
    }
    return t;
}

Json field_description_to_json(const FieldDescription &f) {
    return Json{
        {"header", header_to_json(f.header)},
        {"child_frame_id", std::string(magic_enum::enum_name(f.child_frame_id))},
        {"tf_camera_from_fieldcenter", transform_to_json(f.tf_camera_from_fieldcenter)},
        {"size",
         Json{
             {"header", header_to_json(f.size.header)},
             {"x", f.size.size.x},
             {"y", f.size.size.y},
             {"z", f.size.size.z},
         }},
    };
}

FieldDescription field_description_from_json(const Json &j) {
    FieldDescription f;
    f.header = header_from_json(j.at("header"));
    if (auto v = magic_enum::enum_cast<FrameId>(j.at("child_frame_id").get<std::string>())) {
        f.child_frame_id = *v;
    }
    f.tf_camera_from_fieldcenter = transform_from_json(j.at("tf_camera_from_fieldcenter"));
    const Json &size_j = j.at("size");
    f.size.header = header_from_json(size_j.at("header"));
    f.size.size.x = size_j.at("x").get<double>();
    f.size.size.y = size_j.at("y").get<double>();
    f.size.size.z = size_j.at("z").get<double>();
    return f;
}
}  // namespace

void write_filter_input_record(std::ostream &out, const FilterInputRecord &record) {
    Json j{
        {"tick", record.tick},
        {"svo_frame_index", record.svo_frame_index},
        {"keypoints", keypoints_stamped_to_json(record.keypoints)},
        {"robot_blob_keypoints", keypoints_stamped_to_json(record.robot_blob_keypoints)},
        {"camera_info", camera_info_to_json(record.camera_info)},
        {"field_description", field_description_to_json(record.field_description)},
    };
    out << j.dump() << "\n";
}

std::optional<FilterInputRecord> parse_filter_input_record(const std::string &line) {
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
        return std::nullopt;
    }
    Json j = Json::parse(line);
    FilterInputRecord record;
    record.tick = j.at("tick").get<int>();
    record.svo_frame_index = j.at("svo_frame_index").get<int64_t>();
    record.keypoints = keypoints_stamped_from_json(j.at("keypoints"));
    record.robot_blob_keypoints = keypoints_stamped_from_json(j.at("robot_blob_keypoints"));
    record.camera_info = camera_info_from_json(j.at("camera_info"));
    record.field_description = field_description_from_json(j.at("field_description"));
    return record;
}

}  // namespace auto_battlebot
