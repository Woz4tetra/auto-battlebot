#include "keypoint_model/yolo_keypoint_model.hpp"

namespace auto_battlebot
{
    using torch::indexing::None;
    using torch::indexing::Slice;

    YoloKeypointModel::YoloKeypointModel(YoloKeypointModelConfiguration &config)
        : model_path_(config.model_path),
          threshold_(config.threshold),
          iou_threshold_(config.iou_threshold),
          letterbox_padding_(config.letterbox_padding),
          image_size_(config.image_size),
          debug_visualization_(config.debug_visualization),
          label_map_(config.label_map),
          device_(torch::kCPU),
          initialized_(false)
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("yolo_keypoint_model");
        // Check if CUDA is available
        if (torch::cuda::is_available())
        {
            std::cout << "CUDA is available. Using GPU." << std::endl;
            device_ = torch::Device(torch::kCUDA);
        }
        else
        {
            std::cout << "CUDA not available. Using CPU." << std::endl;
        }
    }

    bool YoloKeypointModel::initialize()
    {
        try
        {
            // Load the TorchScript model
            std::cout << "Loading model from: " << model_path_ << std::endl;
            model_ = torch::jit::load(model_path_);
            model_.to(device_);
            model_.eval();

            // Warmup inference
            std::cout << "Warming up model with dummy input..." << std::endl;
            auto warmup_input = torch::randn({1, 3, image_size_, image_size_}).to(device_);
            for (int i = 0; i < 3; i++)
            {
                model_.forward({warmup_input});
            }

            initialized_ = true;
            diagnostics_logger_->info({}, "YoloKeypointModel initialized successfully");
            return true;
        }
        catch (const c10::Error &e)
        {
            std::cerr << "Error loading model: " << e.what() << std::endl;
            return false;
        }
    }

    KeypointsStamped YoloKeypointModel::update(RgbImage image)
    {
        FunctionTimer timer(diagnostics_logger_, "update", 1000.0); // Warn if > 1000ms

        if (!initialized_)
        {
            diagnostics_logger_->error({}, "Model not initialized");
            return KeypointsStamped{};
        }
        cv::Size input_image_size = cv::Size(image_size_, image_size_);
        cv::Size original_image_size = cv::Size(image.image.cols, image.image.rows);

        // Preprocess the image
        torch::Tensor input_tensor = preprocess_image(image.image, input_image_size);

        // Run inference
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input_tensor);

        torch::Tensor output = model_.forward(inputs).toTensor();

        // Postprocess the output
        KeypointsStamped result = postprocess_output(output, image.header, original_image_size, input_image_size, image.image);

        return result;
    }

    torch::Tensor YoloKeypointModel::preprocess_image(const cv::Mat &image, cv::Size input_image_size)
    {
        // Note: ultralytics YOLO models expect RGB input, but the TorchScript export
        // may handle the conversion internally. Try both BGR and RGB if results are poor.
        cv::Mat processed_image;
        if (image.channels() == 3)
        {
            cv::cvtColor(image, processed_image, cv::COLOR_BGR2RGB);
        }
        else if (image.channels() == 4)
        {
            cv::cvtColor(image, processed_image, cv::COLOR_BGRA2RGB);
        }
        else
        {
            processed_image = image;
        }

        // Resize to model input size with letterboxing
        cv::Mat resized;
        letterbox(processed_image, resized, {input_image_size.height, input_image_size.width});

        // Convert to float and normalize to [0, 1]
        cv::Mat float_image;
        resized.convertTo(float_image, CV_32F, 1.0 / 255.0);

        // Ensure contiguous memory
        if (!float_image.isContinuous())
        {
            float_image = float_image.clone();
        }

        // Convert to tensor: HWC -> CHW
        torch::Tensor tensor = torch::from_blob(
                                   float_image.data,
                                   {float_image.rows, float_image.cols, 3},
                                   torch::kFloat32)
                                   .clone();

        tensor = tensor.permute({2, 0, 1}).contiguous(); // HWC -> CHW
        tensor = tensor.unsqueeze(0);                    // Add batch dimension

        return tensor.to(device_);
    }

    float YoloKeypointModel::generate_scale(cv::Mat &image, const std::vector<int> &target_size)
    {
        int origin_w = image.cols;
        int origin_h = image.rows;

        int target_h = target_size[0];
        int target_w = target_size[1];

        float ratio_h = static_cast<float>(target_h) / static_cast<float>(origin_h);
        float ratio_w = static_cast<float>(target_w) / static_cast<float>(origin_w);
        float resize_scale = std::min(ratio_h, ratio_w);
        return resize_scale;
    }

    float YoloKeypointModel::letterbox(cv::Mat &input_image, cv::Mat &output_image, const std::vector<int> &target_size)
    {
        if (input_image.cols == target_size[1] && input_image.rows == target_size[0])
        {
            if (input_image.data == output_image.data)
            {
                return 1.;
            }
            else
            {
                output_image = input_image.clone();
                return 1.;
            }
        }

        float resize_scale = generate_scale(input_image, target_size);
        int new_shape_w = std::round(input_image.cols * resize_scale);
        int new_shape_h = std::round(input_image.rows * resize_scale);
        float padw = (target_size[1] - new_shape_w) / 2.0f;
        float padh = (target_size[0] - new_shape_h) / 2.0f;

        int top = std::round(padh - letterbox_padding_);
        int bottom = std::round(padh + letterbox_padding_);
        int left = std::round(padw - letterbox_padding_);
        int right = std::round(padw + letterbox_padding_);

        cv::resize(input_image, output_image,
                   cv::Size(new_shape_w, new_shape_h),
                   0, 0, cv::INTER_LINEAR);

        cv::copyMakeBorder(output_image, output_image, top, bottom, left, right,
                           cv::BORDER_CONSTANT, /*border color=*/cv::Scalar(114.0, 114.0, 114.0));
        return resize_scale;
    }

    // Reference: https://github.com/pytorch/vision/blob/main/torchvision/csrc/ops/cpu/nms_kernel.cpp
    torch::Tensor YoloKeypointModel::nms(const torch::Tensor &bboxes, const torch::Tensor &scores, float iou_threshold)
    {
        if (bboxes.numel() == 0)
            return torch::empty({0}, bboxes.options().dtype(torch::kLong));

        auto x1_t = bboxes.select(1, 0).contiguous();
        auto y1_t = bboxes.select(1, 1).contiguous();
        auto x2_t = bboxes.select(1, 2).contiguous();
        auto y2_t = bboxes.select(1, 3).contiguous();

        torch::Tensor areas_t = (x2_t - x1_t) * (y2_t - y1_t);

        auto order_t = std::get<1>(
            scores.sort(/*stable=*/true, /*dim=*/0, /* descending=*/true));

        auto ndets = bboxes.size(0);
        torch::Tensor suppressed_t = torch::zeros({ndets}, bboxes.options().dtype(torch::kByte));
        torch::Tensor keep_t = torch::zeros({ndets}, bboxes.options().dtype(torch::kLong));

        auto suppressed = suppressed_t.data_ptr<uint8_t>();
        auto keep = keep_t.data_ptr<int64_t>();
        auto order = order_t.data_ptr<int64_t>();
        auto x1 = x1_t.data_ptr<float>();
        auto y1 = y1_t.data_ptr<float>();
        auto x2 = x2_t.data_ptr<float>();
        auto y2 = y2_t.data_ptr<float>();
        auto areas = areas_t.data_ptr<float>();

        int64_t num_to_keep = 0;

        for (int64_t _i = 0; _i < ndets; _i++)
        {
            auto i = order[_i];
            if (suppressed[i] == 1)
                continue;
            keep[num_to_keep++] = i;
            auto ix1 = x1[i];
            auto iy1 = y1[i];
            auto ix2 = x2[i];
            auto iy2 = y2[i];
            auto iarea = areas[i];

            for (int64_t _j = _i + 1; _j < ndets; _j++)
            {
                auto j = order[_j];
                if (suppressed[j] == 1)
                    continue;
                auto xx1 = std::max(ix1, x1[j]);
                auto yy1 = std::max(iy1, y1[j]);
                auto xx2 = std::min(ix2, x2[j]);
                auto yy2 = std::min(iy2, y2[j]);

                auto w = std::max(static_cast<float>(0), xx2 - xx1);
                auto h = std::max(static_cast<float>(0), yy2 - yy1);
                auto inter = w * h;
                auto ovr = inter / (iarea + areas[j] - inter);
                if (ovr > iou_threshold)
                    suppressed[j] = 1;
            }
        }
        return keep_t.narrow(0, 0, num_to_keep);
    }

    torch::Tensor YoloKeypointModel::xywh2xyxy(const torch::Tensor &x)
    {
        auto y = torch::empty_like(x);
        auto dw = x.index({"...", 2}).div(2);
        auto dh = x.index({"...", 3}).div(2);
        y.index_put_({"...", 0}, x.index({"...", 0}) - dw);
        y.index_put_({"...", 1}, x.index({"...", 1}) - dh);
        y.index_put_({"...", 2}, x.index({"...", 0}) + dw);
        y.index_put_({"...", 3}, x.index({"...", 1}) + dh);
        return y;
    }

    torch::Tensor YoloKeypointModel::non_max_suppression(torch::Tensor &prediction, int num_keypoints, float conf_thres, float iou_thres, int max_det)
    {
        // YOLOv11 pose output shape: [batch, features, num_predictions]
        // Format: [x, y, w, h, class_scores..., kp1_x, kp1_y, kp1_conf, kp2_x, ...]
        // We need to transpose to [batch, num_predictions, features]
        prediction = prediction.transpose(-1, -2);

        auto batch_size = prediction.size(0);
        auto total_features = prediction.size(2);
        // Each keypoint has 3 values: x, y, confidence
        auto num_keypoint_values = num_keypoints * 3;
        // num_classes = total_features - 4 (bbox) - num_keypoint_values
        auto num_classes = total_features - 4 - num_keypoint_values;
        auto class_score_end_idx = 4 + num_classes;
        auto class_conf_mask = prediction.index({Slice(), Slice(), Slice(4, class_score_end_idx)}).amax(2) > conf_thres;

        // Convert boxes from xywh to xyxy format
        prediction.index_put_({"...", Slice(None, 4)}, xywh2xyxy(prediction.index({"...", Slice(None, 4)})));

        // Prepare output tensor with the maximum possible data slots for this model
        std::vector<torch::Tensor> output;
        for (int batch_index = 0; batch_index < batch_size; batch_index++)
        {
            output.push_back(torch::zeros({0, 6 + num_keypoint_values}, prediction.device()));
        }

        for (int batch_index = 0; batch_index < prediction.size(0); batch_index++)
        {
            auto detections = prediction[batch_index];
            detections = detections.index({class_conf_mask[batch_index]});
            if (detections.size(0) == 0)
            {
                continue;
            }
            auto detections_split = detections.split({4, num_classes, num_keypoint_values}, 1);
            auto box = detections_split[0], cls = detections_split[1], keypoints = detections_split[2];
            auto [conf, class_index] = cls.max(1, true);
            detections = torch::cat({box, conf, class_index.toType(torch::kFloat), keypoints}, 1);
            detections = detections.index({conf.view(-1) > conf_thres});
            int num_detections = detections.size(0);
            if (!num_detections)
            {
                continue;
            }

            // NMS
            // Multiply box coordinates by class index * a large number so NMS doesn't suppress boxes of different classes overlapping each other
            auto class_offset = detections.index({Slice(), Slice{5, 6}}) * 7680;
            auto boxes = detections.index({Slice(), Slice(None, 4)}) + class_offset;
            auto scores = detections.index({Slice(), 4});
            auto selected_indices = nms(boxes, scores, iou_thres);
            selected_indices = selected_indices.index({Slice(None, max_det)});
            output[batch_index] = detections.index({selected_indices});
        }

        return torch::stack(output);
    }

    Keypoint YoloKeypointModel::scale_keypoint(Keypoint output_keypoint, cv::Size original_image_size, cv::Size input_image_size)
    {
        double gain = std::min((double)input_image_size.width / original_image_size.width, (double)input_image_size.height / original_image_size.height);
        double pad0 = std::round((double)(input_image_size.width - original_image_size.width * gain) / 2. - letterbox_padding_);
        double pad1 = std::round((double)(input_image_size.height - original_image_size.height * gain) / 2. - letterbox_padding_);
        double x = output_keypoint.x;
        double y = output_keypoint.y;
        x -= pad0;
        y -= pad1;
        x /= gain;
        y /= gain;

        return Keypoint{output_keypoint.label, output_keypoint.keypoint_label, x, y};
    }

    torch::Tensor YoloKeypointModel::scale_boxes(torch::Tensor &boxes, cv::Size original_image_size, cv::Size input_image_size)
    {
        auto gain = (std::min)((float)input_image_size.height / original_image_size.height, (float)input_image_size.width / original_image_size.width);
        auto pad0 = std::round((float)(input_image_size.width - original_image_size.width * gain) / 2.0 - letterbox_padding_);
        auto pad1 = std::round((float)(input_image_size.height - original_image_size.height * gain) / 2.0 - letterbox_padding_);

        boxes.index_put_({"...", 0}, boxes.index({"...", 0}) - pad0);
        boxes.index_put_({"...", 2}, boxes.index({"...", 2}) - pad0);
        boxes.index_put_({"...", 1}, boxes.index({"...", 1}) - pad1);
        boxes.index_put_({"...", 3}, boxes.index({"...", 3}) - pad1);
        boxes.index_put_({"...", Slice(None, 4)}, boxes.index({"...", Slice(None, 4)}).div(gain));
        return boxes;
    }

    KeypointsStamped YoloKeypointModel::postprocess_output(const torch::Tensor &output, const Header &header, cv::Size original_image_size, cv::Size input_image_size, const cv::Mat &original_image)
    {
        KeypointsStamped result;
        result.header = header;

        // Move output to CPU
        torch::Tensor cpu_output = output.to(torch::kCPU);

        // YOLO output format typically: [batch, num_detections, attributes]
        // attributes include: [x, y, w, h, confidence, class_scores..., keypoints...]
        // For YOLO-pose models, keypoints are typically at the end of the tensor

        if (cpu_output.dim() < 2)
        {
            diagnostics_logger_->warning({}, "Invalid output dimensions");
            return result;
        }

        // Get number of keypoints from the first label in the map
        int num_keypoints = 0;
        if (!label_map_.label_to_keypoints.empty())
        {
            num_keypoints = static_cast<int>(label_map_.label_to_keypoints.front().second.size());
        }

        // NMS
        auto keep = non_max_suppression(cpu_output, num_keypoints, threshold_, iou_threshold_)[0];

        // Get number of detections (size(0) = rows = detections, size(1) = columns = features)
        int64_t num_detections = keep.size(0);

        int valid_detections = 0;

        // Early return if no detections
        if (num_detections == 0)
        {
            DiagnosticsData summary_data;
            summary_data["total_detections"] = 0;
            summary_data["valid_detections"] = 0;
            summary_data["threshold"] = threshold_;
            diagnostics_logger_->info(summary_data);
            return result;
        }

        for (int64_t i = 0; i < num_detections; i++)
        {
            // Extract confidence (typically at index 4)
            float confidence = keep[i][4].item().toFloat();

            // Apply confidence threshold
            if (confidence < threshold_)
            {
                continue;
            }

            valid_detections++;

            // Extract class scores and find the best class
            // The exact structure depends on the YOLO model variant
            // For YOLO with keypoints, format is typically:
            // [x, y, w, h, obj_conf, class_conf, kp1_x, kp1_y, kp1_conf, kp2_x, kp2_y, kp2_conf, ...]

            int num_classes = static_cast<int>(label_map_.size());
            int class_id = keep[i][5].item().toInt();

            // Check if class_id is valid
            if (class_id >= num_classes)
            {
                diagnostics_logger_->warning({}, "Invalid class ID: " + std::to_string(class_id));
                continue;
            }

            // Get the label from the map by index
            Label object_label = label_map_.get_label_at_index(class_id);

            // Log detection info
            DiagnosticsData diag_data;
            diag_data["confidence"] = confidence;
            diag_data["class_id"] = class_id;
            diag_data["stamp"] = header.stamp;
            diagnostics_logger_->info(std::string(magic_enum::enum_name(object_label)) + "-" + std::to_string(i), diag_data);

            // Get keypoint labels for this object label
            const std::vector<KeypointLabel> &keypoint_labels = label_map_.get_keypoint_labels(object_label);

            // Extract keypoints (starting after box + conf + class_id = index 6)
            // YOLOv11 keypoints have 3 values each: x, y, confidence
            int keypoint_start_idx = 6;
            int num_keypoints = static_cast<int>(keypoint_labels.size());

            for (int k = 0; k < num_keypoints; k++)
            {
                // Each keypoint has 3 values: x, y, conf
                int kp_idx = keypoint_start_idx + k * 3;

                if (kp_idx + 2 >= keep.size(1))
                {
                    break;
                }

                // Check keypoint confidence (third value)
                float kp_conf = keep[i][kp_idx + 2].item().toFloat();
                if (kp_conf < threshold_)
                {
                    continue; // Skip low-confidence keypoints
                }

                Keypoint keypoint;
                keypoint.x = keep[i][kp_idx].item().toFloat();
                keypoint.y = keep[i][kp_idx + 1].item().toFloat();
                keypoint = scale_keypoint(keypoint, original_image_size, input_image_size);

                // Set keypoint label from keypoint_labels for this object
                keypoint.keypoint_label = keypoint_labels[k];

                // Set object label
                keypoint.label = object_label;

                result.keypoints.push_back(keypoint);
            }
        }

        DiagnosticsData summary_data;
        summary_data["total_detections"] = (int)num_detections;
        summary_data["valid_detections"] = valid_detections;
        summary_data["threshold"] = threshold_;
        diagnostics_logger_->info(summary_data);

        if (debug_visualization_)
        {
            auto boxes = keep.index({Slice(), Slice(None, 4)});
            auto scaled_boxes = scale_boxes(boxes, original_image_size, input_image_size);
            keep.index_put_({Slice(), Slice(None, 4)}, scaled_boxes);
            visualize_output(original_image, result, keep);
        }

        return result;
    }

    void YoloKeypointModel::visualize_output(const cv::Mat &original_image, const KeypointsStamped &keypoints, const torch::Tensor &detections)
    {
        if (original_image.empty())
            return;

        // Copy image for visualization
        cv::Mat vis_img = original_image.clone();

        // Group keypoints by label to draw connections
        std::map<Label, std::vector<cv::Point>> keypoints_by_label;
        for (const auto &kp : keypoints.keypoints)
        {
            int x = static_cast<int>(std::round(kp.x));
            int y = static_cast<int>(std::round(kp.y));
            keypoints_by_label[kp.label].push_back(cv::Point(x, y));
        }

        // Draw bounding boxes with class labels
        if (detections.defined() && detections.numel() > 0)
        {
            auto detections_cpu = detections.to(torch::kCPU);

            // Collect all label rectangles first to check for overlaps
            struct LabelInfo
            {
                cv::Rect rect;
                std::string text;
                cv::Scalar color;
                cv::Point text_pos;
            };
            std::vector<LabelInfo> labels;

            for (int i = 0; i < detections_cpu.size(0); ++i)
            {
                float x1 = detections_cpu[i][0].item().toFloat();
                float y1 = detections_cpu[i][1].item().toFloat();
                float x2 = detections_cpu[i][2].item().toFloat();
                float y2 = detections_cpu[i][3].item().toFloat();
                float confidence = detections_cpu[i][4].item().toFloat();

                // Get class_id from detections tensor (index 5)
                int class_id = detections_cpu[i][5].item().toInt();
                Label box_label = label_map_.get_label_at_index(class_id);

                auto [b, g, r] = get_color_for_label(box_label).to_bgr_255();
                cv::Scalar color(b, g, r);

                // Format label with confidence score
                std::ostringstream label_stream;
                label_stream << std::string(magic_enum::enum_name(box_label))
                             << " " << std::fixed << std::setprecision(2) << confidence;
                std::string label_text = label_stream.str();

                // Draw box with thicker line
                cv::rectangle(vis_img, cv::Point((int)x1, (int)y1), cv::Point((int)x2, (int)y2), color, 2);

                // Calculate label position
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(label_text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);

                int label_y = (int)y1 - text_size.height - 6;
                int label_x = (int)x1;

                // Check for overlaps with existing labels and offset if needed
                cv::Rect proposed_rect(label_x, label_y, text_size.width + 4, text_size.height + 6);
                int offset_count = 0;
                bool has_overlap = true;
                while (has_overlap && offset_count < 10)
                {
                    has_overlap = false;
                    for (const auto &existing : labels)
                    {
                        if ((proposed_rect & existing.rect).area() > 0)
                        {
                            has_overlap = true;
                            // Move label up
                            label_y -= (text_size.height + 8);
                            proposed_rect = cv::Rect(label_x, label_y, text_size.width + 4, text_size.height + 6);
                            offset_count++;
                            break;
                        }
                    }
                }

                labels.push_back({proposed_rect, label_text, color, cv::Point(label_x + 2, label_y + text_size.height + 2)});
            }

            // Draw all labels
            for (const auto &label : labels)
            {
                cv::rectangle(vis_img, label.rect, label.color, cv::FILLED);
                cv::putText(vis_img, label.text, label.text_pos,
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            }
        }

        // Draw connections between front and back keypoints for each label
        for (const auto &[label, points] : keypoints_by_label)
        {
            if (points.size() >= 2)
            {
                auto [b, g, r] = get_color_for_label(label).to_bgr_255();
                cv::Scalar color(b, g, r);
                // Draw line connecting keypoints (front to back)
                cv::line(vis_img, points[0], points[1], color, 2, cv::LINE_AA);
            }
        }

        // Draw keypoints with labels
        for (const auto &kp : keypoints.keypoints)
        {
            int x = static_cast<int>(std::round(kp.x));
            int y = static_cast<int>(std::round(kp.y));

            auto [b, g, r_val] = get_color_for_label(kp.label).to_bgr_255();
            cv::Scalar color(b, g, r_val);
            std::string kp_name = get_short_name(std::string(magic_enum::enum_name(kp.keypoint_label)));

            // Draw outer circle (white border)
            cv::circle(vis_img, cv::Point(x, y), 7, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
            // Draw inner circle (class color)
            cv::circle(vis_img, cv::Point(x, y), 5, color, -1, cv::LINE_AA);

            // Draw keypoint label with shadow for visibility
            cv::putText(vis_img, kp_name,
                        cv::Point(x + 9, y + 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2); // Shadow
            cv::putText(vis_img, kp_name,
                        cv::Point(x + 8, y + 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1); // Text
        }

        // Show the image in a window (for debug)
        cv::imshow("YOLO Keypoint Visualization", vis_img);
        cv::waitKey(1);
    }

} // namespace auto_battlebot
