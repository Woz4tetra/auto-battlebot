#include "keypoint_model/yolo_keypoint_model.hpp"
#include "enums/label.hpp"
#include <magic_enum.hpp>

namespace auto_battlebot
{
    YoloKeypointModel::YoloKeypointModel(YoloKeypointModelConfiguration &config)
        : model_path_(config.model_path),
          threshold_(config.threshold),
          image_size_(config.image_size),
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

        // Preprocess the image
        torch::Tensor input_tensor = preprocess_image(image.image);

        // Run inference
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input_tensor);

        torch::Tensor output;
        {
            torch::NoGradGuard no_grad;
            auto model_output = model_.forward(inputs);

            // YOLO typically returns a tuple or list of tensors
            if (model_output.isTuple())
            {
                auto output_tuple = model_output.toTuple()->elements();
                if (!output_tuple.empty())
                {
                    output = output_tuple[0].toTensor();
                }
            }
            else if (model_output.isTensor())
            {
                output = model_output.toTensor();
            }
            else if (model_output.isList())
            {
                auto output_list = model_output.toList();
                if (output_list.size() > 0)
                {
                    output = output_list.get(0).toTensor();
                }
            }
        }

        // Postprocess the output
        KeypointsStamped result = postprocess_output(output, image.header);

        return result;
    }

    torch::Tensor YoloKeypointModel::preprocess_image(const cv::Mat &image)
    {
        cv::Mat rgb_image;
        if (image.channels() == 3)
        {
            cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
        }
        else if (image.channels() == 4)
        {
            cv::cvtColor(image, rgb_image, cv::COLOR_BGRA2RGB);
        }
        else
        {
            rgb_image = image;
        }

        // Resize to model input size
        cv::Mat resized;
        cv::resize(rgb_image, resized, cv::Size(image_size_, image_size_));

        // Convert to float and normalize to [0, 1]
        cv::Mat float_image;
        resized.convertTo(float_image, CV_32F, 1.0 / 255.0);

        // Convert to tensor: HWC -> CHW
        torch::Tensor tensor = torch::from_blob(
                                   float_image.data,
                                   {image_size_, image_size_, 3},
                                   torch::kFloat32)
                                   .clone();

        tensor = tensor.permute({2, 0, 1}); // HWC -> CHW
        tensor = tensor.unsqueeze(0);       // Add batch dimension

        return tensor.to(device_);
    }

    KeypointsStamped YoloKeypointModel::postprocess_output(const torch::Tensor &output, const Header &header)
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

        // Get number of detections
        int64_t num_detections = cpu_output.size(1);

        auto accessor = cpu_output.accessor<float, 3>();

        int valid_detections = 0;

        for (int64_t i = 0; i < num_detections; i++)
        {
            // Extract confidence (typically at index 4)
            float confidence = accessor[0][i][4];

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

            int num_classes = static_cast<int>(label_map_.label_to_keypoints.size());
            float class_score = accessor[0][i][5];
            int class_id = 0;

            // For each class, find max score
            for (int c = 0; c < num_classes; c++)
            {
                float score = accessor[0][i][5 + c];
                if (score > class_score)
                {
                    class_score = score;
                    class_id = c;
                }
            }

            // Check if class_id is valid
            if (class_id >= num_classes)
            {
                diagnostics_logger_->warning({}, "Invalid class ID: " + std::to_string(class_id));
                continue;
            }

            // Get the label from the map (get the nth label)
            auto it = label_map_.label_to_keypoints.begin();
            std::advance(it, class_id);
            Label object_label = it->first;

            // Log detection info
            DiagnosticsData diag_data;
            diag_data["confidence"] = confidence;
            diag_data["class_score"] = class_score;
            diag_data["class_id"] = class_id;
            diagnostics_logger_->info(std::string(magic_enum::enum_name(object_label)) + "-" + std::to_string(i), diag_data);

            // Get keypoint labels for this object label
            const std::vector<KeypointLabel> &keypoint_labels = label_map_.get_keypoint_labels(object_label);

            // Extract keypoints (starting after class scores)
            int keypoint_start_idx = 5 + num_classes;
            int num_keypoints = static_cast<int>(keypoint_labels.size());

            for (int k = 0; k < num_keypoints; k++)
            {
                int kp_idx = keypoint_start_idx + k * 2;

                if (kp_idx + 1 >= cpu_output.size(2))
                {
                    break;
                }

                Keypoint keypoint;
                keypoint.x = accessor[0][i][kp_idx];
                keypoint.y = accessor[0][i][kp_idx + 1];

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

        return result;
    }

} // namespace auto_battlebot
