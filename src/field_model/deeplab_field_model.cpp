#include "field_model/deeplab_field_model.hpp"

namespace auto_battlebot
{
    DeepLabFieldModel::DeepLabFieldModel(const std::string &model_path, DeepLabModelType model_type, int image_size)
        : model_path_(model_path), model_type_(model_type), image_size_(image_size),
          device_(torch::kCPU), initialized_(false)
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("deeplab_field_model");
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

    bool DeepLabFieldModel::initialize()
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
            model_.forward({warmup_input});

            initialized_ = true;
            diagnostics_logger_->info({}, "DeepLabFieldModel initialized successfully");
            return true;
        }
        catch (const c10::Error &e)
        {
            std::cerr << "Error loading model: " << e.what() << std::endl;
            return false;
        }
    }

    FieldMaskStamped DeepLabFieldModel::update(RgbImage image)
    {
        FunctionTimer timer(diagnostics_logger_, "update", 100.0); // Warn if > 100ms

        if (!initialized_)
        {
            diagnostics_logger_->error({}, "Model not initialized");
            return FieldMaskStamped{};
        }

        // Store original dimensions
        int original_height = image.image.rows;
        int original_width = image.image.cols;

        // Preprocess the image
        torch::Tensor input_tensor = preprocess_image(image.image);

        // Run inference
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input_tensor);

        torch::Tensor output;
        {
            torch::NoGradGuard no_grad;
            auto model_output = model_.forward(inputs);

            // Handle different output formats
            if (model_output.isTuple())
            {
                // DeepLabV3 returns a dict-like tuple with 'out' key
                auto output_dict = model_output.toTuple()->elements();
                if (!output_dict.empty())
                {
                    // Typically the first element is the segmentation output
                    if (output_dict[0].isGenericDict())
                    {
                        auto dict = output_dict[0].toGenericDict();
                        output = dict.at("out").toTensor();
                    }
                    else if (output_dict[0].isTensor())
                    {
                        output = output_dict[0].toTensor();
                    }
                }
            }
            else if (model_output.isGenericDict())
            {
                auto dict = model_output.toGenericDict();
                output = dict.at("out").toTensor();
            }
            else if (model_output.isTensor())
            {
                output = model_output.toTensor();
            }
        }

        // Postprocess the output
        cv::Mat mask = postprocess_output(output, original_height, original_width);

        // Create FieldMaskStamped result
        FieldMaskStamped result;
        result.header = image.header;
        result.mask.label = Label::FIELD;
        result.mask.mask = mask;

        return result;
    }

    torch::Tensor DeepLabFieldModel::preprocess_image(const cv::Mat &image)
    {
        cv::Mat rgb_image;
        if (image.channels() == 3)
        {
            rgb_image = image;
        }
        else if (image.channels() == 4)
        {
            cv::cvtColor(image, rgb_image, cv::COLOR_BGRA2RGB);
        }
        else
        {
            cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
        }

        // Resize to model input size
        cv::Mat resized;
        cv::resize(rgb_image, resized, cv::Size(image_size_, image_size_));

        // Convert to float and normalize to [0, 1]
        cv::Mat float_image;
        resized.convertTo(float_image, CV_32F, 1.0 / 255.0);

        // ImageNet normalization
        std::vector<float> mean = {0.485f, 0.456f, 0.406f};
        std::vector<float> std = {0.229f, 0.224f, 0.225f};

        std::vector<cv::Mat> channels(3);
        cv::split(float_image, channels);

        for (int i = 0; i < 3; i++)
        {
            channels[i] = (channels[i] - mean[i]) / std[i];
        }

        cv::merge(channels, float_image);

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

    cv::Mat DeepLabFieldModel::postprocess_output(const torch::Tensor &output, int original_height, int original_width)
    {
        // Get the segmentation mask (argmax over classes)
        torch::Tensor segmentation = output.squeeze(0).argmax(0);

        // Convert to CPU and to uint8
        segmentation = segmentation.to(torch::kCPU).to(torch::kUInt8);

        // Create OpenCV Mat
        cv::Mat mask(image_size_, image_size_, CV_8UC1, segmentation.data_ptr<uint8_t>());
        mask = mask.clone(); // Clone to ensure data ownership

        // Resize back to original dimensions
        cv::Mat resized_mask;
        cv::resize(mask, resized_mask, cv::Size(original_width, original_height), 0, 0, cv::INTER_NEAREST);

        return resized_mask;
    }

} // namespace auto_battlebot
