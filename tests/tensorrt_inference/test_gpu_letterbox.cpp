#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "keypoint_model/yolo_keypoint_model.hpp"
#include "tensorrt_inference/gpu_letterbox.hpp"
#include "tensorrt_inference/letterbox_kernel.hpp"
#include "tensorrt_inference/pinned_mat_allocator.hpp"

namespace auto_battlebot {
namespace {

constexpr int kTensorWidth = 640;
constexpr int kTensorHeight = 384;
constexpr double kGreyLevel = 1.0 / 255.0;
constexpr double kMaxDiffOneGreyLevel = 1.01 * kGreyLevel;
constexpr double kCpuUint8RoundingMeanGrey = 0.25;
constexpr double kMeanDiffHeadroomGrey = 0.05;
constexpr double kMeanDiffBound = (kCpuUint8RoundingMeanGrey + kMeanDiffHeadroomGrey) * kGreyLevel;
const float kPadding = YoloKeypointModelConfiguration{}.letterbox_padding;

bool has_cuda_device() {
    int count = 0;
    const bool ok = cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
    cudaGetLastError();
    return ok;
}

std::vector<float> legacy_yolo_cpu_preprocess(const cv::Mat &image) {
    cv::Mat bgr = image;
    if (image.channels() == 4) cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
    const LetterboxGeometry g =
        compute_letterbox_geometry(bgr.cols, bgr.rows, kTensorWidth, kTensorHeight, kPadding);
    cv::Mat resized;
    if (g.resized_width == bgr.cols && g.resized_height == bgr.rows) {
        resized = bgr.clone();
    } else {
        cv::resize(bgr, resized, cv::Size(g.resized_width, g.resized_height), 0, 0,
                   cv::INTER_LINEAR);
    }
    cv::copyMakeBorder(resized, resized, g.top, kTensorHeight - g.top - g.resized_height, g.left,
                       kTensorWidth - g.left - g.resized_width, cv::BORDER_CONSTANT,
                       cv::Scalar(114.0, 114.0, 114.0));
    constexpr bool kSwapRB = true;
    constexpr bool kCrop = false;
    cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0 / 255.0, cv::Size(), cv::Scalar(), kSwapRB,
                                          kCrop, CV_32F);
    std::vector<float> out(blob.total());
    std::memcpy(out.data(), blob.ptr<float>(), out.size() * sizeof(float));
    return out;
}

class GpuLetterboxTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!has_cuda_device()) GTEST_SKIP() << "no CUDA device";
        ASSERT_EQ(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), cudaSuccess);
        ASSERT_EQ(cudaMalloc(&tensor_, kTensorBytes), cudaSuccess);
    }
    void TearDown() override {
        if (tensor_) cudaFree(tensor_);
        if (stream_) cudaStreamDestroy(stream_);
    }

    DeviceTensorView tensor_view() const { return {tensor_, kTensorHeight, kTensorWidth}; }

    std::vector<float> gpu_letterbox(const cv::Mat &image) {
        EXPECT_TRUE(letterbox_.enqueue(image, tensor_view(), kPadding, stream_));
        EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);
        std::vector<float> out(kTensorElements);
        EXPECT_EQ(cudaMemcpy(out.data(), tensor_, kTensorBytes, cudaMemcpyDeviceToHost),
                  cudaSuccess);
        return out;
    }

    static void expect_close(const std::vector<float> &gpu, const std::vector<float> &cpu) {
        ASSERT_EQ(gpu.size(), cpu.size());
        double max_diff = 0.0;
        double sum_diff = 0.0;
        for (size_t i = 0; i < gpu.size(); ++i) {
            const double diff = std::abs(static_cast<double>(gpu[i]) - cpu[i]);
            max_diff = std::max(max_diff, diff);
            sum_diff += diff;
        }
        EXPECT_LE(max_diff, kMaxDiffOneGreyLevel);
        EXPECT_LE(sum_diff / static_cast<double>(gpu.size()), kMeanDiffBound);
    }

    static cv::Mat random_image(int width, int height, int type) {
        cv::Mat image(height, width, type);
        cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(256));
        return image;
    }

    static constexpr size_t kTensorElements = 3U * kTensorHeight * kTensorWidth;
    static constexpr size_t kTensorBytes = kTensorElements * sizeof(float);

    GpuLetterbox letterbox_;
    cudaStream_t stream_{nullptr};
    float *tensor_{nullptr};
};

constexpr float kMapBackTolerancePx = 2.0f;

class ScaleBoxesProbe : public YoloKeypointModel {
   public:
    ScaleBoxesProbe(YoloKeypointModelConfiguration &config) : YoloKeypointModel(config, nullptr) {}
    using YoloKeypointModel::scale_boxes;
};

TEST(LetterboxGeometryTest, ImageExtentsMapBackThroughScaleBoxes) {
    YoloKeypointModelConfiguration config;
    config.letterbox_padding = kPadding;
    ScaleBoxesProbe probe(config);
    const cv::Size tensor(kTensorWidth, kTensorHeight);
    for (const cv::Size source :
         {cv::Size(1920, 1200), cv::Size(1280, 720), cv::Size(2208, 1242), cv::Size(1920, 1080)}) {
        const LetterboxGeometry g = compute_letterbox_geometry(
            source.width, source.height, kTensorWidth, kTensorHeight, kPadding);
        std::vector<DetectionRow> rows = {
            DetectionRow{static_cast<float>(g.left), static_cast<float>(g.top),
                         static_cast<float>(g.left + g.resized_width),
                         static_cast<float>(g.top + g.resized_height)}};
        probe.scale_boxes(rows, source, tensor);
        EXPECT_NEAR(rows[0][detection_row::kX1], 0.0f, kMapBackTolerancePx) << source;
        EXPECT_NEAR(rows[0][detection_row::kY1], 0.0f, kMapBackTolerancePx) << source;
        EXPECT_NEAR(rows[0][detection_row::kX2], static_cast<float>(source.width),
                    kMapBackTolerancePx)
            << source;
        EXPECT_NEAR(rows[0][detection_row::kY2], static_cast<float>(source.height),
                    kMapBackTolerancePx)
            << source;
    }
}

TEST(LetterboxGeometryTest, MatchesZedOneFrame) {
    constexpr double kScale = 384.0 / 1200.0;
    const int kResizedWidth = static_cast<int>(std::lround(1920 * kScale));
    const int kSidePad = (640 - kResizedWidth) / 2;
    const LetterboxGeometry g = compute_letterbox_geometry(1920, 1200, 640, 384, kPadding);
    EXPECT_EQ(g.resized_width, kResizedWidth);
    EXPECT_EQ(g.resized_height, 384);
    EXPECT_EQ(g.left, kSidePad);
    EXPECT_EQ(g.top, 0);
    EXPECT_TRUE(g.fills_tensor);
}

TEST_F(GpuLetterboxTest, PageableBgrMatchesCpuLetterbox) {
    const cv::Mat image = random_image(1920, 1200, CV_8UC3);
    const std::vector<float> gpu = gpu_letterbox(image);
    EXPECT_FALSE(letterbox_.last_was_zero_copy());
    expect_close(gpu, legacy_yolo_cpu_preprocess(image));
}

TEST_F(GpuLetterboxTest, PinnedFrameIsReadInPlace) {
    const cv::Mat source = random_image(1920, 1200, CV_8UC3);
    cv::Mat pinned;
    pinned.allocator = PinnedMatAllocator::instance();
    source.copyTo(pinned);

    const std::vector<float> gpu = gpu_letterbox(pinned);
    EXPECT_TRUE(letterbox_.last_was_zero_copy());
    expect_close(gpu, legacy_yolo_cpu_preprocess(source));
}

TEST_F(GpuLetterboxTest, BgraMatchesBgr) {
    const cv::Mat bgr = random_image(1280, 720, CV_8UC3);
    cv::Mat bgra;
    cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
    expect_close(gpu_letterbox(bgra), legacy_yolo_cpu_preprocess(bgr));
}

TEST_F(GpuLetterboxTest, RoiUsesRowStep) {
    const cv::Mat full = random_image(2000, 1300, CV_8UC3);
    const cv::Rect roi(40, 50, 1920, 1200);
    const cv::Mat pageable_roi = full(roi);
    ASSERT_FALSE(pageable_roi.isContinuous());
    expect_close(gpu_letterbox(pageable_roi), legacy_yolo_cpu_preprocess(pageable_roi.clone()));

    cv::Mat pinned_full;
    pinned_full.allocator = PinnedMatAllocator::instance();
    full.copyTo(pinned_full);
    const cv::Mat pinned_roi = pinned_full(roi);
    const std::vector<float> gpu = gpu_letterbox(pinned_roi);
    EXPECT_TRUE(letterbox_.last_was_zero_copy());
    expect_close(gpu, legacy_yolo_cpu_preprocess(pinned_roi.clone()));
}

TEST_F(GpuLetterboxTest, ManySourceShapesStayWithinOneGreyLevel) {
    for (const int width : {1920, 1280, 2208, 1000, 777}) {
        for (const int height : {1200, 720, 1242, 1080, 555}) {
            const cv::Mat image = random_image(width, height, CV_8UC3);
            expect_close(gpu_letterbox(image), legacy_yolo_cpu_preprocess(image));
        }
    }
}

TEST_F(GpuLetterboxTest, TensorSizedImageIsExact) {
    const cv::Mat image = random_image(kTensorWidth, kTensorHeight, CV_8UC3);
    const std::vector<float> gpu = gpu_letterbox(image);
    const std::vector<float> cpu = legacy_yolo_cpu_preprocess(image);
    ASSERT_EQ(gpu.size(), cpu.size());
    for (size_t i = 0; i < gpu.size(); ++i) ASSERT_NEAR(gpu[i], cpu[i], 1e-6) << "at " << i;
}

TEST_F(GpuLetterboxTest, RejectsUnsupportedImages) {
    const cv::Mat gray(1200, 1920, CV_8UC1, cv::Scalar(0));
    EXPECT_FALSE(letterbox_.enqueue(gray, tensor_view(), kPadding, stream_));
    EXPECT_FALSE(letterbox_.enqueue(cv::Mat(), tensor_view(), kPadding, stream_));
}

TEST_F(GpuLetterboxTest, AllocatorReusesReleasedBuffers) {
    PinnedMatAllocator pool;
    PinnedMatAllocator *allocator = &pool;
    void *first_data = nullptr;
    {
        cv::Mat frame;
        frame.allocator = allocator;
        frame.create(1200, 1920, CV_8UC3);
        first_data = frame.data;
        EXPECT_EQ(allocator->pooled_buffers(), 0u);
    }
    EXPECT_EQ(allocator->pooled_buffers(), 1u);

    cv::Mat frame;
    frame.allocator = allocator;
    frame.create(1200, 1920, CV_8UC3);
    EXPECT_EQ(frame.data, first_data);
    EXPECT_EQ(allocator->pooled_buffers(), 0u);
}

}  // namespace
}  // namespace auto_battlebot
