#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "tensorrt_inference/gpu_letterbox.hpp"
#include "tensorrt_inference/letterbox_kernel.hpp"
#include "tensorrt_inference/pinned_mat_allocator.hpp"

namespace auto_battlebot {
namespace {

constexpr int kTensorWidth = 640;
constexpr int kTensorHeight = 384;
constexpr float kPadding = 0.1f;  // YoloKeypointModelConfiguration's default

bool has_cuda_device() {
    int count = 0;
    const bool ok = cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
    cudaGetLastError();
    return ok;
}

// The CPU preprocess YoloKeypointModel ran before the GPU kernel replaced it: cv::resize
// INTER_LINEAR, a 114-grey border, then blobFromImage's 1/255, BGR to RGB and CHW pack.
std::vector<float> cpu_letterbox(const cv::Mat &image) {
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
    cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0 / 255.0, cv::Size(), cv::Scalar(),
                                          /*swapRB=*/true, /*crop=*/false, CV_32F);
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

    std::vector<float> gpu_letterbox(const cv::Mat &image) {
        EXPECT_TRUE(
            letterbox_.enqueue(image, kTensorWidth, kTensorHeight, kPadding, tensor_, stream_));
        EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);
        std::vector<float> out(kTensorElements);
        EXPECT_EQ(cudaMemcpy(out.data(), tensor_, kTensorBytes, cudaMemcpyDeviceToHost),
                  cudaSuccess);
        return out;
    }

    // cv::resize rounds each 8-bit output and quantizes its weights to 11 bits, and the kernel
    // does neither, so allow one grey level per sample and well under that on average.
    static void expect_close(const std::vector<float> &gpu, const std::vector<float> &cpu) {
        ASSERT_EQ(gpu.size(), cpu.size());
        double max_diff = 0.0;
        double sum_diff = 0.0;
        for (size_t i = 0; i < gpu.size(); ++i) {
            const double diff = std::abs(static_cast<double>(gpu[i]) - cpu[i]);
            max_diff = std::max(max_diff, diff);
            sum_diff += diff;
        }
        EXPECT_LE(max_diff, 1.01 / 255.0);
        EXPECT_LE(sum_diff / static_cast<double>(gpu.size()), 0.3 / 255.0);
    }

    static cv::Mat random_image(int width, int height, int type) {
        cv::Mat image(height, width, type);
        cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(256));
        // Smooth it: bilinear error on white noise is the worst case, not what a camera sends.
        cv::GaussianBlur(image, image, cv::Size(5, 5), 1.5);
        return image;
    }

    static constexpr size_t kTensorElements = 3U * kTensorHeight * kTensorWidth;
    static constexpr size_t kTensorBytes = kTensorElements * sizeof(float);

    GpuLetterbox letterbox_;
    cudaStream_t stream_{nullptr};
    float *tensor_{nullptr};
};

TEST(LetterboxGeometryTest, MatchesZedOneFrame) {
    // 1920x1200 fits the height: 0.32 scale, 614x384, 13 px of grey each side.
    const LetterboxGeometry g = compute_letterbox_geometry(1920, 1200, 640, 384, kPadding);
    EXPECT_EQ(g.resized_width, 614);
    EXPECT_EQ(g.resized_height, 384);
    EXPECT_EQ(g.left, 13);
    EXPECT_EQ(g.top, 0);
    EXPECT_TRUE(g.fills_tensor);
}

TEST_F(GpuLetterboxTest, PageableBgrMatchesCpuLetterbox) {
    const cv::Mat image = random_image(1920, 1200, CV_8UC3);
    const std::vector<float> gpu = gpu_letterbox(image);
    EXPECT_FALSE(letterbox_.last_was_zero_copy());
    expect_close(gpu, cpu_letterbox(image));
}

TEST_F(GpuLetterboxTest, PinnedFrameIsReadInPlace) {
    const cv::Mat source = random_image(1920, 1200, CV_8UC3);
    cv::Mat pinned;
    pinned.allocator = PinnedMatAllocator::instance();
    source.copyTo(pinned);

    const std::vector<float> gpu = gpu_letterbox(pinned);
    EXPECT_TRUE(letterbox_.last_was_zero_copy());
    expect_close(gpu, cpu_letterbox(source));
}

TEST_F(GpuLetterboxTest, BgraMatchesBgr) {
    const cv::Mat bgr = random_image(1280, 720, CV_8UC3);
    cv::Mat bgra;
    cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
    expect_close(gpu_letterbox(bgra), cpu_letterbox(bgr));
}

// A ROI has a row step wider than its pixels, on both the copy and the in-place path.
TEST_F(GpuLetterboxTest, RoiUsesRowStep) {
    const cv::Mat full = random_image(2000, 1300, CV_8UC3);
    const cv::Rect roi(40, 50, 1920, 1200);
    const cv::Mat pageable_roi = full(roi);
    ASSERT_FALSE(pageable_roi.isContinuous());
    expect_close(gpu_letterbox(pageable_roi), cpu_letterbox(pageable_roi.clone()));

    cv::Mat pinned_full;
    pinned_full.allocator = PinnedMatAllocator::instance();
    full.copyTo(pinned_full);
    const cv::Mat pinned_roi = pinned_full(roi);
    const std::vector<float> gpu = gpu_letterbox(pinned_roi);
    EXPECT_TRUE(letterbox_.last_was_zero_copy());
    expect_close(gpu, cpu_letterbox(pinned_roi.clone()));
}

// At the tensor's own size there is no resize, so the result is exact.
TEST_F(GpuLetterboxTest, TensorSizedImageIsExact) {
    const cv::Mat image = random_image(kTensorWidth, kTensorHeight, CV_8UC3);
    const std::vector<float> gpu = gpu_letterbox(image);
    const std::vector<float> cpu = cpu_letterbox(image);
    ASSERT_EQ(gpu.size(), cpu.size());
    for (size_t i = 0; i < gpu.size(); ++i) ASSERT_NEAR(gpu[i], cpu[i], 1e-6) << "at " << i;
}

TEST_F(GpuLetterboxTest, RejectsUnsupportedImages) {
    const cv::Mat gray(1200, 1920, CV_8UC1, cv::Scalar(0));
    EXPECT_FALSE(letterbox_.enqueue(gray, kTensorWidth, kTensorHeight, kPadding, tensor_, stream_));
    EXPECT_FALSE(
        letterbox_.enqueue(cv::Mat(), kTensorWidth, kTensorHeight, kPadding, tensor_, stream_));
}

TEST_F(GpuLetterboxTest, AllocatorReusesReleasedBuffers) {
    // Earlier tests may have left buffers in the process-wide pool, so measure around the release.
    PinnedMatAllocator *allocator = PinnedMatAllocator::instance();
    void *first_data = nullptr;
    size_t pooled_while_held = 0;
    {
        cv::Mat frame;
        frame.allocator = allocator;
        frame.create(1200, 1920, CV_8UC3);
        first_data = frame.data;
        pooled_while_held = allocator->pooled_buffers();
    }
    EXPECT_EQ(allocator->pooled_buffers(), pooled_while_held + 1);

    cv::Mat frame;
    frame.allocator = allocator;
    frame.create(1200, 1920, CV_8UC3);
    EXPECT_EQ(frame.data, first_data);
    EXPECT_EQ(allocator->pooled_buffers(), pooled_while_held);
}

}  // namespace
}  // namespace auto_battlebot
