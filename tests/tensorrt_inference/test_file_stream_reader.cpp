#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "tensorrt_inference/file_stream_reader.hpp"
#include "tensorrt_inference/trt_engine.hpp"

#if AB_TRT_HAS_STREAM_READER_V2

namespace auto_battlebot {
namespace {

constexpr size_t kFileBytes = 1 << 16;

std::vector<char> patterned_bytes() {
    std::vector<char> bytes(kFileBytes);
    for (size_t i = 0; i < bytes.size(); ++i) bytes[i] = static_cast<char>((i * 31 + 7) & 0xff);
    return bytes;
}

class FileStreamReaderTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (cudaFree(nullptr) != cudaSuccess) GTEST_SKIP() << "no CUDA device";
        path_ = std::filesystem::temp_directory_path() / "auto_battlebot_stream_reader.bin";
        expected_ = patterned_bytes();
        std::ofstream(path_, std::ios::binary).write(expected_.data(), expected_.size());
        ASSERT_EQ(cudaStreamCreate(&stream_), cudaSuccess);
    }

    void TearDown() override {
        if (stream_) cudaStreamDestroy(stream_);
        std::filesystem::remove(path_);
    }

    std::vector<char> read_to_device(FileStreamReader& reader, int64_t bytes, int64_t* returned) {
        void* device = nullptr;
        EXPECT_EQ(cudaMalloc(&device, static_cast<size_t>(bytes)), cudaSuccess);
        *returned = reader.read(device, bytes, stream_);
        std::vector<char> host(static_cast<size_t>(bytes));
        EXPECT_EQ(
            cudaMemcpy(host.data(), device, static_cast<size_t>(bytes), cudaMemcpyDeviceToHost),
            cudaSuccess);
        cudaFree(device);
        return host;
    }

    std::filesystem::path path_;
    std::vector<char> expected_;
    cudaStream_t stream_ = nullptr;
};

TEST_F(FileStreamReaderTest, ReadsIntoADeviceDestination) {
    FileStreamReader reader(path_.string());
    ASSERT_TRUE(reader.is_open());
    int64_t returned = 0;
    const std::vector<char> got = read_to_device(reader, 4096, &returned);
    ASSERT_EQ(returned, 4096);
    EXPECT_TRUE(std::equal(got.begin(), got.end(), expected_.begin()));
}

TEST_F(FileStreamReaderTest, ConsecutiveReadsDoNotOverwriteEarlierBytes) {
    FileStreamReader reader(path_.string());
    std::vector<char> assembled;
    for (int chunk = 0; chunk < 64; ++chunk) {
        int64_t returned = 0;
        const std::vector<char> got = read_to_device(reader, 1024, &returned);
        ASSERT_EQ(returned, 1024);
        assembled.insert(assembled.end(), got.begin(), got.end());
    }
    EXPECT_EQ(assembled, expected_);
}

TEST_F(FileStreamReaderTest, SeekAfterReadingToEofRewinds) {
    FileStreamReader reader(path_.string());
    void* device = nullptr;
    ASSERT_EQ(cudaMalloc(&device, kFileBytes * 2), cudaSuccess);
    EXPECT_EQ(reader.read(device, kFileBytes * 2, stream_), static_cast<int64_t>(kFileBytes));
    EXPECT_TRUE(reader.seek(0, nvinfer1::SeekPosition::kSET));
    EXPECT_EQ(reader.read(device, 16, stream_), 16);
    std::vector<char> head(16);
    ASSERT_EQ(cudaMemcpy(head.data(), device, 16, cudaMemcpyDeviceToHost), cudaSuccess);
    EXPECT_TRUE(std::equal(head.begin(), head.end(), expected_.begin()));
    cudaFree(device);
}

TEST(TrtLoggingThreadTest, QuietGuardDoesNotSilenceOtherThreads) {
    TrtEngine::ScopedQuietLogging quiet;
    EXPECT_TRUE(TrtEngine::ScopedQuietLogging::active_on_this_thread());
    bool other_thread_quiet = true;
    std::thread([&] {
        other_thread_quiet = TrtEngine::ScopedQuietLogging::active_on_this_thread();
    }).join();
    EXPECT_FALSE(other_thread_quiet);
}

}  // namespace
}  // namespace auto_battlebot

#endif
