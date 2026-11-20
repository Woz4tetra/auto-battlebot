#pragma once

#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <NvInferVersion.h>
#include <cuda_runtime.h>

#include <cstdint>
#include <fstream>
#include <new>
#include <string>
#include <vector>

#define AB_TRT_HAS_STREAM_READER_V2 \
    (NV_TENSORRT_MAJOR > 10 || (NV_TENSORRT_MAJOR == 10 && NV_TENSORRT_MINOR >= 7))

namespace auto_battlebot {

#if AB_TRT_HAS_STREAM_READER_V2
using TrtStreamReaderBase = nvinfer1::IStreamReaderV2;
#else
using TrtStreamReaderBase = nvinfer1::IStreamReader;
#endif

class FileStreamReader final : public TrtStreamReaderBase {
   public:
    explicit FileStreamReader(const std::string& path) : file_(path, std::ios::binary) {}

    bool is_open() const { return file_.is_open(); }

#if AB_TRT_HAS_STREAM_READER_V2
    int64_t read(void* destination, int64_t nbBytes, cudaStream_t stream) noexcept override {
        if (nbBytes <= 0) return 0;
        try {
            staging_.resize(static_cast<size_t>(nbBytes));
        } catch (const std::bad_alloc&) {
            return -1;
        }
        file_.read(staging_.data(), static_cast<std::streamsize>(nbBytes));
        const int64_t bytes_read = static_cast<int64_t>(file_.gcount());
        if (bytes_read <= 0) return bytes_read;

        if (cudaMemcpyAsync(destination, staging_.data(), static_cast<size_t>(bytes_read),
                            cudaMemcpyDefault, stream) != cudaSuccess) {
            return -1;
        }
        return bytes_read;
    }

    bool seek(int64_t offset, nvinfer1::SeekPosition where) noexcept override {
        std::ios_base::seekdir direction = std::ios_base::beg;
        switch (where) {
            case nvinfer1::SeekPosition::kSET:
                direction = std::ios_base::beg;
                break;
            case nvinfer1::SeekPosition::kCUR:
                direction = std::ios_base::cur;
                break;
            case nvinfer1::SeekPosition::kEND:
                direction = std::ios_base::end;
                break;
        }
        file_.clear();
        file_.seekg(static_cast<std::streamoff>(offset), direction);
        return file_.good();
    }
#else
    int64_t read(void* destination, int64_t nbBytes) noexcept override {
        if (nbBytes <= 0) return 0;
        file_.read(static_cast<char*>(destination), static_cast<std::streamsize>(nbBytes));
        return static_cast<int64_t>(file_.gcount());
    }
#endif

   private:
    std::ifstream file_;
#if AB_TRT_HAS_STREAM_READER_V2
    std::vector<char> staging_;
#endif
};

}  // namespace auto_battlebot
