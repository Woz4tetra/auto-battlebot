#pragma once
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

namespace auto_battlebot
{
    class SharedMemoryReader
    {
    public:
        SharedMemoryReader(const std::string &name, size_t size);
        ~SharedMemoryReader();
        bool open();
        void close();
        bool is_open() const { return data_ != nullptr; }
        const void *data() const { return data_; }
        size_t mapped_size() const { return mapped_size_; }

        template <typename T>
        const T *read_at(size_t offset) const
        {
            if (data_ == nullptr)
            {
                return nullptr;
            }
            if (mapped_size_ > 0 && (offset + sizeof(T) > mapped_size_))
            {
                return nullptr;
            }
            return reinterpret_cast<const T *>(data_ + offset);
        }

    private:
        std::string name_;
        size_t size_;
        int fd_;
        std::byte *data_;
        size_t mapped_size_ = 0;
    };
} // namespace auto_battlebot
