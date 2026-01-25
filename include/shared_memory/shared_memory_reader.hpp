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

        template <typename T>
        const T *read_at(size_t offset) const
        {
            return reinterpret_cast<const T *>(data_ + offset);
        }

    private:
        std::string name_;
        size_t size_;
        int fd_;
        std::byte *data_;
    };
} // namespace auto_battlebot
