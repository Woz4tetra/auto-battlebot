#pragma once
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

namespace auto_battlebot
{
    class SharedMemoryWriter
    {
    public:
        SharedMemoryWriter(const std::string &name, size_t size);
        ~SharedMemoryWriter();
        bool open(bool create = true);
        void close();
        bool is_open() const { return data_ != nullptr; }

        template <typename T>
        void write_at(size_t offset, const T &value);

        void write_bytes(size_t offset, const std::byte *src, size_t len);

    private:
        std::string name_;
        size_t size_;
        int fd_;
        std::byte *data_;
    };
} // namespace auto_battlebot
