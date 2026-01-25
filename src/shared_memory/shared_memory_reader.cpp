#include "shared_memory/shared_memory_reader.hpp"

namespace auto_battlebot
{
    SharedMemoryReader::SharedMemoryReader(const std::string &name, size_t size) : name_(name),
                                                                                   size_(size),
                                                                                   fd_(-1),
                                                                                   data_(nullptr)
    {
    }

    SharedMemoryReader::~SharedMemoryReader()
    {
        close();
    }

    bool SharedMemoryReader::open()
    {
        std::string path = "/dev/shm/" + name_; // Linux specific

        // Open existing shared memory file (created by host program)
        fd_ = ::open(path.c_str(), O_RDONLY); // ::open is in the global namespace and disambiguates from SharedMemoryReader's open method
        if (fd_ == -1)
        {
            // Open failed (file might not exist)
            return false;
        }

        // Map the file to memory
        data_ = static_cast<std::byte *>(mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd_, 0));
        if (data_ == MAP_FAILED)
        {
            ::close(fd_);
            fd_ = -1;
            data_ = nullptr;
            return false;
        }

        return true;
    }

    void SharedMemoryReader::close()
    {
        if (data_ == nullptr || data_ == MAP_FAILED)
        {
            return;
        }
        munmap(data_, size_);
        data_ = nullptr;

        if (fd_ == -1)
        {
            return;
        }
        ::close(fd_);
        fd_ = -1;
    }

} // namespace auto_battlebot