#include "shared_memory/shared_memory_writer.hpp"

namespace auto_battlebot
{
    SharedMemoryWriter::SharedMemoryWriter(const std::string &name, size_t size) : name_(name),
                                                                                   size_(size),
                                                                                   fd_(-1),
                                                                                   data_(nullptr)
    {
    }

    SharedMemoryWriter::~SharedMemoryWriter()
    {
        close();
    }

    bool SharedMemoryWriter::open(bool create)
    {
        std::string path = "/dev/shm/" + name_; // Linux specific

        // If already open, close and reopen (supports recovery after restarts)
        if (is_open())
        {
            close();
        }

        int flags = O_RDWR;
        if (create)
        {
            flags |= O_CREAT;
        }

        fd_ = ::open(path.c_str(), flags, 0666); // open with read and write access
        if (fd_ == -1)
        {
            return false;
        }

        // Check current file size
        struct stat st;
        if (fstat(fd_, &st) == -1)
        {
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        // Resize if size doesn't match (either too small or too large)
        if (static_cast<size_t>(st.st_size) != size_)
        {
            if (ftruncate(fd_, size_) == -1)
            {
                ::close(fd_);
                fd_ = -1;
                return false;
            }
        }

        // Map the file to memory
        data_ = static_cast<std::byte *>(mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
        if (data_ == MAP_FAILED)
        {
            ::close(fd_);
            fd_ = -1;
            data_ = nullptr;
            return false;
        }

        return true;
    }

    void SharedMemoryWriter::close()
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
