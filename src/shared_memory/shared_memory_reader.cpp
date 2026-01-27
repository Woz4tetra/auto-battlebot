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

        // If already open, close and reopen (supports recovery after restarts)
        if (is_open())
        {
            close();
        }

        // Open existing shared memory file (created by host program)
        fd_ = ::open(path.c_str(), O_RDONLY); // ::open is in the global namespace and disambiguates from SharedMemoryReader's open method
        if (fd_ == -1)
        {
            // Open failed (file might not exist)
            return false;
        }

        // Determine actual file size to avoid mapping past EOF (can SIGBUS on access)
        struct stat st;
        if (fstat(fd_, &st) == -1)
        {
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        if (st.st_size <= 0)
        {
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        // If caller provided an expected size, require the file to be at least that large.
        // Mapping past EOF can SIGBUS on access, and mapping a smaller region would break readers.
        if (size_ > 0 && static_cast<size_t>(st.st_size) < size_)
        {
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        mapped_size_ = (size_ > 0) ? size_ : static_cast<size_t>(st.st_size);

        // Map the file to memory
        data_ = static_cast<std::byte *>(mmap(nullptr, mapped_size_, PROT_READ, MAP_SHARED, fd_, 0));
        if (data_ == MAP_FAILED)
        {
            ::close(fd_);
            fd_ = -1;
            data_ = nullptr;
            mapped_size_ = 0;
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
        munmap(data_, mapped_size_ > 0 ? mapped_size_ : size_);
        data_ = nullptr;
        mapped_size_ = 0;

        if (fd_ == -1)
        {
            return;
        }
        ::close(fd_);
        fd_ = -1;
    }

} // namespace auto_battlebot