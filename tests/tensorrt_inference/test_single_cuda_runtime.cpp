#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <fstream>
#include <set>
#include <string>

namespace {

std::set<std::string> loaded_cudart_libraries() {
    std::ifstream maps("/proc/self/maps");
    std::set<std::string> found;
    std::string line;
    while (std::getline(maps, line)) {
        auto slash = line.rfind('/');
        if (slash == std::string::npos) continue;
        std::string name = line.substr(slash + 1);
        if (name.rfind("libcudart", 0) == 0) found.insert(name);
    }
    return found;
}

}  // namespace

TEST(SingleCudaRuntime, NoSharedCudartIsLoadedAfterTheRuntimeInitializes) {
    int count = 0;
    cudaGetDeviceCount(&count);
    cudaFree(nullptr);
    auto shared = loaded_cudart_libraries();
    EXPECT_TRUE(shared.empty()) << "dynamic cudart mapped alongside the static one: "
                                << *shared.begin();
}
