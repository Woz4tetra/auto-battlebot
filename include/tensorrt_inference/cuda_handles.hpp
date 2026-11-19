#pragma once

struct CUstream_st;
struct CUgraphExec_st;

typedef CUstream_st *cudaStream_t;
typedef CUgraphExec_st *cudaGraphExec_t;

namespace auto_battlebot {

struct DeviceTensorView {
    float *data = nullptr;
    int height = 0;
    int width = 0;
};

}  // namespace auto_battlebot
