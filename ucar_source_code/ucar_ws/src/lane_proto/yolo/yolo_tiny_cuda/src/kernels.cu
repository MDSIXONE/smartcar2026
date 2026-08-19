// 手写 CUDA kernel。只用 CUDA 10.2 就存在的基础特性，
// 以便在 Jetson Nano (sm_53 / JetPack 4.6 / CUDA 10.2) 上也能编译。
// 不依赖 cuDNN / cuBLAS。
#include "yolo.h"
#include <cuda_runtime.h>
#include <cstdio>
#include <cstring>

namespace yolo {

#define CUDA_CHECK(call, err) do {                                        \
    cudaError_t _e = (call);                                              \
    if (_e != cudaSuccess) {                                              \
        if (err) *(err) = std::string("CUDA 错误 @" __FILE__ ":")         \
                        + std::to_string(__LINE__) + " -> "               \
                        + cudaGetErrorString(_e);                         \
        return false;                                                     \
    }                                                                     \
} while (0)

static __device__ __forceinline__ float leaky_act(float x) { return x > 0.f ? x : 0.1f * x; }

// ---------------------------------------------------------------- 卷积
// 每个 block 负责 1 个输出通道的 TH x TW 输出瓦片。
// 输入按 CC 个通道一批搬进 shared memory，3x3 窗口在瓦片内复用 9 次。
template<int KS, int ST, int TW, int TH, int CC>
__global__ void conv_kernel(const float* __restrict__ in, int in_c, int in_h, int in_w,
                            const float* __restrict__ wgt, const float* __restrict__ bias,
                            float* __restrict__ out, int out_h, int out_w,
                            int pad, int do_leaky)
{
    const int TIW = TW * ST + KS - 1;      // shared 输入瓦片宽
    const int TIH = TH * ST + KS - 1;
    __shared__ float s_in[CC * TIH * TIW];
    __shared__ float s_w[CC * KS * KS];

    const int tx = threadIdx.x, ty = threadIdx.y;
    const int tid = ty * TW + tx;
    const int nthreads = TW * TH;

    const int ox = blockIdx.x * TW + tx;
    const int oy = blockIdx.y * TH + ty;
    const int f  = blockIdx.z;

    const int ix0 = blockIdx.x * TW * ST - pad;
    const int iy0 = blockIdx.y * TH * ST - pad;

    float acc = 0.f;
    const float* wf = wgt + (size_t)f * in_c * KS * KS;

    for (int c0 = 0; c0 < in_c; c0 += CC) {
        const int cn = (in_c - c0) < CC ? (in_c - c0) : CC;

        // --- 搬输入瓦片 ---
        for (int idx = tid; idx < cn * TIH * TIW; idx += nthreads) {
            const int cc = idx / (TIH * TIW);
            const int rem = idx - cc * TIH * TIW;
            const int ty2 = rem / TIW, tx2 = rem - ty2 * TIW;
            const int iy = iy0 + ty2, ix = ix0 + tx2;
            float v = 0.f;
            if (iy >= 0 && iy < in_h && ix >= 0 && ix < in_w)
                v = in[((size_t)(c0 + cc) * in_h + iy) * in_w + ix];
            s_in[idx] = v;
        }
        // --- 搬权重 ---
        for (int idx = tid; idx < cn * KS * KS; idx += nthreads)
            s_w[idx] = wf[(size_t)c0 * KS * KS + idx];

        __syncthreads();

        if (ox < out_w && oy < out_h) {
            #pragma unroll 1
            for (int cc = 0; cc < cn; ++cc) {
                const float* sp = s_in + cc * TIH * TIW;
                const float* wp = s_w + cc * KS * KS;
                #pragma unroll
                for (int ky = 0; ky < KS; ++ky) {
                    const float* row = sp + (ty * ST + ky) * TIW + tx * ST;
                    #pragma unroll
                    for (int kx = 0; kx < KS; ++kx)
                        acc += wp[ky * KS + kx] * row[kx];
                }
            }
        }
        __syncthreads();
    }

    if (ox < out_w && oy < out_h) {
        float v = acc + bias[f];
        out[((size_t)f * out_h + oy) * out_w + ox] = do_leaky ? leaky_act(v) : v;
    }
}

// ---------------------------------------------------------------- maxpool 2x2 / stride 2
__global__ void maxpool_kernel(const float* __restrict__ in, int c, int in_h, int in_w,
                               int size, int stride,
                               float* __restrict__ out, int out_h, int out_w)
{
    const int ox = blockIdx.x * blockDim.x + threadIdx.x;
    const int oy = blockIdx.y * blockDim.y + threadIdx.y;
    const int k  = blockIdx.z;
    if (ox >= out_w || oy >= out_h) return;

    float m = -INFINITY;
    for (int ky = 0; ky < size; ++ky)
        for (int kx = 0; kx < size; ++kx) {
            const int iy = oy * stride + ky, ix = ox * stride + kx;
            if (iy < in_h && ix < in_w) {
                float v = in[((size_t)k * in_h + iy) * in_w + ix];
                if (v > m) m = v;
            }
        }
    out[((size_t)k * out_h + oy) * out_w + ox] = m;
}

// ---------------------------------------------------------------- upsample（最近邻）
__global__ void upsample_kernel(const float* __restrict__ in, int c, int in_h, int in_w,
                                int stride, float* __restrict__ out)
{
    const int ow = in_w * stride, oh = in_h * stride;
    const int ox = blockIdx.x * blockDim.x + threadIdx.x;
    const int oy = blockIdx.y * blockDim.y + threadIdx.y;
    const int k  = blockIdx.z;
    if (ox >= ow || oy >= oh) return;
    out[((size_t)k * oh + oy) * ow + ox] =
        in[((size_t)k * in_h + (oy / stride)) * in_w + (ox / stride)];
}

// ---------------------------------------------------------------- Backend
class CudaBackend : public Backend {
public:
    explicit CudaBackend(const Network& net) : net_(net) {}
    ~CudaBackend() override {
        for (int i = 0; i < NUM_LAYERS; ++i) {
            if (d_buf_[i]) cudaFree(d_buf_[i]);
            if (d_w_[i])   cudaFree(d_w_[i]);
            if (d_b_[i])   cudaFree(d_b_[i]);
        }
        if (d_in_) cudaFree(d_in_);
        for (int i = 0; i < 2; ++i) if (h_out_[i]) cudaFreeHost(h_out_[i]);
    }
    const char* name() const override { return "cuda"; }

    bool init(int device, std::string* err) {
        CUDA_CHECK(cudaSetDevice(device), err);
        CUDA_CHECK(cudaMalloc(&d_in_, sizeof(float) * NET_C * NET_H * NET_W), err);
        for (int i = 0; i < NUM_LAYERS; ++i) {
            CUDA_CHECK(cudaMalloc(&d_buf_[i], sizeof(float) * net_.shape[i].out_elems), err);
            if (LAYERS[i].type != L_CONV) continue;
            const ConvWeights& cw = net_.conv[i];
            CUDA_CHECK(cudaMalloc(&d_w_[i], sizeof(float) * cw.weights.size()), err);
            CUDA_CHECK(cudaMalloc(&d_b_[i], sizeof(float) * cw.biases.size()), err);
            CUDA_CHECK(cudaMemcpy(d_w_[i], cw.weights.data(),
                                  sizeof(float) * cw.weights.size(), cudaMemcpyHostToDevice), err);
            CUDA_CHECK(cudaMemcpy(d_b_[i], cw.biases.data(),
                                  sizeof(float) * cw.biases.size(), cudaMemcpyHostToDevice), err);
        }
        for (int i = 0; i < 2; ++i) {
            const size_t n = net_.shape[net_.yolo_layers[i]].out_elems;
            CUDA_CHECK(cudaMallocHost(&h_out_[i], sizeof(float) * n), err);
        }
        return true;
    }

    bool forward(const float* input, const float* out[2], std::string* err) override {
        CUDA_CHECK(cudaMemcpy(d_in_, input, sizeof(float) * NET_C * NET_H * NET_W,
                              cudaMemcpyHostToDevice), err);

        for (int i = 0; i < NUM_LAYERS; ++i) {
            const LayerDef& d = LAYERS[i];
            const LayerShape& s = net_.shape[i];
            const float* in = (i == 0) ? d_in_ : d_buf_[i - 1];
            float* o = d_buf_[i];

            switch (d.type) {
            case L_CONV:
                if (!launch_conv(in, s, d, d_w_[i], d_b_[i], o, err)) return false;
                break;
            case L_MAXPOOL: {
                dim3 blk(16, 16), grd((s.out_w + 15) / 16, (s.out_h + 15) / 16, s.in_c);
                maxpool_kernel<<<grd, blk>>>(in, s.in_c, s.in_h, s.in_w,
                                             d.pool_size, d.pool_stride, o, s.out_h, s.out_w);
                break;
            }
            case L_UPSAMPLE: {
                dim3 blk(16, 16), grd((s.out_w + 15) / 16, (s.out_h + 15) / 16, s.in_c);
                upsample_kernel<<<grd, blk>>>(in, s.in_c, s.in_h, s.in_w, d.up_stride, o);
                break;
            }
            case L_ROUTE: {
                size_t off = 0;
                for (int t = 0; t < 2; ++t) {
                    int src = d.route_from[t];
                    if (src < 0) continue;
                    size_t part = net_.shape[src].out_elems / d.route_groups;
                    CUDA_CHECK(cudaMemcpyAsync(o + off,
                                               d_buf_[src] + part * d.route_group_id,
                                               part * sizeof(float),
                                               cudaMemcpyDeviceToDevice), err);
                    off += part;
                }
                break;
            }
            case L_YOLO:
                CUDA_CHECK(cudaMemcpyAsync(o, in, s.out_elems * sizeof(float),
                                           cudaMemcpyDeviceToDevice), err);
                break;
            }
        }

        for (int i = 0; i < 2; ++i) {
            const int li = net_.yolo_layers[i];
            CUDA_CHECK(cudaMemcpy(h_out_[i], d_buf_[li],
                                  sizeof(float) * net_.shape[li].out_elems,
                                  cudaMemcpyDeviceToHost), err);
            out[i] = h_out_[i];
        }
        CUDA_CHECK(cudaGetLastError(), err);
        return true;
    }

private:
    bool launch_conv(const float* in, const LayerShape& s, const LayerDef& d,
                     const float* w, const float* b, float* o, std::string* err)
    {
        const int pad = d.pad ? d.ksize / 2 : 0;
        const int leak = (d.act == ACT_LEAKY) ? 1 : 0;
        const int TW = 32, TH = 8;
        dim3 blk(TW, TH);
        dim3 grd((s.out_w + TW - 1) / TW, (s.out_h + TH - 1) / TH, s.out_c);

        if (d.ksize == 3 && d.stride == 1)
            conv_kernel<3, 1, TW, TH, 4><<<grd, blk>>>(in, s.in_c, s.in_h, s.in_w, w, b,
                                                       o, s.out_h, s.out_w, pad, leak);
        else if (d.ksize == 3 && d.stride == 2)
            conv_kernel<3, 2, TW, TH, 4><<<grd, blk>>>(in, s.in_c, s.in_h, s.in_w, w, b,
                                                       o, s.out_h, s.out_w, pad, leak);
        else if (d.ksize == 1 && d.stride == 1)
            conv_kernel<1, 1, TW, TH, 16><<<grd, blk>>>(in, s.in_c, s.in_h, s.in_w, w, b,
                                                        o, s.out_h, s.out_w, 0, leak);
        else {
            if (err) *err = "不支持的卷积参数组合 (ksize/stride)";
            return false;
        }
        CUDA_CHECK(cudaGetLastError(), err);
        return true;
    }

    const Network& net_;
    float* d_buf_[NUM_LAYERS] = {};
    float* d_w_[NUM_LAYERS] = {};
    float* d_b_[NUM_LAYERS] = {};
    float* d_in_ = nullptr;
    float* h_out_[2] = {};
};

Backend* make_cuda_backend(const Network& net, int device, std::string* err) {
    CudaBackend* b = new CudaBackend(net);
    if (!b->init(device, err)) { delete b; return nullptr; }
    return b;
}

} // namespace yolo
