// CPU 参考实现。存在的意义有两个：
//   1. 没有 GPU 的机器上也能跑通、能验证；
//   2. 作为 CUDA kernel 的数值基准 —— 两条路径必须给出一致结果。
// 性能不是目标（640x352 下单张大约几百毫秒）。
#include "yolo.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace yolo {

static inline float leaky(float x) { return x > 0 ? x : 0.1f * x; }

static void conv_forward(const float* in, int in_c, int in_h, int in_w,
                         const ConvWeights& w, int filters, int ksize, int stride, int pad,
                         Activation act, float* out, int out_h, int out_w)
{
    const size_t fsz = (size_t)in_c * ksize * ksize;
    // 输出通道之间完全独立，直接并行；没有 OpenMP 时这行 pragma 被忽略
    #ifdef _OPENMP
    #pragma omp parallel for schedule(static)
    #endif
    for (int f = 0; f < filters; ++f) {
        const float* wf = w.weights.data() + (size_t)f * fsz;
        const float bias = w.biases[f];
        float* op = out + (size_t)f * out_h * out_w;
        for (int oy = 0; oy < out_h; ++oy)
            for (int ox = 0; ox < out_w; ++ox) {
                float acc = bias;
                for (int c = 0; c < in_c; ++c) {
                    const float* ip = in + (size_t)c * in_h * in_w;
                    const float* kp = wf + (size_t)c * ksize * ksize;
                    for (int ky = 0; ky < ksize; ++ky) {
                        const int iy = oy * stride - pad + ky;
                        if (iy < 0 || iy >= in_h) continue;
                        for (int kx = 0; kx < ksize; ++kx) {
                            const int ix = ox * stride - pad + kx;
                            if (ix < 0 || ix >= in_w) continue;
                            acc += kp[ky * ksize + kx] * ip[(size_t)iy * in_w + ix];
                        }
                    }
                }
                op[(size_t)oy * out_w + ox] = (act == ACT_LEAKY) ? leaky(acc) : acc;
            }
    }
}

static void maxpool_forward(const float* in, int c, int in_h, int in_w,
                            int size, int stride, float* out, int out_h, int out_w)
{
    // darknet: w_offset = -pad/2 = 0（pad = size-1，整数除后为 0）
    for (int k = 0; k < c; ++k) {
        const float* ip = in + (size_t)k * in_h * in_w;
        float* op = out + (size_t)k * out_h * out_w;
        for (int oy = 0; oy < out_h; ++oy)
            for (int ox = 0; ox < out_w; ++ox) {
                float m = -INFINITY;
                for (int ky = 0; ky < size; ++ky)
                    for (int kx = 0; kx < size; ++kx) {
                        int iy = oy * stride + ky;
                        int ix = ox * stride + kx;
                        bool valid = (iy >= 0 && iy < in_h && ix >= 0 && ix < in_w);
                        float v = valid ? ip[(size_t)iy * in_w + ix] : -INFINITY;
                        if (v > m) m = v;
                    }
                op[(size_t)oy * out_w + ox] = m;
            }
    }
}

static void upsample_forward(const float* in, int c, int in_h, int in_w, int stride, float* out) {
    const int ow = in_w * stride, oh = in_h * stride;
    for (int k = 0; k < c; ++k) {
        const float* ip = in + (size_t)k * in_h * in_w;
        float* op = out + (size_t)k * oh * ow;
        for (int j = 0; j < oh; ++j)
            for (int i = 0; i < ow; ++i)
                op[(size_t)j * ow + i] = ip[(size_t)(j / stride) * in_w + (i / stride)];
    }
}

class CpuBackend : public Backend {
public:
    explicit CpuBackend(const Network& net) : net_(net) {
        for (int i = 0; i < NUM_LAYERS; ++i) buf_[i].assign(net.shape[i].out_elems, 0.f);
    }
    const char* name() const override { return "cpu"; }

    bool forward(const float* input, const float* out[2], std::string* err) override {
        (void)err;
        // 调试：设 YT_DUMP=<prefix> 时把每层输出落盘，便于和 darknet 对拍
        const char* dump = getenv("YT_DUMP");
        for (int i = 0; i < NUM_LAYERS; ++i) {
            const LayerDef& d = LAYERS[i];
            const LayerShape& s = net_.shape[i];
            const float* in = (i == 0) ? input : buf_[i - 1].data();
            float* o = buf_[i].data();

            switch (d.type) {
            case L_CONV:
                conv_forward(in, s.in_c, s.in_h, s.in_w, net_.conv[i],
                             d.filters, d.ksize, d.stride, d.pad ? d.ksize / 2 : 0,
                             d.act, o, s.out_h, s.out_w);
                break;
            case L_MAXPOOL:
                maxpool_forward(in, s.in_c, s.in_h, s.in_w, d.pool_size, d.pool_stride,
                                o, s.out_h, s.out_w);
                break;
            case L_UPSAMPLE:
                upsample_forward(in, s.in_c, s.in_h, s.in_w, d.up_stride, o);
                break;
            case L_ROUTE: {
                // darknet forward_route_layer: 每路取 input_size/groups 个连续元素，
                // 起点偏移 = part_size * group_id，然后依次拼接
                size_t off = 0;
                for (int t = 0; t < 2; ++t) {
                    int src = d.route_from[t];
                    if (src < 0) continue;
                    size_t total = net_.shape[src].out_elems;
                    size_t part  = total / d.route_groups;
                    std::memcpy(o + off, buf_[src].data() + part * d.route_group_id,
                                part * sizeof(float));
                    off += part;
                }
                break;
            }
            case L_YOLO:
                // 本实现把 sigmoid/scale_x_y 全部放在后处理里做，这里只透传
                std::memcpy(o, in, s.out_elems * sizeof(float));
                break;
            }
            if (dump) {
                char path[512];
                snprintf(path, sizeof(path), "%s_%02d.bin", dump, i);
                FILE* fp = fopen(path, "wb");
                if (fp) { fwrite(o, sizeof(float), s.out_elems, fp); fclose(fp); }
            }
        }
        out[0] = buf_[net_.yolo_layers[0]].data();
        out[1] = buf_[net_.yolo_layers[1]].data();
        return true;
    }

private:
    const Network& net_;
    std::vector<float> buf_[NUM_LAYERS];
};

Backend* make_cpu_backend(const Network& net) { return new CpuBackend(net); }

} // namespace yolo
