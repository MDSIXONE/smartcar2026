// darknet .weights 解析。加载时直接把 BN 折叠进 weight/bias，推理路径就不用再算 BN 了。
//
// 文件布局（AlexeyAB darknet, src/parser.c load_weights_upto）:
//   int32 major, int32 minor, int32 revision
//   if (major*10 + minor >= 2)  uint64 seen      // 否则 uint32
//   然后按层序，每个 convolutional 层:
//       float biases[n]
//       if batch_normalize: float scales[n], rolling_mean[n], rolling_variance[n]
//       float weights[n * (c/groups) * ksize * ksize]     // 布局 [n][c][ky][kx]
//   非卷积层（route/maxpool/upsample/yolo）不占字节。
#include "yolo.h"
#include <cstdio>
#include <cmath>
#include <cstring>

namespace yolo {

static bool rd(FILE* f, void* p, size_t bytes, const char* what, std::string* err) {
    if (fread(p, 1, bytes, f) != bytes) {
        if (err) *err = std::string("权重文件提前结束，读取 ") + what + " 失败";
        return false;
    }
    return true;
}

bool load_weights(Network& net, const char* path, std::string* err) {
    FILE* f = fopen(path, "rb");
    if (!f) { if (err) *err = std::string("打不开权重文件: ") + path; return false; }

    int32_t major = 0, minor = 0, revision = 0;
    if (!rd(f, &major, 4, "major", err) ||
        !rd(f, &minor, 4, "minor", err) ||
        !rd(f, &revision, 4, "revision", err)) { fclose(f); return false; }

    if (major * 10 + minor >= 2) {
        uint64_t seen = 0;
        if (!rd(f, &seen, 8, "seen(u64)", err)) { fclose(f); return false; }
    } else {
        uint32_t seen = 0;
        if (!rd(f, &seen, 4, "seen(u32)", err)) { fclose(f); return false; }
    }

    for (int i = 0; i < NUM_LAYERS; ++i) {
        const LayerDef& d = LAYERS[i];
        if (d.type != L_CONV) continue;
        const LayerShape& s = net.shape[i];

        const int n  = d.filters;
        const size_t fsz = (size_t)s.in_c * d.ksize * d.ksize;   // 单个卷积核元素数
        ConvWeights& cw = net.conv[i];
        cw.biases.assign(n, 0.f);
        cw.weights.assign((size_t)n * fsz, 0.f);

        if (!rd(f, cw.biases.data(), sizeof(float) * n, "biases", err)) { fclose(f); return false; }

        std::vector<float> scales, mean, var;
        if (d.batch_norm) {
            scales.resize(n); mean.resize(n); var.resize(n);
            if (!rd(f, scales.data(), sizeof(float) * n, "scales", err) ||
                !rd(f, mean.data(),   sizeof(float) * n, "rolling_mean", err) ||
                !rd(f, var.data(),    sizeof(float) * n, "rolling_variance", err)) { fclose(f); return false; }
        }
        if (!rd(f, cw.weights.data(), sizeof(float) * cw.weights.size(), "weights", err)) { fclose(f); return false; }

        // --- BN 折叠 ---
        // darknet 前向: y = leaky( scale * (conv(x) - mean)/sqrt(var + 1e-5) + bias )
        // 折叠后:       y = leaky( conv'(x) + bias' )
        //   w' = w * scale / sqrt(var + 1e-5)
        //   b' = bias - mean * scale / sqrt(var + 1e-5)
        // 与 darknet 自己的 fuse_conv_batchnorm() 完全一致（src/network.c）。
        if (d.batch_norm) {
            for (int fi = 0; fi < n; ++fi) {
                const double denom = std::sqrt((double)var[fi] + (double)BN_EPS);
                const double k = (double)scales[fi] / denom;
                cw.biases[fi] = (float)((double)cw.biases[fi] - (double)mean[fi] * k);
                float* w = cw.weights.data() + (size_t)fi * fsz;
                for (size_t j = 0; j < fsz; ++j) w[j] = (float)((double)w[j] * k);
            }
        }
    }

    // 检查是否读完了整个文件——多出的字节说明结构对不上
    long cur = ftell(f);
    fseek(f, 0, SEEK_END);
    long end = ftell(f);
    fclose(f);
    if (cur != end) {
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "权重文件大小对不上：已读 %ld 字节，文件共 %ld 字节，剩余 %ld。"
                 "多半是 cfg 结构和权重不匹配。", cur, end, end - cur);
        if (err) *err = buf;
        return false;
    }
    return true;
}

} // namespace yolo
