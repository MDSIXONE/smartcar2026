/* core_ops.h — 逐元素算子核心 (CPU/GPU 共用同一份数学)
 * ====================================================================
 * 每个函数计算"第 i 个输出元素", CUDA kernel 与 CPU for 循环都调它,
 * 保证两个后端一致。只用 CUDA 10.0 / sm_53 (Maxwell) 支持的朴素语法:
 * 无 shuffle / 无 tensor core / 无 cooperative groups。
 *
 * 精度约定(见 ts_half.h):
 *   存储 ts_h  —— 默认 fp16, make FP32=1 退回 float
 *   累加 float —— **永远**是 float, 不随存储精度变
 * 读的时候 TS_H2F 转出来算, 写回去 TS_F2H。fp32 编译时这两个宏是恒等,
 * 生成的代码和改造前一模一样, 所以 fp32 那条路没有任何性能/精度损失。
 *
 * 为什么这样切: 这些 kernel 是一线程一输出、每次都从 global memory 拉
 * 权重和特征图, 瓶颈在访存不在算术。存 fp16 直接把访存量砍半, 而算术
 * 保持 fp32 就不用担心长累加掉精度 —— 收益拿到了, 风险没引进来。
 *
 * 激活: 0=linear 1=ReLU6 2=ReLU
 */
#ifndef TS_CORE_OPS_H
#define TS_CORE_OPS_H

#include "ts_half.h"

#ifdef __CUDACC__
#define TS_FN static __device__ __forceinline__
#else
#define TS_FN static inline
#endif

TS_FN float ts_act(float v, int act)
{
    if (act == 1) { v = v < 0.f ? 0.f : v; return v > 6.f ? 6.f : v; }
    if (act == 2) { return v < 0.f ? 0.f : v; }
    return v;
}

/* uint8 BGR HWC (H*W*3) -> 归一化 CHW (3*H*W), RGB 顺序 */
TS_FN void ts_preprocess_e(int i, const unsigned char *bgr, ts_h *dst,
                           const ts_h *mean, const ts_h *std_, int H, int W)
{
    int hw = H * W;
    int c = i / hw, p = i - c * hw;
    float v = (float)bgr[p * 3 + (2 - c)] / 255.f;   /* BGR -> RGB */
    dst[i] = TS_F2H((v - TS_H2F(mean[c])) / TS_H2F(std_[c]));
}

/* 通用 3x3 卷积 (stem 两层用, stride s, pad 1) */
TS_FN void ts_conv3x3_e(int i, const ts_h *src, ts_h *dst,
                        const ts_h *w, const ts_h *b,
                        int Cout, int Cin, int Hs, int Ws,
                        int Hd, int Wd, int s, int act)
{
    int hw = Hd * Wd;
    int co = i / hw, r = i - co * hw;
    int yo = r / Wd, xo = r - yo * Wd;
    float acc = TS_H2F(b[co]);                      /* 累加器: float */
    int ci, ky, kx;
    for (ci = 0; ci < Cin; ci++)
        for (ky = 0; ky < 3; ky++) {
            int yi = yo * s - 1 + ky;
            if (yi < 0 || yi >= Hs) continue;
            for (kx = 0; kx < 3; kx++) {
                int xi = xo * s - 1 + kx;
                if (xi < 0 || xi >= Ws) continue;
                acc += TS_H2F(src[(ci * Hs + yi) * Ws + xi]) *
                       TS_H2F(w[((co * Cin + ci) * 3 + ky) * 3 + kx]);
            }
        }
    dst[i] = TS_F2H(ts_act(acc, act));
}

/* depthwise 3x3, pad 1, stride s */
TS_FN void ts_dw3x3_e(int i, const ts_h *src, ts_h *dst,
                      const ts_h *w, const ts_h *b,
                      int C, int Hs, int Ws, int Hd, int Wd, int s, int act)
{
    int hw = Hd * Wd;
    int c = i / hw, r = i - c * hw;
    int yo = r / Wd, xo = r - yo * Wd;
    float acc = TS_H2F(b[c]);
    int ky, kx;
    for (ky = 0; ky < 3; ky++) {
        int yi = yo * s - 1 + ky;
        if (yi < 0 || yi >= Hs) continue;
        for (kx = 0; kx < 3; kx++) {
            int xi = xo * s - 1 + kx;
            if (xi < 0 || xi >= Ws) continue;
            acc += TS_H2F(src[(c * Hs + yi) * Ws + xi]) *
                   TS_H2F(w[(c * 3 + ky) * 3 + kx]);
        }
    }
    dst[i] = TS_F2H(ts_act(acc, act));
}

/* 1x1 卷积 */
TS_FN void ts_conv1x1_e(int i, const ts_h *src, ts_h *dst,
                        const ts_h *w, const ts_h *b,
                        int Cout, int Cin, int HW, int act)
{
    int co = i / HW, p = i - co * HW;
    float acc = TS_H2F(b[co]);
    int ci;
    const ts_h *wr = w + co * Cin;
    for (ci = 0; ci < Cin; ci++)
        acc += TS_H2F(src[ci * HW + p]) * TS_H2F(wr[ci]);
    dst[i] = TS_F2H(ts_act(acc, act));
}

/* dst = relu(dst + src) (解码跳连) */
TS_FN void ts_add_relu_e(int i, ts_h *dst, const ts_h *src)
{
    float v = TS_H2F(dst[i]) + TS_H2F(src[i]);
    dst[i] = TS_F2H(v < 0.f ? 0.f : v);
}

/* 双线性上采样, align_corners=False (与 PyTorch 完全一致):
 * src_x = (dst_x + 0.5) * (Ws/Wd) - 0.5, 负数截 0, 右边界用 x1p=0 处理。
 * 插值本身在 float 里做, 只有读写是 ts_h。 */
TS_FN void ts_upsample_e(int i, const ts_h *src, ts_h *dst,
                         int C, int Hs, int Ws, int Hd, int Wd)
{
    int hw = Hd * Wd;
    int c = i / hw, r = i - c * hw;
    int yo = r / Wd, xo = r - yo * Wd;
    float sy = ((float)yo + 0.5f) * ((float)Hs / (float)Hd) - 0.5f;
    float sx = ((float)xo + 0.5f) * ((float)Ws / (float)Wd) - 0.5f;
    int y0, x0, y1p, x1p;
    float ly, lx, v00, v01, v10, v11;
    const ts_h *sp;
    if (sy < 0.f) sy = 0.f;
    if (sx < 0.f) sx = 0.f;
    y0 = (int)sy; x0 = (int)sx;
    y1p = (y0 < Hs - 1) ? 1 : 0;
    x1p = (x0 < Ws - 1) ? 1 : 0;
    ly = sy - (float)y0; lx = sx - (float)x0;
    sp = src + c * Hs * Ws;
    v00 = TS_H2F(sp[y0 * Ws + x0]);
    v01 = TS_H2F(sp[y0 * Ws + x0 + x1p]);
    v10 = TS_H2F(sp[(y0 + y1p) * Ws + x0]);
    v11 = TS_H2F(sp[(y0 + y1p) * Ws + x0 + x1p]);
    dst[i] = TS_F2H((1.f - ly) * ((1.f - lx) * v00 + lx * v01) +
                    ly * ((1.f - lx) * v10 + lx * v11));
}

/* 上采样+argmax **融合**版: 对每个输出像素直接在 float 里做 4 个通道的
 * 双线性插值并取最大, 不再把 245K 个 logits 写回显存再读一遍 ——
 * 省 ~1MB 往返流量 + 一次 kernel 启动。只有 ts_infer_logits 才走
 * 老的"先上采样存下来再 argmax"路径(要交 logits 给外面)。
 * fp32 下与老路径逐位一致; fp16 下少了一次"存 half 的舍入", 反而更准。 */
TS_FN void ts_up_argmax_e(int p, const ts_h *src, unsigned char *mask,
                          int C, int Hs, int Ws, int Hd, int Wd)
{
    int yo = p / Wd, xo = p - yo * Wd;
    float sy = ((float)yo + 0.5f) * ((float)Hs / (float)Hd) - 0.5f;
    float sx = ((float)xo + 0.5f) * ((float)Ws / (float)Wd) - 0.5f;
    int y0, x0, y1p, x1p, c, best = 0;
    float ly, lx, bv = 0.f;
    if (sy < 0.f) sy = 0.f;
    if (sx < 0.f) sx = 0.f;
    y0 = (int)sy; x0 = (int)sx;
    y1p = (y0 < Hs - 1) ? Ws : 0;
    x1p = (x0 < Ws - 1) ? 1 : 0;
    ly = sy - (float)y0; lx = sx - (float)x0;
    for (c = 0; c < C; c++) {
        const ts_h *sp = src + (c * Hs + y0) * Ws + x0;
        float v = (1.f - ly) * ((1.f - lx) * TS_H2F(sp[0]) +
                                lx * TS_H2F(sp[x1p])) +
                  ly * ((1.f - lx) * TS_H2F(sp[y1p]) +
                        lx * TS_H2F(sp[y1p + x1p]));
        if (c == 0 || v > bv) { bv = v; best = c; }
    }
    mask[p] = (unsigned char)best;
}

/* 4 类 argmax -> uint8 掩码 (每像素)。
 * 比较在 float 里做; fp16 下相邻两类分数相等的概率变高, 但**并列时取
 * 先出现的那一类**这个规则和 PyTorch 的 argmax 一致, 不会引入偏差。 */
TS_FN void ts_argmax_e(int p, const ts_h *logits, unsigned char *mask,
                       int C, int HW)
{
    int c, best = 0;
    float bv = TS_H2F(logits[p]);
    for (c = 1; c < C; c++) {
        float v = TS_H2F(logits[c * HW + p]);
        if (v > bv) { bv = v; best = c; }
    }
    mask[p] = (unsigned char)best;
}

/* ---------------- half2 打包版卷积 (见 ts_half.h 里的说明) ----------------
 * 一个线程/迭代算**同一通道的两个相邻输出像素**, 乘加走 HFMA2。
 * 前提(本网络的图恒满足, ts_init 会跑一帧校验): HW / Wd 都是偶数。
 * dw3x3 不做打包: 全网 dw 只占 ~3% MACs, 不值得多一份边界代码。 */
#ifdef TS_H2

/* 1x1 卷积: i ∈ [0, Cout*HW/4), 算同一通道 4 个相邻输出 (p..p+3)。
 * 两条独立的 HFMA2 累加链(hA,hB)互不等待, 权重每次加载喂 4 个输出。
 * 每个输出各自的分块累加序列与 2 路版完全一致 -> 数值逐位不变。
 * 前提: HW 是 4 的倍数(本图: 15360/3840/960/240, 恒成立)。 */
TS_FN void ts_conv1x1_p4(int i, const ts_h *src, ts_h *dst,
                         const ts_h *w, const ts_h *b,
                         int Cout, int Cin, int HW, int act)
{
    int hw4 = HW >> 2;
    int co = i / hw4, p = (i - co * hw4) * 4;
    float bb = TS_H2F(b[co]);
    float a0 = bb, a1 = bb, a2 = bb, a3 = bb;
    const ts_h *wr = w + co * Cin;
    const ts_h *sp = src + p;
    int ci = 0;
    while (ci < Cin) {
        int e = ci + TS_BLK;
        ts_h2 hA = TS_H2_ZERO(), hB = TS_H2_ZERO();
        if (e > Cin) e = Cin;
        for (; ci < e; ci++) {
            ts_h2 wb = TS_H2_BCAST(wr[ci]);
            const ts_h *q = sp + ci * HW;
            hA = TS_HFMA2(TS_H2_LD(q), wb, hA);
            hB = TS_HFMA2(TS_H2_LD(q + 2), wb, hB);
        }
        a0 += TS_H2_LOF(hA); a1 += TS_H2_HIF(hA);
        a2 += TS_H2_LOF(hB); a3 += TS_H2_HIF(hB);
    }
    TS_H2_ST(dst + co * HW + p,
             TS_H2_PACK(TS_F2H(ts_act(a0, act)), TS_F2H(ts_act(a1, act))));
    TS_H2_ST(dst + co * HW + p + 2,
             TS_H2_PACK(TS_F2H(ts_act(a2, act)), TS_F2H(ts_act(a3, act))));
}

/* 3x3 卷积: i ∈ [0, Cout*Hd*Wd/4), 算 (co,yo,xo..xo+3) 四个输出。
 * stride=2 时四个输出的输入横向各错开 s 列 -> 标量取数拼 half2;
 * 越界 tap 拼 0(bias 在 float 里加)。每个输入通道 ≤9 项刷一次 float。
 * 前提: Wd 是 4 的倍数(stem: 160/80, 恒成立)。 */
TS_FN void ts_conv3x3_p4(int i, const ts_h *src, ts_h *dst,
                         const ts_h *w, const ts_h *b,
                         int Cout, int Cin, int Hs, int Ws,
                         int Hd, int Wd, int s, int act)
{
    int wd4 = Wd >> 2;
    int hw4 = Hd * wd4;
    int co = i / hw4, r = i - co * hw4;
    int yo = r / wd4, xo = (r - yo * wd4) * 4;
    float bb = TS_H2F(b[co]);
    float a0 = bb, a1 = bb, a2 = bb, a3 = bb;
    ts_h hz = TS_F2H(0.f);
    int ci, ky, kx;
    for (ci = 0; ci < Cin; ci++) {
        const ts_h *wr = w + (co * Cin + ci) * 9;
        ts_h2 hA = TS_H2_ZERO(), hB = TS_H2_ZERO();
        for (ky = 0; ky < 3; ky++) {
            int yi = yo * s - 1 + ky;
            const ts_h *row;
            if (yi < 0 || yi >= Hs) continue;
            row = src + (ci * Hs + yi) * Ws;
            for (kx = 0; kx < 3; kx++) {
                int x0 = xo * s - 1 + kx;
                int x1 = x0 + s, x2 = x1 + s, x3 = x2 + s;
                ts_h2 wb = TS_H2_BCAST(wr[ky * 3 + kx]);
                ts_h e0 = (x0 >= 0 && x0 < Ws) ? row[x0] : hz;
                ts_h e1 = (x1 >= 0 && x1 < Ws) ? row[x1] : hz;
                ts_h e2 = (x2 >= 0 && x2 < Ws) ? row[x2] : hz;
                ts_h e3 = (x3 >= 0 && x3 < Ws) ? row[x3] : hz;
                hA = TS_HFMA2(TS_H2_PACK(e0, e1), wb, hA);
                hB = TS_HFMA2(TS_H2_PACK(e2, e3), wb, hB);
            }
        }
        a0 += TS_H2_LOF(hA); a1 += TS_H2_HIF(hA);
        a2 += TS_H2_LOF(hB); a3 += TS_H2_HIF(hB);
    }
    TS_H2_ST(dst + (co * Hd + yo) * Wd + xo,
             TS_H2_PACK(TS_F2H(ts_act(a0, act)), TS_F2H(ts_act(a1, act))));
    TS_H2_ST(dst + (co * Hd + yo) * Wd + xo + 2,
             TS_H2_PACK(TS_F2H(ts_act(a2, act)), TS_F2H(ts_act(a3, act))));
}

#endif /* TS_H2 */

#endif /* TS_CORE_OPS_H */
