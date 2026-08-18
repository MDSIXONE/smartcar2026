/* trackseg_cuda.cu — CUDA 后端 (Jetson Nano / Maxwell sm_53 / CUDA 10.0)
 * small(TrackSegNet, Fast-SCNN 风格) 版
 * ====================================================================
 * 朴素 kernel: 每线程一个输出元素, 只用最基础的 CUDA C 语法,
 * 无 shared memory 技巧/无 warp shuffle/无半精度 —— Maxwell 全兼容。
 * 权重编译期固化(gen/weights_embed.c), 运行时零外部文件。
 *
 * 编译(见 Makefile):
 *   nvcc -O3 -gencode arch=compute_53,code=sm_53 -Xcompiler -fPIC \
 *        -shared src/trackseg_cuda.cu gen/weights_embed.c -o libtrackseg.so
 */
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "core_ops.h"
#include "trackseg.h"
#include "../gen/net_meta.h"

extern "C" {
extern const unsigned char ts_weights_bytes[];
extern const unsigned int ts_weights_len;
}

static char g_err[256] = "";
static int g_ready = 0;
static unsigned int ts_used_floats = 0;
static int ts_want_logits = 0;      /* 见 net_graph.inc 末段的分支 */

#define CK(call)                                                          \
    do {                                                                  \
        cudaError_t e_ = (call);                                          \
        if (e_ != cudaSuccess) {                                          \
            snprintf(g_err, sizeof(g_err), "%s:%d %s", __FILE__,          \
                     __LINE__, cudaGetErrorString(e_));                   \
            return -1;                                                    \
        }                                                                 \
    } while (0)

/* ---------------- kernels: 薄封装 core_ops ---------------- */
__global__ void k_pre(const unsigned char *bgr, ts_h *dst,
                      const ts_h *mean, const ts_h *std_, int H, int W,
                      int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_preprocess_e(i, bgr, dst, mean, std_, H, W);
}
__global__ void k_c3(const ts_h *__restrict__ s, ts_h *__restrict__ d,
                     const ts_h *__restrict__ w, const ts_h *__restrict__ b, int Co, int Ci, int Hs, int Ws, int Hd,
                     int Wd, int st, int act, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_conv3x3_e(i, s, d, w, b, Co, Ci, Hs, Ws, Hd, Wd, st, act);
}
__global__ void k_dw(const ts_h *__restrict__ s, ts_h *__restrict__ d,
                     const ts_h *__restrict__ w, const ts_h *__restrict__ b, int C, int Hs, int Ws, int Hd, int Wd,
                     int st, int act, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_dw3x3_e(i, s, d, w, b, C, Hs, Ws, Hd, Wd, st, act);
}
__global__ void k_c1(const ts_h *__restrict__ s, ts_h *__restrict__ d,
                     const ts_h *__restrict__ w, const ts_h *__restrict__ b, int Co, int Ci, int HW, int act, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_conv1x1_e(i, s, d, w, b, Co, Ci, HW, act);
}
__global__ void k_addrelu(ts_h *d, const ts_h *s, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_add_relu_e(i, d, s);
}
__global__ void k_up(const ts_h *__restrict__ s, ts_h *__restrict__ d,
                     int C, int Hs, int Ws,
                     int Hd, int Wd, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_upsample_e(i, s, d, C, Hs, Ws, Hd, Wd);
}
#ifdef TS_H2
__global__ void k_c3p(const ts_h *__restrict__ s, ts_h *__restrict__ d,
                      const ts_h *__restrict__ w, const ts_h *__restrict__ b, int Co, int Ci, int Hs, int Ws, int Hd,
                      int Wd, int st, int act, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_conv3x3_p4(i, s, d, w, b, Co, Ci, Hs, Ws, Hd, Wd, st, act);
}
__global__ void k_c1p(const ts_h *__restrict__ s, ts_h *__restrict__ d,
                      const ts_h *__restrict__ w, const ts_h *__restrict__ b, int Co, int Ci, int HW, int act, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_conv1x1_p4(i, s, d, w, b, Co, Ci, HW, act);
}
#endif
__global__ void k_uparg(const ts_h *__restrict__ s, unsigned char *m,
                        int C, int Hs, int Ws, int Hd, int Wd, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) ts_up_argmax_e(i, s, m, C, Hs, Ws, Hd, Wd);
}
__global__ void k_argmax(const ts_h *lg, unsigned char *m, int C, int HW)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < HW) ts_argmax_e(i, lg, m, C, HW);
}
/* 只在 ts_infer_logits 里用: 内部是 ts_h, 对外 ABI 保持 float 不变 */
__global__ void k_tofloat(const ts_h *s, float *d, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) d[i] = TS_H2F(s[i]);
}

#define NB(n) ((n) + 255) / 256, 256

/* ---------------- 显存缓冲 ---------------- */
static ts_h *w_all = 0;                  /* 权重 blob (device) */
static float *d_logits32 = 0;            /* 只给 ts_infer_logits 用 */
static unsigned char *d_bgr = 0, *B_MASK = 0;
static ts_h *B_IN, *B_A, *B_S, *B_D, *B_P, *B_Q, *B_U, *B_H, *B_OUT;

static int alloc_all(void)
{
    /* 嵌入的 blob 永远是 fp32(导出脚本不用改), 在 host 侧按编译精度转一遍
     * 再上传。fp16 时显存里的权重和特征图全部减半 —— 这些 kernel 是
     * 一线程一输出、反复从 global memory 拉数据, 访存量砍半就是主要收益。 */
    {
        const float *src = (const float *)(const void *)ts_weights_bytes;
        ts_h *tmp = (ts_h *)malloc((size_t)TS_TOTAL_FLOATS * sizeof(ts_h));
        unsigned int k;
        if (!tmp) { snprintf(g_err, sizeof(g_err), "host malloc failed"); return -1; }
        for (k = 0; k < TS_TOTAL_FLOATS; k++) tmp[k] = TS_F2H(src[k]);
        if (cudaMalloc(&w_all, (size_t)TS_TOTAL_FLOATS * sizeof(ts_h))
                != cudaSuccess) { free(tmp); return -1; }
        CK(cudaMemcpy(w_all, tmp, (size_t)TS_TOTAL_FLOATS * sizeof(ts_h),
                      cudaMemcpyHostToDevice));
        free(tmp);
    }
    CK(cudaMalloc(&d_bgr, 192 * 320 * 3));
    CK(cudaMalloc(&B_MASK, 192 * 320));
#define AF(p, n) CK(cudaMalloc(&p, (size_t)(n) * sizeof(ts_h)))
    AF(B_IN, 3 * 192 * 320);
    AF(B_A, 32 * 96 * 160);      /* stem.0 输出, 全网最大的中间张量 */
    AF(B_S, 48 * 48 * 80);       /* stem 输出, 要留到 fuse_shallow */
    AF(B_D, 48 * 48 * 80);       /* dw 输出最大: head.0 48ch@48x80 */
    AF(B_P, 48 * 48 * 80);       /* 乒 */
    AF(B_Q, 48 * 48 * 80);       /* 乓 */
    AF(B_U, 48 * 48 * 80);       /* 上采样后的深层特征 */
    AF(B_H, 4 * 48 * 80);
    AF(B_OUT, 4 * 192 * 320);
    CK(cudaMalloc(&d_logits32, 4 * 192 * 320 * sizeof(float)));
#undef AF
    return 0;
}

/* ---------------- 前向图 (与 CPU 后端共用同一份 inc) ---------------- */
#define OP_PRE(dst, src, m, s, H, W) \
    k_pre<<<NB(3 * (H) * (W))>>>(src, dst, m, s, H, W, 3 * (H) * (W))
#ifdef TS_H2                       /* half2 打包: 线程数减半, 乘加走 HFMA2 */
#define OP_C3(d, s, w, b, Co, Ci, Hs, Ws, Hd, Wd, st, act)                 \
    k_c3p<<<NB((Co) * (Hd) * (Wd) / 4)>>>(s, d, w, b, Co, Ci, Hs, Ws, Hd,  \
                                          Wd, st, act,                     \
                                          (Co) * (Hd) * (Wd) / 4)
#else
#define OP_C3(d, s, w, b, Co, Ci, Hs, Ws, Hd, Wd, st, act)                 \
    k_c3<<<NB((Co) * (Hd) * (Wd))>>>(s, d, w, b, Co, Ci, Hs, Ws, Hd, Wd,   \
                                     st, act, (Co) * (Hd) * (Wd))
#endif
#define OP_DW(d, s, w, b, C, Hs, Ws, Hd, Wd, st, act)                      \
    k_dw<<<NB((C) * (Hd) * (Wd))>>>(s, d, w, b, C, Hs, Ws, Hd, Wd, st,     \
                                    act, (C) * (Hd) * (Wd))
#ifdef TS_H2
#define OP_C1(d, s, w, b, Co, Ci, HW, act)                                 \
    k_c1p<<<NB((Co) * (HW) / 4)>>>(s, d, w, b, Co, Ci, HW, act,            \
                                   (Co) * (HW) / 4)
#else
#define OP_C1(d, s, w, b, Co, Ci, HW, act) \
    k_c1<<<NB((Co) * (HW))>>>(s, d, w, b, Co, Ci, HW, act, (Co) * (HW))
#endif
#define OP_ADDRELU(d, s, n) k_addrelu<<<NB(n)>>>(d, s, n)
#define OP_UP(d, s, C, Hs, Ws, Hd, Wd)                                     \
    k_up<<<NB((C) * (Hd) * (Wd))>>>(s, d, C, Hs, Ws, Hd, Wd,               \
                                    (C) * (Hd) * (Wd))
#define OP_ARGMAX(m, lg, C, HW) k_argmax<<<NB(HW)>>>(lg, m, C, HW)
#define OP_UPARG(m, s, C, Hs, Ws, Hd, Wd)                                  \
    k_uparg<<<NB((Hd) * (Wd))>>>(s, m, C, Hs, Ws, Hd, Wd, (Hd) * (Wd))

static void run_graph(const unsigned char *in_bgr)
{
    const ts_h *wp = w_all;
#include "net_graph.inc"
}

/* ---------------- C ABI ---------------- */
/* 启动横幅: 往 stderr 刷 TS_BANNER_TIMES 遍, 想漏看都难。
 * 为什么要这个: 换 .so 是"拷文件"这种最容易出错的操作 —— 编了 fp16 却
 * 拷了旧的 fp32、或者压根没拷成功, 从行为上几乎看不出来(掩码只差 0.002%)。
 * 所以把精度、权重数、**编译时刻**、以及运行时真实的设备型号一起打出来:
 * built 那个时间戳对不上, 就是没拷成功。
 * 用 stderr 不用 stdout: stderr 不缓冲, roslaunch 也照样收得到。 */
#ifndef TS_BANNER_TIMES
#define TS_BANNER_TIMES 10
#endif

static void ts_banner(void)
{
    char dev[320] = "unknown";  /* prop.name 本身就有 256 字节 */
    int d = 0, k;
    cudaDeviceProp prop;
    if (cudaGetDevice(&d) == cudaSuccess &&
            cudaGetDeviceProperties(&prop, d) == cudaSuccess) {
        snprintf(dev, sizeof(dev), "%s sm_%d%d", prop.name, prop.major,
                 prop.minor);
    }
    for (k = 0; k < TS_BANNER_TIMES; k++)
        fprintf(stderr,
                "[trackseg] ##### %s ##### small/cuda  dev=%s  weights=%u  "
                "built=%s %s\n",
                TS_PREC_NAME, dev, (unsigned)TS_TOTAL_FLOATS,
                __DATE__, __TIME__);
    fflush(stderr);
}

extern "C" int ts_init(void)
{
    if (g_ready) return 0;
    if (ts_weights_len != TS_TOTAL_FLOATS * sizeof(float)) {
        snprintf(g_err, sizeof(g_err), "weights_embed size mismatch");
        return -1;
    }
    if (alloc_all()) return -1;
    /* 预热一帧并校验权重游标 */
    CK(cudaMemset(d_bgr, 127, 192 * 320 * 3));
    run_graph(d_bgr);
    CK(cudaDeviceSynchronize());
    CK(cudaGetLastError());
    if (ts_used_floats != TS_TOTAL_FLOATS) {
        snprintf(g_err, sizeof(g_err),
                 "graph/export 权重顺序不一致: used %u expect %u",
                 ts_used_floats, (unsigned)TS_TOTAL_FLOATS);
        return -2;
    }
    g_ready = 1;
    ts_banner();
    return 0;
}

extern "C" int ts_infer(const unsigned char *bgr, unsigned char *mask)
{
    if (!g_ready) { snprintf(g_err, sizeof(g_err), "call ts_init first"); return -3; }
    ts_want_logits = 0;
    CK(cudaMemcpy(d_bgr, bgr, 192 * 320 * 3, cudaMemcpyHostToDevice));
    run_graph(d_bgr);
    CK(cudaMemcpy(mask, B_MASK, 192 * 320, cudaMemcpyDeviceToHost));
    CK(cudaGetLastError());
    return 0;
}

extern "C" int ts_infer_logits(const unsigned char *bgr, float *logits)
{
    const int n = 4 * 192 * 320;
    if (!g_ready) { snprintf(g_err, sizeof(g_err), "call ts_init first"); return -3; }
    ts_want_logits = 1;
    CK(cudaMemcpy(d_bgr, bgr, 192 * 320 * 3, cudaMemcpyHostToDevice));
    run_graph(d_bgr);
    k_tofloat<<<NB(n)>>>(B_OUT, d_logits32, n);   /* 对外仍然给 float */
    CK(cudaMemcpy(logits, d_logits32, n * sizeof(float),
                  cudaMemcpyDeviceToHost));
    CK(cudaGetLastError());
    return 0;
}

extern "C" const char *ts_error(void) { return g_err; }
extern "C" const char *ts_backend(void) { return "cuda/" TS_PREC_NAME; }

extern "C" void ts_destroy(void)
{
    if (!g_ready) return;
    cudaFree(w_all); cudaFree(d_bgr); cudaFree(B_MASK);
    cudaFree(d_logits32);
    ts_h *bufs[] = { B_IN, B_A, B_S, B_D, B_P, B_Q, B_U, B_H, B_OUT };
    for (unsigned i = 0; i < sizeof(bufs) / sizeof(bufs[0]); i++)
        cudaFree(bufs[i]);
    g_ready = 0;
}
