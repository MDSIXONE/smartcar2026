/* trackseg_cpu.c — CPU 参考后端 (验证/无卡机器用), small 版
 * 与 CUDA 后端共用 core_ops.h + net_graph.inc, 数学逐位一致。
 * 故意不开 OpenMP: pip 版 cv2 自带的 OpenMP 与 libgomp 同进程会段错误,
 * 验证用单线程足够(见 Makefile 注释)。
 *   gcc -O2 -fPIC -shared src/trackseg_cpu.c gen/weights_embed.c \
 *       -o libtrackseg_cpu.so -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "core_ops.h"
#include "trackseg.h"
#include "../gen/net_meta.h"

extern const unsigned char ts_weights_bytes[];
extern const unsigned int ts_weights_len;

static char g_err[256] = "";
static int g_ready = 0;
static unsigned int ts_used_floats = 0;
static int ts_want_logits = 0;      /* 见 net_graph.inc 末段的分支 */

static ts_h *w_all = 0;
static unsigned char *B_MASK = 0;
static ts_h *B_IN, *B_A, *B_S, *B_D, *B_P, *B_Q, *B_U, *B_H, *B_OUT;

#define LOOP(n) for (i = 0; i < (n); i++)

#define OP_PRE(dst, src, m, s, H, W)                                      \
    do { int i, n_ = 3 * (H) * (W);                                       \
         LOOP(n_) ts_preprocess_e(i, src, dst, m, s, H, W); } while (0)
#ifdef TS_H2                       /* half2 打包: 一次算两个相邻输出 */
#define OP_C3(d, s, w, b, Co, Ci, Hs, Ws, Hd, Wd, st, act)                \
    do { int i, n_ = (Co) * (Hd) * (Wd) / 4;                              \
         LOOP(n_) ts_conv3x3_p4(i, s, d, w, b, Co, Ci, Hs, Ws, Hd, Wd,    \
                                st, act); } while (0)
#else
#define OP_C3(d, s, w, b, Co, Ci, Hs, Ws, Hd, Wd, st, act)                \
    do { int i, n_ = (Co) * (Hd) * (Wd);                                  \
         LOOP(n_) ts_conv3x3_e(i, s, d, w, b, Co, Ci, Hs, Ws, Hd, Wd,     \
                               st, act); } while (0)
#endif
#define OP_DW(d, s, w, b, C, Hs, Ws, Hd, Wd, st, act)                     \
    do { int i, n_ = (C) * (Hd) * (Wd);                                   \
         LOOP(n_) ts_dw3x3_e(i, s, d, w, b, C, Hs, Ws, Hd, Wd, st, act);  \
    } while (0)
#ifdef TS_H2
#define OP_C1(d, s, w, b, Co, Ci, HW, act)                                \
    do { int i, n_ = (Co) * (HW) / 4;                                     \
         LOOP(n_) ts_conv1x1_p4(i, s, d, w, b, Co, Ci, HW, act); } while (0)
#else
#define OP_C1(d, s, w, b, Co, Ci, HW, act)                                \
    do { int i, n_ = (Co) * (HW);                                         \
         LOOP(n_) ts_conv1x1_e(i, s, d, w, b, Co, Ci, HW, act); } while (0)
#endif
#define OP_ADDRELU(d, s, n)                                               \
    do { int i; LOOP(n) ts_add_relu_e(i, d, s); } while (0)
#define OP_UP(d, s, C, Hs, Ws, Hd, Wd)                                    \
    do { int i, n_ = (C) * (Hd) * (Wd);                                   \
         LOOP(n_) ts_upsample_e(i, s, d, C, Hs, Ws, Hd, Wd); } while (0)
#define OP_ARGMAX(m, lg, C, HW)                                           \
    do { int i; LOOP(HW) ts_argmax_e(i, lg, m, C, HW); } while (0)
#define OP_UPARG(m, s, C, Hs, Ws, Hd, Wd)                                 \
    do { int i, n_ = (Hd) * (Wd);                                         \
         LOOP(n_) ts_up_argmax_e(i, s, m, C, Hs, Ws, Hd, Wd); } while (0)

static void run_graph(const unsigned char *in_bgr)
{
    const ts_h *wp = w_all;
#include "net_graph.inc"
}

/* 见 CUDA 后端里的说明: 刷 10 遍, 免得拷错 .so 还不知道 */
#ifndef TS_BANNER_TIMES
#define TS_BANNER_TIMES 10
#endif

static void ts_banner(void)
{
    int k;
    for (k = 0; k < TS_BANNER_TIMES; k++)
        fprintf(stderr,
                "[trackseg] ##### %s ##### small/cpu  weights=%u  "
                "built=%s %s\n",
                TS_PREC_NAME, (unsigned)TS_TOTAL_FLOATS, __DATE__, __TIME__);
    fflush(stderr);
}

int ts_init(void)
{
    unsigned char *dummy;
    if (g_ready) return 0;
    if (ts_weights_len != TS_TOTAL_FLOATS * sizeof(float)) {
        snprintf(g_err, sizeof(g_err), "weights_embed size mismatch");
        return -1;
    }
    /* 嵌入的 blob 永远是 fp32(导出脚本不用改), 这里按编译精度转一次。
     * fp16 时运行时内存减半, 而 .so 里那 0.24MB 无所谓。 */
    {
        const float *src = (const float *)(const void *)ts_weights_bytes;
        unsigned int k;
        w_all = (ts_h *)malloc((size_t)TS_TOTAL_FLOATS * sizeof(ts_h));
        for (k = 0; k < TS_TOTAL_FLOATS; k++) w_all[k] = TS_F2H(src[k]);
    }
    B_MASK = (unsigned char *)malloc(192 * 320);
#define AF(p, n) p = (ts_h *)malloc((size_t)(n) * sizeof(ts_h))
    AF(B_IN, 3 * 192 * 320);
    AF(B_A, 32 * 96 * 160);      /* stem.0 输出, 全网最大的中间张量 */
    AF(B_S, 48 * 48 * 80);       /* stem 输出, 要留到 fuse_shallow */
    AF(B_D, 48 * 48 * 80);       /* dw 输出最大: head.0 48ch@48x80 */
    AF(B_P, 48 * 48 * 80);       /* 乒 */
    AF(B_Q, 48 * 48 * 80);       /* 乓 */
    AF(B_U, 48 * 48 * 80);       /* 上采样后的深层特征 */
    AF(B_H, 4 * 48 * 80);
    AF(B_OUT, 4 * 192 * 320);
#undef AF
    /* 跑一帧校验权重游标: 图和导出脚本对不上的话这里就会炸, 不会带着
     * 错位的权重上车 */
    dummy = (unsigned char *)malloc(192 * 320 * 3);
    memset(dummy, 127, 192 * 320 * 3);
    run_graph(dummy);
    free(dummy);
    if (ts_used_floats != TS_TOTAL_FLOATS) {
        snprintf(g_err, sizeof(g_err),
                 "graph/export mismatch: used %u expect %u",
                 ts_used_floats, (unsigned)TS_TOTAL_FLOATS);
        return -2;
    }
    g_ready = 1;
    ts_banner();
    return 0;
}

int ts_infer(const unsigned char *bgr, unsigned char *mask)
{
    if (!g_ready) { snprintf(g_err, sizeof(g_err), "call ts_init first"); return -3; }
    ts_want_logits = 0;
    run_graph(bgr);
    memcpy(mask, B_MASK, 192 * 320);
    return 0;
}

int ts_infer_logits(const unsigned char *bgr, float *logits)
{
    int k;
    if (!g_ready) { snprintf(g_err, sizeof(g_err), "call ts_init first"); return -3; }
    ts_want_logits = 1;
    run_graph(bgr);
    for (k = 0; k < 4 * 192 * 320; k++) logits[k] = TS_H2F(B_OUT[k]);
    return 0;
}

const char *ts_error(void) { return g_err; }
const char *ts_backend(void) { return "cpu/" TS_PREC_NAME; }

void ts_destroy(void)
{
    if (!g_ready) return;
    free(w_all); free(B_MASK);
    free(B_IN); free(B_A); free(B_S); free(B_D); free(B_P);
    free(B_Q); free(B_U); free(B_H); free(B_OUT);
    g_ready = 0;
}
