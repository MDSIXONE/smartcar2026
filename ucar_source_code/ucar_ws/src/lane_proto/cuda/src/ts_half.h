/* ts_half.h — 存储精度抽象 (fp16 / fp32 一套源码两种编译)
 * ====================================================================
 * 设计:
 *   权重和特征图存 ts_h(fp16 时是 half), **累加器永远是 float**。
 *   累加器用 fp32 不是保守是必须: stem.1 一个输出要累加 32*9=288 项,
 *   half 的 11 位尾数扛不住; 而累加器是寄存器里的标量, 用 float 既不占
 *   带宽也不占显存。这个组合已在 PyTorch 侧用实拍验证集验证过:
 *   loss 不变、白线 IoU 不变、argmax 掩码一致率 99.998%。
 *
 * 编译开关(见 Makefile):
 *   默认            fp16 存储
 *   make FP32=1     退回 fp32 (TS_FP32), 两种实现共用同一份 core_ops.h
 *
 * CPU 后端没有硬件 half, 用软件转换(下面那两个函数, 已与 numpy.float16
 * 逐位对拍: 全部 65536 个 half 值 + 百万个随机 float 都一致)。它只用于
 * 验证, 慢一点无所谓; 车上跑的是 CUDA 后端, 走硬件指令。
 */
#ifndef TS_HALF_H
#define TS_HALF_H

#if defined(TS_FP32)

typedef float ts_h;
#define TS_H2F(x) (x)
#define TS_F2H(x) (x)
#define TS_PREC_NAME "fp32"

#else   /* ---------------- fp16 ---------------- */

#ifdef __CUDACC__
#include <cuda_fp16.h>
typedef __half ts_h;
#define TS_H2F(x) __half2float(x)
#define TS_F2H(x) __float2half(x)
#else
typedef unsigned short ts_h;
#define TS_H2F(x) ts_h2f_sw(x)
#define TS_F2H(x) ts_f2h_sw(x)

/* IEEE754 binary16 <-> binary32, 舍入到最近偶数(和 GPU 的 __float2half 一致) */
static inline float ts_h2f_sw(unsigned short h)
{
    union { float f; unsigned int u; } v;
    unsigned int sign = (unsigned int)(h & 0x8000u) << 16;
    unsigned int exp = (h >> 10) & 0x1fu;
    unsigned int man = h & 0x3ffu;
    if (exp == 0u) {
        if (man == 0u) { v.u = sign; return v.f; }
        exp = 1u;                                  /* 次正规数: 规格化 */
        while (!(man & 0x400u)) { man <<= 1; exp--; }
        man &= 0x3ffu;
        v.u = sign | ((exp + 127u - 15u) << 23) | (man << 13);
        return v.f;
    }
    if (exp == 0x1fu) {
        v.u = sign | 0x7f800000u | (man << 13);
        return v.f;
    }
    v.u = sign | ((exp + 127u - 15u) << 23) | (man << 13);
    return v.f;
}

static inline unsigned short ts_f2h_sw(float f)
{
    union { float f; unsigned int u; } v;
    unsigned int x, sign, man, h, rem, hlf;
    int exp;
    v.f = f;
    x = v.u;
    sign = (x >> 16) & 0x8000u;
    exp = (int)((x >> 23) & 0xffu);
    man = x & 0x7fffffu;
    if (exp == 0xff) {                              /* inf / nan */
        return (unsigned short)(sign | 0x7c00u | (man ? 0x200u : 0u));
    }
    exp = exp - 127 + 15;
    if (exp >= 0x1f) return (unsigned short)(sign | 0x7c00u);   /* 溢出 */
    if (exp <= 0) {                                 /* 次正规数 / 下溢 */
        int shift;
        if (exp < -10) return (unsigned short)sign;
        man |= 0x800000u;
        shift = 14 - exp;
        h = man >> shift;
        rem = man & ((1u << shift) - 1u);
        hlf = 1u << (shift - 1);
        if (rem > hlf || (rem == hlf && (h & 1u))) h++;
        return (unsigned short)(sign | h);
    }
    h = ((unsigned int)exp << 10) | (man >> 13);
    rem = man & 0x1fffu;
    if (rem > 0x1000u || (rem == 0x1000u && (h & 1u))) h++;  /* 进位会自然
                                                    * 溢到指数位, 不用特判 */
    return (unsigned short)(sign | h);
}
#endif  /* __CUDACC__ */

#define TS_PREC_NAME "fp16"

/* ---------------- half2 打包运算 (TS_H2) ----------------
 * fp16 下默认开启, -DTS_NO_H2 关掉(退回"存fp16/纯fp32累加"的标量版)。
 *
 * 为什么值得: sm_53 的 HFMA2 一条指令做两个 half FMA, 吞吐是 FFMA 的
 * 2 倍; 而标量版每个 MAC 要 2 次 h->f 转换 + 1 次 FFMA, 指令数是它的
 * 4~6 倍。这些 kernel 同时受指令发射和访存限制, 两头都省。
 *
 * 数值模型(分块累加): 在 half2 里连乘加最多 TS_BLK 个乘积, 然后刷进
 * float 累加器清零重来。块内 half 精度(11位尾数, ≤16项误差~2e-3 相对),
 * 块间 float —— stem.1 的 288 项长累加被切成 18 块, 不会像纯 half
 * 累加那样炸掉。CPU 后端用软件 half 模拟**同一个分块模型**(每步
 * 乘加后立即舍入到 half, 与 __hfma2 只差"单次舍入 vs 两次舍入",
 * 概率 ~2^-13 差 1ulp), 所以无卡机器上照样能验数。
 *
 * 打包方向是**空间**(同通道相邻两个输出像素), 不是通道:
 *   - 1x1 卷积里 src 沿空间连续 -> 直接 half2 对齐加载(HW 全是偶数,
 *     基址来自 cudaMalloc, 偶数偏移 x2 字节 = 4 字节对齐, 恒成立)
 *   - 权重对两个像素是同一个标量 -> 广播, 不用重排权重 blob
 *   - 3x3 卷积 stride=2 时两个输出的输入错开 2 列, 不连续 -> 标量取数
 *     再 __halves2half2 拼, 乘加仍走 HFMA2 */
#ifndef TS_NO_H2
#define TS_H2 1
#undef TS_PREC_NAME
#define TS_PREC_NAME "fp16+h2"
#define TS_BLK 16          /* half 里最多连乘加这么多项就刷 float */

#ifdef __CUDACC__
typedef __half2 ts_h2;
#define TS_H2_LD(p)       (*(const __half2 *)(const void *)(p))
#define TS_H2_ST(p, v)    (*(__half2 *)(void *)(p) = (v))
#define TS_H2_ZERO()      __float2half2_rn(0.f)
#define TS_H2_BCAST(w)    __half2half2(w)
#define TS_H2_PACK(a, b)  __halves2half2((a), (b))
#define TS_HFMA2(a, b, c) __hfma2((a), (b), (c))
#define TS_H2_LOF(v)      __low2float(v)
#define TS_H2_HIF(v)      __high2float(v)
#else
typedef struct { ts_h x, y; } ts_h2;
static inline ts_h2 ts_h2_ld_sw(const ts_h *p)
{ ts_h2 r; r.x = p[0]; r.y = p[1]; return r; }
static inline void ts_h2_st_sw(ts_h *p, ts_h2 v) { p[0] = v.x; p[1] = v.y; }
static inline ts_h2 ts_h2_zero_sw(void) { ts_h2 r; r.x = 0; r.y = 0; return r; }
static inline ts_h2 ts_h2_bcast_sw(ts_h w)
{ ts_h2 r; r.x = w; r.y = w; return r; }
static inline ts_h2 ts_h2_pack_sw(ts_h a, ts_h b)
{ ts_h2 r; r.x = a; r.y = b; return r; }
static inline ts_h2 ts_hfma2_sw(ts_h2 a, ts_h2 b, ts_h2 c)
{
    ts_h2 r;
    r.x = ts_f2h_sw(ts_h2f_sw(a.x) * ts_h2f_sw(b.x) + ts_h2f_sw(c.x));
    r.y = ts_f2h_sw(ts_h2f_sw(a.y) * ts_h2f_sw(b.y) + ts_h2f_sw(c.y));
    return r;
}
#define TS_H2_LD(p)       ts_h2_ld_sw(p)
#define TS_H2_ST(p, v)    ts_h2_st_sw((p), (v))
#define TS_H2_ZERO()      ts_h2_zero_sw()
#define TS_H2_BCAST(w)    ts_h2_bcast_sw(w)
#define TS_H2_PACK(a, b)  ts_h2_pack_sw((a), (b))
#define TS_HFMA2(a, b, c) ts_hfma2_sw((a), (b), (c))
#define TS_H2_LOF(v)      ts_h2f_sw((v).x)
#define TS_H2_HIF(v)      ts_h2f_sw((v).y)
#endif  /* __CUDACC__ */
#endif  /* TS_NO_H2 */

#endif  /* TS_FP32 */

#endif /* TS_HALF_H */
