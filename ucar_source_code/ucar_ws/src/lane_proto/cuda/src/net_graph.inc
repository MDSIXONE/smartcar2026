/* net_graph.inc — TrackSegNet(small, Fast-SCNN 风格) 前向图
 * ====================================================================
 * 两个后端 #include 同一份, 展开成 ts_run_graph() 的函数体。
 * 权重顺序与 export_cuda_weights.py 严格一致(TAKE 依次消费),
 * 改网络必须同时改导出脚本与本文件, 否则 ts_init 的游标校验会报错。
 *
 * 与 mnv2 版的结构差异(为什么不能共用一份 inc):
 *   - 干路是 Fast-SCNN 风格: 两层普通 conv3x3 直接下采到 1/4(通道少但
 *     特征图大), 而不是 mnv2 的 13 个倒残差块
 *   - 只有一条跳连: 深层 1/16 上采到 1/4 与浅层相加, 没有 1/8 那一级
 *   - 激活全是**普通 ReLU**(act=2), 不是 mnv2 的 ReLU6(act=1)
 *   - 无残差 add(除了那一次融合), 所以用不到 OP_ADD
 *   - 预处理无 mean/std 归一化(训练时只做 /255), 导出时写 mean=0 std=1,
 *     省得再分一套 OP_PRE
 *
 * 依赖调用方定义:
 *   OP_PRE / OP_C3 / OP_DW / OP_C1 / OP_ADDRELU / OP_UP / OP_ARGMAX
 *   缓冲指针: B_IN,B_A,B_S,B_D,B_P,B_Q,B_U,B_H,B_OUT,B_MASK
 *   const ts_h *wp = 权重 blob 起始; w_all = 同上(算游标用)
 *   (ts_h = fp16 时的 half / fp32 时的 float, 见 ts_half.h)
 * 空间尺寸: 输入 192x320; /2:96x160 /4:48x80 /8:24x40 /16:12x20
 */

#define TAKE(n) (wp += (n), wp - (n))

{
    const ts_h *mean_ = TAKE(3);
    const ts_h *std_ = TAKE(3);
    const ts_h *w_, *b_;

    OP_PRE(B_IN, in_bgr, mean_, std_, 192, 320);

    /* ---- stem: 两层 conv3x3 s2, 直接到 1/4 ---- */
    /* stem.0: 3->32, 192x320 -> 96x160, relu */
    w_ = TAKE(32 * 3 * 9); b_ = TAKE(32);
    OP_C3(B_A, B_IN, w_, b_, 32, 3, 192, 320, 96, 160, 2, 2);
    /* stem.1: 32->48, 96x160 -> 48x80, relu。
     * 这层是全网最贵的一层(48*48*80*32*9 ≈ 53M MAC, 占总量一半),
     * 因为它是**普通**卷积不是 depthwise。B_S 要一直留到 fuse_shallow。 */
    w_ = TAKE(48 * 32 * 9); b_ = TAKE(48);
    OP_C3(B_S, B_A, w_, b_, 48, 32, 96, 160, 48, 80, 2, 2);

    /* DSCONV(输出buf, 输入buf, cin, cout, Hs,Ws, Hd,Wd, 步幅)
     * = depthwise3x3+BN+ReLU 接 pointwise1x1+BN+ReLU */
#define DSCONV(BO, BI, CIN, COUT, HS, WS, HD, WD, S)                        \
    do {                                                                     \
        w_ = TAKE((CIN) * 9); b_ = TAKE(CIN);                                \
        OP_DW(B_D, BI, w_, b_, CIN, HS, WS, HD, WD, S, 2);                   \
        w_ = TAKE((COUT) * (CIN)); b_ = TAKE(COUT);                          \
        OP_C1(BO, B_D, w_, b_, COUT, CIN, (HD) * (WD), 2);                   \
    } while (0)

    /* ---- deep: 1/4 -> 1/16 语义分支 ---- */
    DSCONV(B_P, B_S, 48, 64, 48, 80, 24, 40, 2);   /* deep.0 */
    DSCONV(B_Q, B_P, 64, 64, 24, 40, 24, 40, 1);   /* deep.1 */
    DSCONV(B_P, B_Q, 64, 96, 24, 40, 12, 20, 2);   /* deep.2 */
    DSCONV(B_Q, B_P, 96, 96, 12, 20, 12, 20, 1);   /* deep.3 */
    DSCONV(B_P, B_Q, 96, 96, 12, 20, 12, 20, 1);   /* deep.4 */

    /* ---- 融合: 深层上采回 1/4 与浅层相加 ---- */
    /* fuse_deep: 1x1 96->48 +BN, **线性**(act=0), 12x20 */
    w_ = TAKE(48 * 96); b_ = TAKE(48);
    OP_C1(B_Q, B_P, w_, b_, 48, 96, 12 * 20, 0);
    /* 12x20 -> 48x80 (PyTorch 是 size=s.shape[2:], 即直接给目标尺寸) */
    OP_UP(B_U, B_Q, 48, 12, 20, 48, 80);
    /* fuse_shallow: 1x1 48->48 +BN, 线性, 输入是 stem 输出 */
    w_ = TAKE(48 * 48); b_ = TAKE(48);
    OP_C1(B_P, B_S, w_, b_, 48, 48, 48 * 80, 0);
    OP_ADDRELU(B_U, B_P, 48 * 48 * 80);            /* y = relu(d + s) */

    /* ---- head ---- */
    DSCONV(B_P, B_U, 48, 48, 48, 80, 48, 80, 1);   /* head.0 */
    /* head.1: 1x1 48->4, **原生带 bias 无 BN**, 线性 */
    w_ = TAKE(4 * 48); b_ = TAKE(4);
    OP_C1(B_H, B_P, w_, b_, 4, 48, 48 * 80, 0);
#undef DSCONV

    /* 上采样 48x80 -> 192x320 (scale_factor=4) + argmax。
     * 平时(只要掩码)走融合版, 不写 B_OUT, 省 ~1MB 往返 + 一次启动;
     * ts_infer_logits 才走两步版把 logits 留在 B_OUT。
     * 两条分支都不消费权重, 游标校验不受影响。 */
    if (ts_want_logits) {
        OP_UP(B_OUT, B_H, 4, 48, 80, 192, 320);
        OP_ARGMAX(B_MASK, B_OUT, 4, 192 * 320);
    } else {
        OP_UPARG(B_MASK, B_H, 4, 48, 80, 192, 320);
    }

    ts_used_floats = (unsigned int)(wp - w_all);
}
#undef TAKE
