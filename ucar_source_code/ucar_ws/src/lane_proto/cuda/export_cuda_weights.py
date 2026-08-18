#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
seg_best.pth (TrackSegNet = small, 4类) -> 固化权重
====================================================================
做三件事:
  1. 把所有 Conv+BN 折叠成 Conv+bias (eps=1e-5), 数学上完全等价
  2. 按 C 代码消费的固定顺序拼成一个 float32 blob -> weights.bin
  3. 生成 gen/weights_embed.c (字节数组) + gen/net_meta.h (总量校验)
     —— 权重直接编进 .so, 运行时零外部文件

顺序约定(与 src/net_graph.inc 的 TAKE 顺序一一对应, 改一处必须改两处):
  mean[3], std[3]                 # small 训练时只做 /255, 所以写 0/1
  stem.0:  conv3x3 w(32,3,3,3),  b(32)
  stem.1:  conv3x3 w(48,32,3,3), b(48)
  deep.0..deep.4 每个 DSConv:
           dw w(cin,1,3,3), b(cin)
           pw w(cout,cin),  b(cout)
  fuse_deep    w(48,96), b(48)
  fuse_shallow w(48,48), b(48)
  head.0:  dw w(48,1,3,3), b(48); pw w(48,48), b(48)
  head.1:  w(4,48), b(4)          # 原生带 bias, 无 BN, 不折叠

用法: python3 export_cuda_weights.py [seg_best.pth]
"""
import sys
from pathlib import Path
import numpy as np
import torch

EPS = 1e-5
# deep 分支的 DSConv: (cin, cout) —— 与 seg_model.TrackSegNet.deep 一致
DEEP = [(48, 64), (64, 64), (64, 96), (96, 96), (96, 96)]


def fold(sd, conv, bn):
    """Conv(no bias)+BN -> (w', b'), eps 在 sqrt 里面"""
    w = sd[conv + ".weight"].numpy().astype(np.float64)
    g = sd[bn + ".weight"].numpy().astype(np.float64)
    b = sd[bn + ".bias"].numpy().astype(np.float64)
    m = sd[bn + ".running_mean"].numpy().astype(np.float64)
    v = sd[bn + ".running_var"].numpy().astype(np.float64)
    s = g / np.sqrt(v + EPS)
    wf = w * s.reshape(-1, 1, 1, 1)
    bf = b - m * s
    return wf.astype(np.float32), bf.astype(np.float32)


def main():
    ckpt = sys.argv[1] if len(sys.argv) > 1 else "seg_best.pth"
    sd = torch.load(ckpt, map_location="cpu")
    assert "stem.0.0.weight" in sd, (
        "这不是 small(TrackSegNet) 的权重。mnv2 的请用 ../cuda_seg/ 那套")
    out = []

    def put(*arrs):
        for a in arrs:
            out.append(np.ascontiguousarray(a, np.float32).ravel())

    # small 的预处理只有 /255(见 train_seg.TrackDataset.__getitem__),
    # 没有 ImageNet 均值方差。为了和 mnv2 共用同一个 OP_PRE, 这里写恒等值。
    put(np.zeros(3, np.float32), np.ones(3, np.float32))

    for i, cout in ((0, 32), (1, 48)):                 # stem
        w, b = fold(sd, "stem.%d.0" % i, "stem.%d.1" % i)
        assert w.shape[0] == cout and w.shape[2:] == (3, 3), (i, w.shape)
        put(w, b)

    for i, (cin, cout) in enumerate(DEEP):             # deep
        p = "deep.%d" % i
        w, b = fold(sd, p + ".dw.0", p + ".dw.1")
        assert w.shape == (cin, 1, 3, 3), (i, w.shape)
        put(w, b)
        w, b = fold(sd, p + ".pw.0", p + ".pw.1")
        assert w.shape == (cout, cin, 1, 1), (i, w.shape)
        put(w, b)

    w, b = fold(sd, "fuse_deep.0", "fuse_deep.1")
    assert w.shape == (48, 96, 1, 1), w.shape
    put(w, b)
    w, b = fold(sd, "fuse_shallow.0", "fuse_shallow.1")
    assert w.shape == (48, 48, 1, 1), w.shape
    put(w, b)

    w, b = fold(sd, "head.0.dw.0", "head.0.dw.1")      # head.0 = DSConv
    assert w.shape == (48, 1, 3, 3), w.shape
    put(w, b)
    w, b = fold(sd, "head.0.pw.0", "head.0.pw.1")
    assert w.shape == (48, 48, 1, 1), w.shape
    put(w, b)
    # head.1 是裸 Conv2d(48,4,1) 带 bias, 没有 BN 可折
    put(sd["head.1.weight"].numpy(), sd["head.1.bias"].numpy())

    blob = np.concatenate(out)
    blob.tofile("weights.bin")
    Path("gen").mkdir(exist_ok=True)
    Path("gen/net_meta.h").write_text(
        "/* 自动生成: export_cuda_weights.py, 勿手改 */\n"
        "#define TS_TOTAL_FLOATS %du\n" % blob.size)
    by = blob.tobytes()
    with open("gen/weights_embed.c", "w") as f:
        f.write("/* 自动生成: 固化的折叠权重 (%d floats) */\n" % blob.size)
        f.write("const unsigned int ts_weights_len = %du;\n" % len(by))
        f.write("const unsigned char ts_weights_bytes[] = {\n")
        for i in range(0, len(by), 24):
            f.write(",".join(str(c) for c in by[i:i + 24]) + ",\n")
        f.write("};\n")
    print("weights.bin + gen/weights_embed.c: %d floats (%.2f MB)"
          % (blob.size, blob.size * 4 / 1e6))


if __name__ == "__main__":
    main()
