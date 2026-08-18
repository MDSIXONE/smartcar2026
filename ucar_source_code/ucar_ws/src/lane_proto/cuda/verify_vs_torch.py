#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_vs_torch.py — 固化的 .so 与 PyTorch 原网逐帧对拍 (small 版)
====================================================================
比什么:
  1. logits 的最大绝对误差 —— 检验 BN 折叠、卷积、上采样都写对了
  2. argmax 掩码的逐像素一致率 —— 这才是下游真正吃的东西
  3. 各类像素数对比 —— 万一某一类整体偏了, 这里最直观

用法(有 torch 的机器上, 不必有显卡):
    make cpu
    python3 verify_vs_torch.py                 # 随机图 + test_frame.jpg
    python3 verify_vs_torch.py 某张图.jpg ...
    TRACKSEG_LIB=./libtrackseg.so python3 verify_vs_torch.py   # 验 CUDA 版

判据按 .so 的编译精度自动切换(ts_backend() 会报 "cuda/fp16" 这种):
  fp32  logits 误差 <1e-2 且掩码 **100%** 一致 —— 差一个像素就是图写错了
  fp16  logits 误差会到 1e-1(必然的, 存储只有 11 位尾数), 判据换成:
        掩码一致 >=99.9%, 且**与 fp16 模拟的偏差远小于与 fp32 的偏差**
        —— 后面这条才是关键: 它证明 kernel 做的正是我们在 PyTorch 里
        验证过的那套"存 fp16 / 累加 fp32", 而不是别处写错了。
"""
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seg_model import TrackSegNet, IN_W, IN_H     # noqa: E402
from trackseg import TrackSeg                     # noqa: E402

try:
    import cv2
except ImportError:
    cv2 = None


class Fp16Sim(object):
    """把权重和每层输出压到 fp16 再还原, 算术仍走 fp32 —— 和
    seg/fp16_vs_fp32.py 里的模拟完全一致, 也就是 kernel 应该做的事"""

    def __init__(self, net):
        self.net = net
        self.backup = [p.detach().clone() for p in net.parameters()]
        self.handles = []

    def __enter__(self):
        import torch.nn as nn
        for p in self.net.parameters():
            p.data = p.data.half().float()

        def hook(mod, inp, out):
            if isinstance(out, torch.Tensor):
                return out.half().float()
            return out
        for mod in self.net.modules():
            if isinstance(mod, (nn.Conv2d, nn.ReLU, nn.Identity)):
                self.handles.append(mod.register_forward_hook(hook))
        return self

    def __exit__(self, *a):
        for p, b in zip(self.net.parameters(), self.backup):
            p.data = b
        for h in self.handles:
            h.remove()


def fold_bn(net):
    """先折叠 Conv+BN 再模拟 fp16 —— 车上存的就是折叠后的权重"""
    import torch.nn as nn
    from torch.nn.utils.fusion import fuse_conv_bn_eval
    for mod in net.modules():
        if isinstance(mod, nn.Sequential):
            for i in range(len(mod) - 1):
                if isinstance(mod[i], nn.Conv2d) and \
                        isinstance(mod[i + 1], nn.BatchNorm2d):
                    mod[i] = fuse_conv_bn_eval(mod[i], mod[i + 1])
                    mod[i + 1] = nn.Identity()
    return net


def torch_logits(net, bgr):
    """和训练时完全一致的前处理: BGR->RGB, /255, CHW"""
    x = torch.from_numpy(bgr[:, :, ::-1].copy()).permute(2, 0, 1).float() / 255
    with torch.no_grad():
        return net(x[None])[0].numpy()


def main():
    ckpt = os.environ.get("TS_CKPT", "seg_best.pth")
    net = TrackSegNet().eval()
    net.load_state_dict(torch.load(ckpt, map_location="cpu"))
    ts = TrackSeg(os.environ.get("TRACKSEG_LIB"))
    fp16 = "fp16" in ts.backend
    print("后端 = %s   权重 = %s   判据 = %s"
          % (ts.backend, ckpt, "fp16(宽松)" if fp16 else "fp32(严格)"))
    sim = None
    if fp16:
        sim = TrackSegNet().eval()
        sim.load_state_dict(torch.load(ckpt, map_location="cpu"))
        sim = fold_bn(sim)

    frames = []
    for p in sys.argv[1:]:
        im = cv2.imread(p) if cv2 else None
        if im is None:
            print("读不了", p)
            continue
        if im.shape[:2] != (IN_H, IN_W):
            im = cv2.resize(im, (IN_W, IN_H), interpolation=cv2.INTER_LINEAR)
        frames.append((os.path.basename(p), np.ascontiguousarray(im)))
    if not frames:      # 没给图就用随机噪声 + 灰底, 一样能验数值
        rng = np.random.RandomState(0)
        frames = [("随机噪声", rng.randint(0, 256, (IN_H, IN_W, 3), np.uint8)),
                  ("灰底", np.full((IN_H, IN_W, 3), 127, np.uint8))]

    ok = True
    for name, im in frames:
        ref = torch_logits(net, im)
        got = ts.infer_logits(im)
        d = np.abs(ref - got)
        m_ref, m_got = ref.argmax(0), got.argmax(0)
        same = float((m_ref == m_got).mean())
        print("%-14s vs fp32: logits max|Δ| %.3e  掩码一致 %.4f%%"
              % (name, d.max(), 100.0 * same))
        cnt_r = [int((m_ref == c).sum()) for c in range(4)]
        cnt_g = [int((m_got == c).sum()) for c in range(4)]
        print("               各类像素 torch %s / so %s" % (cnt_r, cnt_g))
        if fp16:
            with Fp16Sim(sim):
                ref16 = torch_logits(sim, im)
            d16 = np.abs(ref16 - got)
            same16 = float((ref16.argmax(0) == m_got).mean())
            print("               vs fp16模拟: logits max|Δ| %.3e  "
                  "掩码一致 %.4f%%  (比 fp32 那栏小 %.1fx = kernel 没写错)"
                  % (d16.max(), 100.0 * same16,
                     d.max() / max(d16.max(), 1e-12)))
            if same < 0.999 or same16 < 0.999 or d16.max() > d.max():
                ok = False
        elif same < 1.0 or d.max() > 1e-2:
            ok = False
    print("=> %s" % ("通过" if ok else "不通过!! 图或导出顺序有问题"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
