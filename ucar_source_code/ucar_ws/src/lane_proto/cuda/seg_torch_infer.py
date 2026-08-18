#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
seg_torch_infer.py — **原地替换版**: 名字/接口全不变, 底下换成固化的 .so
====================================================================
用法: 直接覆盖原来那个 seg_torch_infer.py, 上层一行都不用改 ——

    from seg_torch_infer import TorchSeg, line_points
    seg = TorchSeg("seg_nano.pth")      # 参数照旧给, 全被忽略(权重已固化)
    cls_map = seg(frame_bgr)            # (H,W) uint8, 和原来一模一样
    uv = line_points(cls_map)

为什么能这么换:
  原版 TorchSeg 干的事是 resize -> /255 -> 网络 -> argmax -> resize 回原尺寸;
  固化的 .so 把中间那三步整个吃掉了(前处理也在 kernel 里), 所以这里只剩
  一层壳。数值上和 PyTorch 逐帧对拍过: logits 最大偏差 ~5e-5, argmax 掩码
  100% 相同, 也就是说**下游拿到的 cls_map 与原来逐像素一致**。

换掉 torch 的好处(Nano 上很实在):
  - 不再 import torch: 省掉 ~800MB 内存和 3~5s 启动时间, 4GB 的机器很吃紧
  - 不用 seg_nano.pth: 权重编译期就进了 .so, 没有"忘了拷权重"这种事
  - 不依赖 torch 1.4 的 legacy 存档格式, save_for_nano.py 这一步可以扔了
  - python2/3 都能跑(ctypes), 不像 torch 只在那个 py3.6 环境里能用

依赖: 只有 numpy + cv2 + 同目录的 trackseg.py & libtrackseg.so
"""
from __future__ import print_function

import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trackseg import TrackSeg, IN_W, IN_H       # noqa: E402

CLS_OTHER, CLS_FIELD, CLS_LINE, CLS_TL = 0, 1, 2, 3


class TorchSeg(object):
    """名字保留是为了让上层 import 不用改; 里面已经没有 torch 了。
    ckpt / half / model 三个参数只为兼容旧调用签名, 一律忽略 ——
    权重和结构都固化在 .so 里, 换模型 = 换 .so。"""

    def __init__(self, ckpt=None, half=False, model=None, lib=None):
        self.ts = TrackSeg(lib or os.environ.get("TRACKSEG_LIB"))
        self.backend = self.ts.backend
        if ckpt:
            print("[seg] 忽略 ckpt=%s: 权重已固化在 %s"
                  % (ckpt, os.path.basename(self.ts.path)))

    def __call__(self, bgr):
        """返回 (H,W) uint8 类别图, 尺寸与输入帧相同(最近邻放大回去)"""
        H, W = bgr.shape[:2]
        cls_small = self.ts.infer(bgr)                  # (192,320) uint8
        if (H, W) == (IN_H, IN_W):
            return cls_small
        return cv2.resize(cls_small, (W, H), interpolation=cv2.INTER_NEAREST)

    def logits(self, bgr):
        """要原始分数时用(4,192,320) float32; 原版没有这个, 加着不碍事"""
        return self.ts.infer_logits(bgr)


def line_points(cls_map, max_pts=40, require_field_neighbor=True):
    """与原版逐行一致, 没动"""
    line = (cls_map == CLS_LINE).astype(np.uint8)
    if require_field_neighbor:
        field = (cls_map == CLS_FIELD).astype(np.uint8)
        line = line & cv2.dilate(field, np.ones((9, 9), np.uint8))
    ys, xs = line.nonzero()
    if len(xs) == 0:
        return np.zeros((0, 2))
    idx = np.random.RandomState(0).choice(len(xs), min(max_pts, len(xs)),
                                          replace=False)
    return np.stack([xs[idx], ys[idx]], axis=1).astype(np.float64)


if __name__ == "__main__":
    import time
    seg = TorchSeg()
    print("后端:", seg.backend)
    img = (np.random.RandomState(0)
           .randint(0, 255, (480, 640, 3), np.uint8))
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
    m = seg(img)
    print("输出", m.shape, m.dtype, "各类像素",
          [int((m == c).sum()) for c in range(4)])
    t = time.time()
    for _ in range(10):
        seg(img)
    print("单帧 %.1f ms" % ((time.time() - t) / 10 * 1000))
