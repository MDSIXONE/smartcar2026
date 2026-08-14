#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trackseg.py — 固化 mnv2 分割网的 Python 封装 (ctypes, 零依赖 torch)
====================================================================
    from trackseg import TrackSeg
    ts = TrackSeg()                       # 自动找 libtrackseg.so(CUDA),
                                          # 没有就退 libtrackseg_cpu.so
    mask = ts.infer(frame_bgr)            # 任意分辨率 BGR 帧
    # mask: (192,320) uint8  0其他 1场地 2白线 3红绿灯
    mask_full = ts.infer(frame_bgr, out_size=True)   # 缩回原帧大小
    logits = ts.infer_logits(frame_bgr)   # (4,192,320) float32

Nano 提示: 只依赖 numpy+cv2, 不碰系统 torch; venv 里能跑。
"""
import ctypes
import os
import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

IN_W, IN_H = 320, 192


class TrackSeg(object):
    def __init__(self, lib_path=None):
        here = os.path.dirname(os.path.abspath(__file__))
        cands = ([lib_path] if lib_path else
                 [os.path.join(here, "libtrackseg.so"),
                  os.path.join(here, "libtrackseg_cpu.so")])
        last = None
        self.lib = None
        for c in cands:
            if c and os.path.exists(c):
                try:
                    self.lib = ctypes.CDLL(c)
                    self.path = c
                    break
                except OSError as e:
                    last = e
        if self.lib is None:
            raise RuntimeError("找不到 libtrackseg*.so (先 make / make cpu): %s"
                               % last)
        self.lib.ts_error.restype = ctypes.c_char_p
        self.lib.ts_backend.restype = ctypes.c_char_p
        if self.lib.ts_init() != 0:
            raise RuntimeError("ts_init: %s" %
                               self.lib.ts_error().decode("utf-8", "replace"))
        self.backend = self.lib.ts_backend().decode()

    def _prep(self, frame_bgr):
        img = np.ascontiguousarray(frame_bgr, dtype=np.uint8)
        if img.shape[:2] != (IN_H, IN_W):
            assert cv2 is not None, "非 320x192 输入需要 cv2 做 resize"
            img = cv2.resize(img, (IN_W, IN_H), interpolation=cv2.INTER_LINEAR)
            img = np.ascontiguousarray(img)
        assert img.shape == (IN_H, IN_W, 3), "需要 BGR 三通道"
        return img

    def infer(self, frame_bgr, out_size=False):
        """-> mask (192,320) uint8; out_size=True 时缩回输入帧大小(最近邻)"""
        img = self._prep(frame_bgr)
        mask = np.empty((IN_H, IN_W), np.uint8)
        r = self.lib.ts_infer(img.ctypes.data_as(ctypes.c_char_p),
                              mask.ctypes.data_as(ctypes.c_char_p))
        if r != 0:
            raise RuntimeError("ts_infer: %s" %
                               self.lib.ts_error().decode("utf-8", "replace"))
        if out_size and frame_bgr.shape[:2] != (IN_H, IN_W):
            mask = cv2.resize(mask, (frame_bgr.shape[1], frame_bgr.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
        return mask

    def infer_logits(self, frame_bgr):
        """-> logits (4,192,320) float32 (softmax 前)"""
        img = self._prep(frame_bgr)
        logits = np.empty((4, IN_H, IN_W), np.float32)
        r = self.lib.ts_infer_logits(
            img.ctypes.data_as(ctypes.c_char_p),
            logits.ctypes.data_as(ctypes.c_char_p))
        if r != 0:
            raise RuntimeError("ts_infer_logits: %s" %
                               self.lib.ts_error().decode("utf-8", "replace"))
        return logits

    def close(self):
        if self.lib is not None:
            self.lib.ts_destroy()


if __name__ == "__main__":
    import sys
    import time
    ts = TrackSeg()
    print("backend:", ts.backend, " lib:", ts.path)
    if len(sys.argv) > 1:
        frame = cv2.imread(sys.argv[1])
        assert frame is not None
        for _ in range(3):
            mask = ts.infer(frame)                       # 预热
        t0 = time.time()
        N = 20
        for _ in range(N):
            mask = ts.infer(frame)
        dt = (time.time() - t0) / N
        print("%.1f ms/frame (%.1f FPS)  像素分布 %s" %
              (dt * 1e3, 1 / dt, np.bincount(mask.ravel(), minlength=4)))
        vis = frame.copy()
        vis = cv2.resize(vis, (IN_W, IN_H))
        vis[..., 0][mask == 1] = 0
        vis[..., 0][mask == 2] = 0
        vis[..., 1][mask == 2] = 0
        vis[..., 0][mask == 3] = 255
        vis[..., 1][mask == 3] = 0
        cv2.imwrite("trackseg_out.jpg", vis)
        print("-> trackseg_out.jpg")
    else:
        mask = ts.infer(np.full((IN_H, IN_W, 3), 127, np.uint8))
        print("dummy ok, 像素分布", np.bincount(mask.ravel(), minlength=4))
