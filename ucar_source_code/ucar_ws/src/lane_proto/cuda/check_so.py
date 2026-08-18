#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
check_so.py — **不用 torch** 的 .so 交叉验证 + 计时 (车上直接跑)
====================================================================
思路: 车上没有 torch, 但我们有一个已经和 PyTorch 对拍过的东西可以当基准 ——
      **CPU/fp32 版的 .so**。它和 PyTorch 的 argmax 掩码在 PC 上验过是
      100% 一致的, 而它和 CUDA 版共用同一份 core_ops.h + net_graph.inc,
      所以拿它当"标尺"去量 CUDA/fp16 版, 结论一样可信, 全程只要 numpy+cv2。

在车上先编三个库(互不覆盖):
    make                          && mv libtrackseg.so     libtrackseg_fp16.so
    make clean && make FP32=1     && mv libtrackseg.so     libtrackseg_fp32.so
    make clean && make cpu FP32=1 && mv libtrackseg_cpu.so libtrackseg_ref.so
然后:
    python3 check_so.py --libs libtrackseg_ref.so libtrackseg_fp32.so \\
                               libtrackseg_fp16.so --images dump/*.jpg

第一个 --libs 是基准, 后面每个都跟它比。看三件事:
  1. **掩码一致率** —— fp32 版应当 100.0000%(同一份数学, 只是跑在 GPU 上);
     fp16 版应当 >=99.9%(和 PC 上量的 99.998% 同量级)
  2. 各类像素数 —— 某一类整体偏了的话这里最直观
  3. 每帧耗时 p50/p90 —— fp16 到底比 fp32 快多少, 这是换它的唯一理由

CPU 基准库很慢(ARM 上一帧可能好几秒), 所以默认只测 8 帧; 计时那栏看
CUDA 那两个就行。加 --skip-ref-time 可以不给基准库计时。
"""
from __future__ import print_function

import argparse
import glob
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trackseg import TrackSeg, IN_W, IN_H       # noqa: E402

try:
    import cv2
except ImportError:
    cv2 = None


def grab_frames(dev, n, save_dir=None):
    """直接从相机抓 **原始帧** —— 这才是网络在车上真正吃的东西。
    ⚠ 别拿 dump/*.jpg 测: 那些是 960x576 的**诊断图**(白线标红、触发区
    标绿、红绿灯标品红、还写了字), 属于严重的分布外输入, 网络在上面
    的 logits 本来就摇摆, 一堆像素处在"两类分数几乎相等"的状态,
    fp16 的舍入自然更容易把它们翻过去。实测同一套 CPU 库: 真实帧的
    fp16/fp32 分歧 0.003%, 换成诊断图立刻涨到 0.029% —— 差 9 倍, 而这
    跟 kernel 写得对不对毫无关系。"""
    cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        sys.exit("打不开相机 %s (roslaunch 还开着? 相机是独占的)" % dev)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    out = []
    for k in range(n):
        for _ in range(3):          # 甩掉 V4L2 缓冲里的陈帧
            cap.grab()
        ok, im = cap.read()
        if not ok or im is None:
            continue
        if save_dir:
            if not os.path.isdir(save_dir):
                os.makedirs(save_dir)
            cv2.imwrite(os.path.join(save_dir, "raw_%03d.jpg" % k), im)
        if im.shape[:2] != (IN_H, IN_W):
            im = cv2.resize(im, (IN_W, IN_H), interpolation=cv2.INTER_LINEAR)
        out.append(("cam_%03d" % k, np.ascontiguousarray(im)))
        time.sleep(0.05)
    cap.release()
    return out


def boundary_frac(ref, got):
    """分歧像素里有多少落在**类别边界**上(参考掩码 3x3 邻域内不止一类)。
    接近 100% 说明差异全在边缘那些"两类打平"的地方, 是 fp16 舍入的正常
    表现, 对下游没影响; 要是大片区域中间也翻, 那才是真出问题。"""
    diff = (ref != got)
    if not diff.any():
        return 1.0, 0
    k = np.ones((3, 3), np.uint8)
    edge = cv2.dilate(ref, k) != cv2.erode(ref, k)
    return float(edge[diff].mean()), int(diff.sum())


def load_frames(pats, n):
    files = []
    for p in pats:
        files += sorted(glob.glob(p))
    files = files[:n]
    out = []
    for f in files:
        im = cv2.imread(f)
        if im is None:
            continue
        if im.shape[:2] != (IN_H, IN_W):
            im = cv2.resize(im, (IN_W, IN_H), interpolation=cv2.INTER_LINEAR)
        out.append((os.path.basename(f), np.ascontiguousarray(im)))
    if not out:      # 没图也能跑: 合成几帧, 至少能验"两个库结果一致"
        rng = np.random.RandomState(0)
        out = [("随机噪声", rng.randint(0, 256, (IN_H, IN_W, 3), np.uint8)),
               ("灰底", np.full((IN_H, IN_W, 3), 127, np.uint8)),
               ("渐变", np.tile(np.arange(IN_W, dtype=np.uint8)[None, :, None],
                                (IN_H, 1, 3)))]
        print("没找到图片, 用合成帧(只能验一致性, 不代表真实场景)")
    return out


def bench(ts, frames, rounds):
    ms = []
    for _, im in frames:
        ts.infer(im)                       # 预热(首帧含 CUDA 上下文/分配)
        break
    for _ in range(rounds):
        for _, im in frames:
            t = time.time()
            ts.infer(im)
            ms.append((time.time() - t) * 1000.0)
    a = np.sort(np.array(ms))
    return (a.mean(), a[len(a) // 2], a[int(len(a) * 0.9)], a.max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", nargs="+", required=True,
                    help="第一个是基准(建议 CPU/fp32 那个), 后面的都跟它比")
    ap.add_argument("--images", nargs="*", default=[],
                    help="图片通配符。⚠ 别用 dump/*.jpg(那是画了标注的诊断图, "
                         "分布外输入会让 fp16 分歧虚高好几倍), 用 --camera")
    ap.add_argument("--camera", default=None,
                    help="直接抓原始帧测, 如 --camera /dev/ucar_camera (推荐)")
    ap.add_argument("--save-raw", default=None,
                    help="把抓到的原始帧存下来, 方便下次复现同一组")
    ap.add_argument("--n", type=int, default=8, help="用几帧(基准库很慢)")
    ap.add_argument("--rounds", type=int, default=5, help="计时重复几轮")
    ap.add_argument("--skip-ref-time", action="store_true",
                    help="不给基准库计时(CPU 版太慢时用)")
    a = ap.parse_args()

    if a.camera:
        frames = grab_frames(a.camera, a.n, a.save_raw)
    else:
        if a.images and any("dump" in x for x in a.images):
            print("!! 你指的是 dump/ 里的诊断图(带标注叠加)。那是分布外输入,"
                  " fp16 分歧会虚高好几倍, 判不了 kernel 对不对。\n"
                  "   建议改用: --camera /dev/ucar_camera --n 40")
        frames = load_frames(a.images or ["dump/*.jpg"], a.n)
    print("测试帧 %d 张: %s" % (len(frames),
                              ", ".join(n for n, _ in frames[:6]) +
                              (" ..." if len(frames) > 6 else "")))

    ref_masks = None
    ok = True
    for i, path in enumerate(a.libs):
        if not os.path.exists(path):
            print("!! 找不到 %s" % path)
            ok = False
            continue
        # 必须转绝对路径: dlopen 对"不含斜杠"的名字会去系统库目录找,
        # 而不是当前目录 —— 传 libtrackseg_ref.so 会直接 not found
        ts = TrackSeg(os.path.abspath(path))
        masks = [ts.infer(im) for _, im in frames]
        tag = "%s [%s]" % (os.path.basename(path), ts.backend)
        if i == 0:
            ref_masks = masks
            cnt = np.bincount(np.concatenate([m.ravel() for m in masks]),
                              minlength=4)
            print("\n基准 %s   各类像素合计 %s" % (tag, list(cnt)))
        else:
            agree = [float((m == r).mean()) for m, r in zip(masks, ref_masks)]
            cnt = np.bincount(np.concatenate([m.ravel() for m in masks]),
                              minlength=4)
            rcnt = np.bincount(np.concatenate([r.ravel() for r in ref_masks]),
                               minlength=4)
            dpix = [(int(c) - int(r)) for c, r in zip(cnt, rcnt)]
            bf = [boundary_frac(r, m) for m, r in zip(masks, ref_masks)]
            nd = sum(x[1] for x in bf)
            edge = (sum(x[0] * x[1] for x in bf) / nd) if nd else 1.0
            print("\n%s\n  掩码一致 平均 %.4f%%  最差 %.4f%%   各类像素差 %s"
                  "\n  分歧像素 %d 个, %.1f%% 落在类别边界上%s"
                  % (tag, 100 * np.mean(agree), 100 * min(agree), dpix, nd,
                     100 * edge,
                     " (边界舍入, 正常)" if edge > 0.9 else
                     " (!! 有非边界像素翻了, 要查)"))
            need = 0.999 if "fp16" in ts.backend else 1.0
            good = min(agree) >= need
            print("  判据: %s 要求 >= %.3f%%  ->  %s"
                  % (ts.backend, 100 * need, "通过" if good else "不通过!!"))
            if not good:
                ok = False
        if not (a.skip_ref_time and i == 0):
            m, p50, p90, mx = bench(ts, frames, a.rounds)
            print("  单帧耗时 mean %.1f ms  p50 %.1f  p90 %.1f  max %.1f"
                  "   (%.1f fps)" % (m, p50, p90, mx, 1000.0 / max(m, 1e-6)))
    print("\n=> %s" % ("全部通过" if ok else "有不通过项, 别上车"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
