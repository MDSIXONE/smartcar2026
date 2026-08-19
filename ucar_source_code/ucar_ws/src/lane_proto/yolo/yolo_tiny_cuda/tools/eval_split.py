#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑一整个 split, 统计类别命中率和框的 IoU。

用它做两件事:
  1. 看这套权重在引擎里到底行不行
  2. mask 0,1,2 vs 1,2,3 的 A/B —— 类别不会变(anchor 只影响框的宽高),
     变的是 IoU。谁的 IoU 高谁就是对的。
"""
import json
import os
import subprocess
import sys

ENG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ENG)
from detect import Detector                                    # noqa: E402

NAMES = [l.strip() for l in open(os.path.join(ENG, "cfg", "obj.names"),
                                 encoding="utf-8") if l.strip()]


def iou(a, b):
    ax0, ay0, ax1, ay1 = a[0]-a[2]/2, a[1]-a[3]/2, a[0]+a[2]/2, a[1]+a[3]/2
    bx0, by0, bx1, by1 = b[0]-b[2]/2, b[1]-b[3]/2, b[0]+b[2]/2, b[1]+b[3]/2
    ix0, iy0, ix1, iy1 = max(ax0, bx0), max(ay0, by0), min(ax1, bx1), min(ay1, by1)
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    inter = (ix1-ix0)*(iy1-iy0)
    return inter / (a[2]*a[3] + b[2]*b[3] - inter)


def gt_path(img):
    return img.replace("/images/", "/labels/")[:-4] + ".txt"


def run(root, split, weights, conf=0.25):
    d = os.path.join(root, "images", split)
    imgs = sorted(os.path.join(d, f) for f in os.listdir(d)
                  if f.lower().endswith((".jpg", ".png")))
    hit = wrong = miss = extra = 0
    ious, whr, confs = [], [], []
    percls = {}
    with Detector(os.path.join(ENG, "yolo_tiny_cuda"), weights, "cpu", 0,
                  stderr=subprocess.DEVNULL) as det:
        det.configure(conf=conf, nms=0.45, timings=False)
        for img in imgs:
            gts = [l.split() for l in open(gt_path(img)) if l.strip()]
            gts = [(int(p[0]), [float(x) for x in p[1:5]]) for p in gts]
            r = det.detect_file(img)
            dets = sorted(r["det"], key=lambda x: -x["conf"])
            if not dets:
                miss += len(gts)
                for c, _ in gts:
                    percls.setdefault(c, [0, 0])[1] += 1
                continue
            extra += max(0, len(dets) - len(gts))
            used = set()
            for gc, gb in gts:
                percls.setdefault(gc, [0, 0])[1] += 1
                best, bi = 0.0, -1
                for i, dd in enumerate(dets):
                    if i in used:
                        continue
                    v = iou(gb, dd["box"])
                    if v > best:
                        best, bi = v, i
                if bi < 0:
                    miss += 1
                    continue
                used.add(bi)
                ious.append(best)
                confs.append(dets[bi]["conf"])
                whr.append((dets[bi]["box"][2]/gb[2], dets[bi]["box"][3]/gb[3]))
                if dets[bi]["cls"] == gc and best >= 0.5:
                    hit += 1
                    percls[gc][0] += 1
                elif dets[bi]["cls"] != gc:
                    wrong += 1
                else:
                    miss += 1
    n = len(imgs)
    tot = hit + wrong + miss
    mi = sum(ious)/len(ious) if ious else 0
    print("%-6s %3d 图 %3d 真值 | 命中 %3d (%.1f%%) 错类 %d 漏/IoU低 %d 多检 %d"
          % (split, n, tot, hit, 100.0*hit/max(1, tot), wrong, miss, extra))
    print("       平均 IoU %.3f   平均置信度 %.3f   宽比 %.3f 高比 %.3f"
          % (mi, sum(confs)/max(1, len(confs)),
             sum(w for w, _ in whr)/max(1, len(whr)),
             sum(h for _, h in whr)/max(1, len(whr))))
    for c in sorted(percls):
        ok, t = percls[c]
        print("         %-16s %2d/%-2d" % (NAMES[c], ok, t))
    return mi, hit, tot


if __name__ == "__main__":
    root = sys.argv[1] if len(sys.argv) > 1 else "/home/claude/tl/tl7"
    w = sys.argv[2] if len(sys.argv) > 2 else "/home/claude/tl7_best.weights"
    for sp in ("val", "train"):
        if os.path.isdir(os.path.join(root, "images", sp)):
            run(root, sp, w)
