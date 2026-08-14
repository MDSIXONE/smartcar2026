#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yolo_probe.py — 单独验红绿灯识别 (不用 ROS, python2/3 都能跑)
====================================================================
lane_follow 里的认灯是起点那一次(认完就杀), 想反复调角度、换姿势、
验左右有没有反, 用这个 —— 连续认, 不动车, 不用起 ROS。

⚠ 相机是独占的: 跑这个之前先把 roslaunch 停掉, 否则打不开 /dev/video0。

    cd ~/ucar_ws/src/lane_proto/scripts
    python2 yolo_probe.py                 # 摄像头连续认, Ctrl-C 停
    python2 yolo_probe.py -n 20           # 只跑 20 帧就退
    python2 yolo_probe.py --raw           # 喂不去畸变的帧(对应 yolo_use_raw)
    python2 yolo_probe.py --zoom 2        # 远处的灯太小认不到时放大中间再认
    python2 yolo_probe.py --img a.jpg b.jpg   # 拿现成图片试, 不开相机

每帧打印所有检出, 并把**实际喂给 yolo 的那张图**连框存到
dump/probe_%03d.jpg。左右反没反就看这个图: 图里箭头朝左而标签是
right, 就在 launch 里加 yolo_swap_lr:=true。

退出时打印票数汇总 —— 摆一个姿势连拍十几帧, 看看稳不稳。
"""
from __future__ import print_function

import argparse
import os
import sys
import time

import numpy as np
import cv2

here = os.path.dirname(os.path.abspath(__file__))
pkg = os.path.dirname(here)
sys.path.insert(0, here)
from yolo_client import YoloProc, find_exe, find_weights   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img", nargs="*", default=None,
                    help="给了就认这些图片, 不开相机")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("-n", "--frames", type=int, default=0, help="0=一直跑")
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--nms", type=float, default=0.45)
    ap.add_argument("--raw", action="store_true",
                    help="喂原始帧(只翻正不去畸变), 对应 yolo_use_raw:=true")
    ap.add_argument("--no-mirror", action="store_true",
                    help="相机不是镜像的时候用; 默认翻正")
    ap.add_argument("--zoom", type=float, default=1.0,
                    help="数字变焦: 抠中间 1/zoom 放大再送(远处小灯用), "
                         "对应 yolo_zoom")
    ap.add_argument("--zoom-cy", type=float, default=0.45,
                    help="抠图中心行(0=顶 1=底), 灯一般偏上")
    ap.add_argument("--no-swap-lr", action="store_true",
                    help="不做左右互换。节点里 yolo_swap_lr 默认 true(这套"
                         "权重是镜像数据训的), 探针默认跟它一致")
    ap.add_argument("--backend", default="auto")
    ap.add_argument("--yolo-dir", default=os.path.join(pkg, "yolo",
                                                       "yolo_tiny_cuda"))
    ap.add_argument("--dump", default=os.path.join(pkg, "dump"))
    a = ap.parse_args()

    exe = find_exe(a.yolo_dir)
    wts = find_weights(a.yolo_dir)
    if not exe:
        sys.exit("在 %s 里没找到可执行文件 —— 先在那个目录 make GPU=1 "
                 "ARCH=sm_53" % a.yolo_dir)
    if not wts:
        sys.exit("在 %s 里没找到 .weights" % a.yolo_dir)
    if a.dump and not os.path.isdir(a.dump):
        os.makedirs(a.dump)

    m1 = m2 = None
    if not a.raw:
        z = np.load(os.path.join(pkg, "config", "maps_640.npz"))
        m1, m2 = z["m1"], z["m2"]

    cap = None
    if not a.img:
        cap = cv2.VideoCapture(a.device)
        if not cap.isOpened():
            sys.exit("打不开相机 %s —— roslaunch 还开着? 相机是独占的"
                     % a.device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("exe     = %s" % exe)
    print("weights = %s" % wts)
    t0 = time.time()
    y = YoloProc(exe, wts, a.backend, 0, timeout=30.0)
    print("backend = %s  classes = %s  (启动 %.1fs)"
          % (y.backend, ",".join(y.classes), time.time() - t0))
    if y.backend != "cuda":
        print("!! 跑在 %s 上, 单帧要好几秒。make 时带上 GPU=1 ARCH=sm_53"
              % y.backend)
    y.configure(conf=a.conf, nms=a.nms, timings=True)

    votes = {}
    i = 0
    try:
        while True:
            if a.img:
                if i >= len(a.img):
                    break
                im = cv2.imread(a.img[i])
                if im is None:
                    print("读不了 %s" % a.img[i])
                    i += 1
                    continue
                if im.shape[1] != 640 or im.shape[0] != 480:
                    im = cv2.resize(im, (640, 480))
                tag = os.path.basename(a.img[i])
            else:
                for _ in range(3):        # 甩掉 V4L2 缓冲里的陈帧
                    cap.grab()
                ok, im = cap.read()
                if not ok or im is None:
                    print("取帧失败")
                    time.sleep(0.1)
                    continue
                tag = "cam"
            if m1 is not None:
                im = cv2.remap(im, m1, m2, cv2.INTER_LINEAR)
            if not a.no_mirror:
                im = cv2.flip(im, 1)      # 相机输出是镜像的, 翻正再认
            if a.zoom > 1.001:            # 数字变焦, 和节点里一样
                h, w = im.shape[:2]
                cw, ch = int(w / a.zoom), int(h / a.zoom)
                x0 = (w - cw) // 2
                y0 = max(0, min(h - ch, int(a.zoom_cy * h - ch / 2)))
                im = cv2.resize(im[y0:y0 + ch, x0:x0 + cw], (w, h))

            t = time.time()
            r = y.detect_bgr(im, timeout=25.0)
            dt = (time.time() - t) * 1000.0
            dets = r.get("det", [])
            # 和节点一致地做左右互换, 免得探针和实车判定不一样
            swap = {"left": "right", "right": "left"}
            for d in dets:
                d["shown"] = d["name"] if a.no_swap_lr \
                    else swap.get(d["name"], d["name"])
            if dets:
                s = "  ".join("%s%s %.2f @[%.3f,%.3f]"
                              % (d["shown"],
                                 "" if d["shown"] == d["name"]
                                 else "(原始%s)" % d["name"],
                                 d["conf"], d["box"][0], d["box"][1])
                              for d in dets)
                for d in dets:
                    votes[d["shown"]] = votes.get(d["shown"], 0) + 1
            else:
                s = "(什么都没有)"
            print("#%03d %-18s %6.0fms  %s" % (i, tag, dt, s))

            if a.dump:
                vis = im.copy()
                h, w = vis.shape[:2]
                for d in dets:
                    cx, cy, bw, bh = d["box"]
                    p1 = (int((cx - bw / 2) * w), int((cy - bh / 2) * h))
                    p2 = (int((cx + bw / 2) * w), int((cy + bh / 2) * h))
                    cv2.rectangle(vis, p1, p2, (0, 255, 0), 2)
                    cv2.putText(vis, "%s %.2f" % (d["shown"], d["conf"]),
                                (p1[0], max(14, p1[1] - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imwrite(os.path.join(a.dump, "probe_%03d.jpg" % i), vis)
            i += 1
            if a.frames and i >= a.frames:
                break
    except KeyboardInterrupt:
        print("")
    finally:
        y.close()
        if cap:
            cap.release()

    print("---- %d 帧, 票数 %s ----"
          % (i, ", ".join("%s:%d" % kv for kv in sorted(votes.items()))
             or "无"))
    if a.dump:
        print("图在 %s/probe_*.jpg —— 图上标的是**互换后**的结论%s, "
              "和箭头朝向对不上就 yolo_swap_lr:=false" 
              % (a.dump, "(已关闭互换)" if a.no_swap_lr else ""))


if __name__ == "__main__":
    main()
