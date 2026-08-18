#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yolo_probe.py — 单独验红绿灯识别 (不用 ROS, python2/3 都能跑)
====================================================================
lane_follow 里的认灯是起点那一次(认完就杀), 想反复调角度、换姿势、
验左右有没有反, 用这个 —— 连续认, 不动车, 不用起 ROS。

⚠ 相机是独占的: 跑这个之前先把 roslaunch 停掉, 否则打不开 /dev/ucar_camera。

    cd ~/ucar_ws/src/lane_proto/scripts
    python2 yolo_probe.py                 # 摄像头连续认, Ctrl-C 停
    python2 yolo_probe.py -n 20           # 只跑 20 帧就退
    python2 yolo_probe.py --raw           # 喂不去畸变的帧(对应 yolo_use_raw)
    python2 yolo_probe.py --zoom 2        # 远处的灯太小认不到时放大中间再认
    python2 yolo_probe.py --img a.jpg b.jpg   # 拿现成图片试, 不开相机

每帧打印所有检出, 并把**实际喂给 yolo 的那张图**连框存到
dump/probe_%03d.jpg。左右反没反就看这个图: 图里箭头朝左而标签是
right, 就在 launch 里加 yolo_swap_lr:=true。

退出时打印票数汇总, 以及**认灯频率**(均值/中位/p90、Hz, 还有引擎自己报的
前处理/推理/后处理拆分)。Ctrl-C 中断也会打, 不用非得跑满 -n。
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


def pct(v, q):
    """第 q 百分位(0~100)。py2 没有 statistics, 自己算"""
    if not v:
        return 0.0
    s = sorted(v)
    k = (len(s) - 1) * q / 100.0
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def report(lat, eng, min_votes=2):
    """认灯的频率到底是多少 —— 这里是唯一给数字的地方。

    lat 是 detect_bgr 的整个来回(含把 640x480x3 = 900KB 塞管道的 IPC),
    eng 是引擎自己报的前处理/推理/后处理。两者的差就是 IPC + 进程调度。
    """
    if not lat:
        print("没有有效帧, 不给频率。")
        return
    body = lat[1:] or lat          # 第 0 帧含 CUDA 上下文预热, 单独看
    mean = sum(body) / len(body)
    print("\n---- 认灯频率 ----")
    print("  首帧(含预热) %.0f ms" % lat[0])
    print("  之后 %d 帧: 均值 %.0f ms  中位 %.0f  p90 %.0f  最快 %.0f  最慢 %.0f"
          % (len(body), mean, pct(body, 50), pct(body, 90),
             min(body), max(body)))
    print("  => %.2f 帧/秒 (%.2f Hz)" % (1000.0 / mean, 1000.0 / mean))
    if eng:
        e = eng[1:] or eng
        p = sum(x[0] for x in e) / len(e)
        f = sum(x[1] for x in e) / len(e)
        q = sum(x[2] for x in e) / len(e)
        print("  引擎内部均值: 前处理 %.0f  推理 %.0f  后处理 %.0f  "
              "= %.0f ms" % (p, f, q, p + f + q))
        print("  管道 IPC + 调度: %.0f ms (%.0f%%)"
              % (mean - (p + f + q),
                 100.0 * (mean - (p + f + q)) / mean if mean else 0))
        if f > 0.6 * mean:
            print("  -> 瓶颈在**推理内核**本身, 不是传图。要快只能动 "
                  "kernels.cu / 降输入分辨率 / 确认 nvpmodel -m 0 + "
                  "jetson_clocks 已经开。")
        elif mean - (p + f + q) > 0.4 * mean:
            print("  -> 瓶颈在**进程间传图**, 不是算。考虑送 JPEG 而不是 "
                  "裸 BGR, 或者别每帧都送。")
    else:
        print("  (引擎没报 ms —— configure(timings=True) 没生效?)")
    print("  实车影响: lane_follow 的 yolo_min_votes=%d, 也就是至少 "
          "%.1f 秒才能定方向(还没算漏检重来的帧); 到 yolo_wait_max"
          "(默认 60s) 就按最高票/fallback 走。" % (min_votes,
                                                min_votes * mean / 1000.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img", nargs="*", default=None,
                    help="给了就认这些图片, 不开相机")
    ap.add_argument("--device", default="/dev/ucar_camera")
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
    ap.add_argument("--min-votes", type=int, default=2,
                    help="只影响最后那行'至少要几秒才能定方向'的估算, "
                         "和节点的 yolo_min_votes 对齐")
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
    lat = []          # 每帧 detect_bgr 的整个来回(毫秒), 含 IPC
    eng = []          # 引擎自己报的 (pre, infer, post) 毫秒
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
            lat.append(dt)
            ms = r.get("ms") or {}      # configure(timings=True) 时引擎会带上
            if ms:
                eng.append((float(ms.get("pre", 0.0)),
                            float(ms.get("infer", 0.0)),
                            float(ms.get("post", 0.0))))
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
            br = ""
            if ms:
                br = "(前%.0f 推%.0f 后%.0f)" % (ms.get("pre", 0),
                                               ms.get("infer", 0),
                                               ms.get("post", 0))
            print("#%03d %-18s %6.0fms %-22s %s" % (i, tag, dt, br, s))

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
    report(lat, eng, min_votes=a.min_votes)
    if a.dump:
        print("图在 %s/probe_*.jpg —— 图上标的是**互换后**的结论%s, "
              "和箭头朝向对不上就 yolo_swap_lr:=false" 
              % (a.dump, "(已关闭互换)" if a.no_swap_lr else ""))


if __name__ == "__main__":
    main()
