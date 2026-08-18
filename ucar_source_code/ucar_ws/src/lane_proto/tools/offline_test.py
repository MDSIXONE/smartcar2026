#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PC 离线全链路测试(不需要车/ROS): 帧 -> 去畸变 -> 分割 -> 触发区判决
   TRACKSEG_LIB=.../libtrackseg_cpu.so python3 tools/offline_test.py 帧.jpg ...
   与节点用的是同一个 lane_common.Decider, 判决逻辑完全一致。"""
import os, sys
import numpy as np, cv2
here = os.path.dirname(os.path.abspath(__file__))
pkg = os.path.dirname(here)
sys.path.insert(0, os.path.join(pkg, "scripts"))
from trackseg import TrackSeg
from lane_common import load_template, Decider

z = np.load(os.path.join(pkg, "config/maps_640.npz"))
m1, m2 = z["m1"], z["m2"]
dec = Decider(*load_template(os.path.join(pkg, "config/red_template.png")))
seg = TrackSeg(os.environ.get("TRACKSEG_LIB"))
os.makedirs("offline_out", exist_ok=True)
print("backend:", seg.backend)
for p in sys.argv[1:]:
    frame = cv2.imread(p)
    if frame is None:
        print(p, "读不到"); continue
    if frame.shape[:2] != (480, 640):
        frame = cv2.resize(frame, (640, 480))
    und = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
    mask = seg.infer(und)
    IL, IR, az, nL, nR = dec.decide(mask)
    print("%-24s IL=%.2f IR=%.2f (px %4d/%4d) az=%+.2f %s" %
          (os.path.basename(p), IL, IR, nL, nR, az, dec.action_text(az)))
    cv2.imwrite("offline_out/%s_dbg.jpg" % os.path.splitext(
        os.path.basename(p))[0], dec.overlay(und, mask, IL, IR, az))
print("-> offline_out/")
