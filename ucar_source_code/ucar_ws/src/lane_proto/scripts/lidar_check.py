#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""lidar_check.py — 量出雷达到底装在哪、朝哪

为什么需要它: 代码里 board_lidar_x 目前取 -0.11(雷达在车体中心**后**方
11cm), 这个数是从 ucar_bringup.launch 的静态 TF 和 ucar_2026_visual.urdf
抄来的 —— 而那份 urdf 开头就写着 "RViz-only visual model", 它是照着 TF
画的, 不是独立量出来的。那条 TF 以前还错过一次(注释里写着以前多了
-0.07 rad 的 yaw, 让每帧点云偏了四度)。所以这个数需要实测。

它有多要紧: 绕障第二段的判据是"板心落到车心后方 车半长+禁区 = 0.371m"。
board_lidar_x 差 0.22m(符号弄反), 这一段就会提前 0.22m 收尾, 车尾净空
从 +20cm 变成 -2cm —— 直接蹭上板子。

用法(车静止, 正前方 1~2m 放一面平的墙/板子, 量好**车头保险杠**到它的距离):
    python2 lidar_check.py --front 0.80
不带 --front 也能跑, 只是不给出标定值, 只打各个方向的距离。
"""
from __future__ import print_function
import argparse
import math

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

HALF_LEN = 0.171          # 车半长(实测 footprint)

buf = []


def cb(msg):
    buf.append(msg)


def bearing_range(msg, deg, half=5.0, need=2):
    """取某个方位角附近的中位距离(米); 没有有效点返回 None。

    ⚠ half 别再调回 2.0: 这台 G4 是 4kHz @ 12Hz, 每圈只有约 333 点
    (1.08°/点), ±2° 的窗口里总共才 4 个点, 掉两个就凑不满门限, 明明
    有东西也报"无回波"(第一次实测就是这么误报的)。±5° 里有 ~9 个点。
    """
    r = np.asarray(msg.ranges, np.float32)
    a = msg.angle_min + msg.angle_increment * np.arange(len(r))
    d = np.degrees(np.arctan2(np.sin(a - math.radians(deg)),
                              np.cos(a - math.radians(deg))))
    m = (np.abs(d) <= half) & np.isfinite(r) & (r > msg.range_min) \
        & (r < msg.range_max)
    n = int(m.sum())
    return (float(np.median(r[m])), n) if n >= need else (None, n)


def sweep(msg, step=15.0):
    """整圈扫一遍, 每 step 度一格, 打中位距离和点数 —— 用来一眼看出
    雷达到底看得见什么, 以及有没有被车身自己挡住的扇区。"""
    print("\n整圈概览(每 %.0f° 一格, 括号内是有效点数):" % step)
    line = []
    deg = -180.0
    while deg < 180.0:
        v, n = bearing_range(msg, deg, half=step / 2.0, need=1)
        line.append("%+4.0f:%s" % (deg, ("%.2f(%d)" % (v, n)) if v else "--"))
        if len(line) == 6:
            print("   " + "  ".join(line))
            line = []
        deg += step
    if line:
        print("   " + "  ".join(line))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--front", type=float, default=None,
                    help="车头保险杠到正前方那面墙/板子的实测距离(m)")
    ap.add_argument("--sec", type=float, default=3.0)
    a = ap.parse_args()

    rospy.init_node("lidar_check", anonymous=True)
    rospy.Subscriber(a.topic, LaserScan, cb, queue_size=50)
    print("收 %s %.1f 秒..." % (a.topic, a.sec))
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t0 < a.sec:
        rospy.sleep(0.05)
    if not buf:
        print("!! 一帧都没收到。雷达驱动起了吗? 话题名对吗?")
        return
    msg = buf[len(buf) // 2]
    n = len(msg.ranges)
    print("\n收到 %d 帧, 每帧 %d 点, 角度 %.1f°..%.1f°, 分辨率 %.2f°"
          % (len(buf), n, math.degrees(msg.angle_min),
             math.degrees(msg.angle_min + msg.angle_increment * (n - 1)),
             math.degrees(msg.angle_increment)))

    r = np.asarray(msg.ranges, np.float32)
    ang = msg.angle_min + msg.angle_increment * np.arange(n)
    ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
    print("有效回波 %d/%d 点 (%.0f%%)" % (ok.sum(), n, 100.0 * ok.sum() / n))

    print("\n各方位的距离(雷达自己的坐标系, 0°=angle_min 起算的 0 弧度):")
    for deg in (0, 45, 90, 135, 180, -135, -90, -45):
        v, cnt = bearing_range(msg, deg)
        print("   %+5d°  %s"
              % (deg, "%.3f m (%d 点)" % (v, cnt) if v
                 else "无回波 (窗口内 %d 点)" % cnt))

    sweep(msg)

    if ok.any():
        i = int(np.argmin(np.where(ok, r, 1e9)))
        print("\n最近的回波: %.3f m 在 %+.1f°" % (r[i], math.degrees(ang[i])))

    print("""
怎么判方向:
  把一个明显的东西(比如板子)只放在车的**正前方**, 看上面哪个方位角的距离
  最短。那个角就是"车头方向"在雷达坐标系里的角度, 填进
      board_lidar_yaw_deg := -(那个角)
  比如正前方的东西出现在 180°, 就填 board_lidar_yaw_deg:=180 (或 -180)。
  ⚠ ydlidar 驱动里 reversion=true 会把角度整体转 180°, 所以这一步必须实测,
    不能照抄。""")

    if a.front is not None:
        f0, _ = bearing_range(msg, 0)
        f180, _ = bearing_range(msg, 180)
        print("\n标定 board_lidar_x (你说车头到墙 %.3f m):" % a.front)
        print("   车体中心到墙 = %.3f + 车半长 %.3f = %.3f m"
              % (a.front, HALF_LEN, a.front + HALF_LEN))
        for nm, rr in (("车头在雷达 0°", f0), ("车头在雷达 180°", f180)):
            if rr is None:
                print("   %s: 那个方向没有回波" % nm)
                continue
            print("   %s: 雷达测得 %.3f m -> board_lidar_x = %+.3f m"
                  % (nm, rr, a.front + HALF_LEN - rr))
        print("""
   取那个和实际朝向对得上的一行。正值 = 雷达在车体中心**前**方,
   负值 = 在后方。现在代码里的默认是 -0.110。""")
    else:
        f0, _ = bearing_range(msg, 0)
        if f0:
            print("""
没给 --front, 不能标定 board_lidar_x。正前方现在读到 %.3f m, 那么:
   车头到板子量出来是 %.3f m  -> 雷达在车心**后** 0.110m (=代码默认值)
   车头到板子量出来是 %.3f m  -> 雷达正好在车心
   车头到板子量出来是 %.3f m  -> 雷达在车心**前** 0.110m (符号和默认相反!)
拿尺子量一下车头保险杠到板子的距离, 对上哪一行就知道了; 或者直接
   python2 lidar_check.py --front <量出来的米数>
让它算。""" % (f0, f0 - HALF_LEN - 0.11, f0 - HALF_LEN, f0 - HALF_LEN + 0.11))


if __name__ == "__main__":
    main()
