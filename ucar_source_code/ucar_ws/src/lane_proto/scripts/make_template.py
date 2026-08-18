#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_template.py — 从真实"居中直道"帧生成对称的触发区模板
====================================================================
几何依据:
  相机装正(无 yaw/roll)时, 无限长直道的两条边线在去畸变图像里交于
  【主点所在竖直线】上的一点(俯仰只让消失点上下移动, 不左右移动)。
  所以标称线必须**关于主点竖直线严格对称** —— 拟合出的任何左右不对称
  都来自"采样帧里车没摆正", 不是相机, 一律对称化掉(并打印出偏差量,
  偏差大说明该换更正的帧, 或者相机真的装歪了)。

触发区 = 两个**背靠背的直角梯形**(中间填满, 不留空):
  斜边   标称车道线向内让开 tol(默认 8% 半宽) —— 死区, 线在这以外
         摆动不修正, 消抖
  竖直边 画面中线(对称轴), 两个梯形在这里背靠背
  上下边 评估行 ylo..1.0, 只看近场(消失点到底边的下 55%);
         远处一个像素对应好几厘米, 分割抖一下就误触发
  死区按【当前行车道半宽的比例】给是关键: 半宽在图像里近宽远窄, 而
  "横向偏了 2cm"在每一行对应的像素数不同; 按比例给, 各行阈值才代表
  同一个真实横向偏差(约 tol*210mm)。
  填满到中线以后, 判决用的是**侵入深度**(线从斜边走到竖直边的百分比),
  浅偏轻修、快压中线才满舵 —— 见 lane_common.Decider。

用法(PC, 需要 libtrackseg*.so):
    TRACKSEG_LIB=../trackseg_cuda/libtrackseg_cpu.so \
    python3 tools/make_template.py 居中直道帧1.jpg 帧2.jpg ...
    (640x480 原始帧, 内部先去畸变; 3~10 张, 尽量车摆正、正对直道)
    -> config/red_template.png + _preview.jpg

没有合适帧时纯参数生成: python3 tools/make_template.py --param
参数: --tol 0.08 --ylo 0.45 --slope 1.33 --vy 71
"""
import argparse
import os
import sys

import numpy as np
import cv2

here = os.path.dirname(os.path.abspath(__file__))
pkg = os.path.dirname(here)
sys.path.insert(0, os.path.join(pkg, "scripts"))
from trackseg import IN_W, IN_H          # noqa: E402

BLUE = (232, 162, 0)
RED = (36, 28, 237)


def principal_axis(maps_path):
    """去畸变图主点 x, 折算到掩码坐标 —— 对称轴 / 消失点应在的竖直线"""
    z = np.load(maps_path)
    nK, W = z["nK"], int(z["W"])
    return float(nK[0, 2]) * IN_W / W


def fit_side(xs, ys):
    """x = a*y + b, 两轮去离群"""
    a, b = np.polyfit(ys, xs, 1)
    for _ in range(2):
        r = np.abs(xs - (a * ys + b))
        keep = r < max(4.0, 2.5 * r.std())
        if keep.sum() < 10:
            break
        a, b = np.polyfit(ys[keep], xs[keep], 1)
    return float(a), float(b)


def inner_cluster(xs, from_right):
    """一行里最靠画面中央的那一簇线像素 —— 要跟的是车道内边线,
    画面里别的线(邻道、场地其他线)不能混进拟合"""
    if len(xs) < 2:
        return None
    xs = np.sort(xs)
    seq = xs[::-1] if from_right else xs
    c = [seq[0]]
    for v in seq[1:]:
        if abs(v - c[-1]) <= 3:
            c.append(v)
        else:
            break
    return float(np.median(c)) if len(c) >= 2 else None


def lines_from_frames(files, maps_path):
    from trackseg import TrackSeg
    seg = TrackSeg(os.environ.get("TRACKSEG_LIB"))
    z = np.load(maps_path)
    m1, m2 = z["m1"], z["m2"]
    L, R = [], []
    cx = IN_W // 2
    for p in files:
        f = cv2.imread(p)
        assert f is not None, p
        if f.shape[:2] != (480, 640):
            f = cv2.resize(f, (640, 480))
        mask = seg.infer(cv2.remap(f, m1, m2, cv2.INTER_LINEAR))
        line = mask == 2
        for y in range(int(IN_H * 0.35), IN_H):
            xs = np.where(line[y])[0]
            lv = inner_cluster(xs[xs < cx], True)
            rv = inner_cluster(xs[xs >= cx], False)
            if lv is not None:
                L.append((lv, float(y)))
            if rv is not None:
                R.append((rv, float(y)))
    assert len(L) > 30 and len(R) > 30, \
        "线像素太少(L=%d R=%d), 换几张更清晰的居中直道帧" % (len(L), len(R))
    L, R = np.array(L), np.array(R)
    return fit_side(L[:, 0], L[:, 1]), fit_side(R[:, 0], R[:, 1])


def symmetrize(nomL, nomR, axis):
    """把左右两条拟合线关于 axis 对称化: 取"左线"与"右线镜像"的平均。
    返回 (半宽斜率 k, 消失点 y, 原始不对称量)"""
    aL, bL = nomL
    aR, bR = nomR
    k = 0.5 * ((-aL) + aR)                       # 半宽随 y 的增长率
    # 左线: x = axis - k*(y-vy);  右线: x = axis + k*(y-vy)
    vyL = (axis - bL) / aL if aL else 0.0        # 各自与 axis 的交点
    vyR = (axis - bR) / aR if aR else 0.0
    vy = 0.5 * (vyL + vyR)
    vx_raw = (aL * ((bR - bL) / (aL - aR)) + bL)  # 原始交点 x
    return k, vy, vx_raw


def build(axis, k, vy, tol, ylo, out, band=0.0):
    y0 = int(max(0, min(IN_H - 2, vy + ylo * (IN_H - vy))))
    y1 = IN_H
    img = np.zeros((IN_H, IN_W, 3), np.uint8)
    img[:] = BLUE
    ys = np.arange(y0, y1)
    hw = k * (ys - vy)                            # 每行车道半宽(像素)
    for i, y in enumerate(ys):
        h = hw[i]
        if h <= 2:
            continue
        # 左区: 标称线内侧 tol*h 起。band=0 -> 一直填到中线(直角梯形);
        # band>0 -> 只填 band*h 宽(平行四边形)。右区镜像。
        cx = int(round(axis))
        l0 = int(round(axis - h * (1.0 - tol)))
        r1 = int(round(axis + h * (1.0 - tol)))
        if band > 0:
            l1 = min(cx, int(round(axis - h * (1.0 - tol - band))))
            r0 = max(cx, int(round(axis + h * (1.0 - tol - band))))
        else:
            l1, r0 = cx, cx
        if l1 > l0:
            img[y, max(0, l0):min(IN_W, l1)] = RED
        if r1 > r0:
            img[y, max(0, r0):min(IN_W, r1)] = RED
    for s in (-1, 1):                             # 标称线(白, 仅供参考)
        p1 = (int(axis + s * k * (y0 - vy)), y0)
        p2 = (int(axis + s * k * (IN_H - 1 - vy)), IN_H - 1)
        cv2.line(img, p1, p2, (255, 255, 255), 1)
    cv2.imwrite(out, img)
    print("对称轴 x=%.2f  消失点 y=%.1f  半宽斜率 k=%.3f" % (axis, vy, k))
    print("评估行 y=%d..%d  死区 %.0f%% 半宽, %s"
          % (y0, y1, tol * 100,
             ("触发带宽 %.0f%% 半宽(平行四边形)" % (band * 100)) if band > 0
             else "死区以内到中线全部填满(直角梯形)"))
    print("  (420mm 车道: 死区 ≈%.0fmm; 线压到中线 = 侵入深度 1.0 = 满舵)"
          % (tol * 210))
    print("->", out)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames", nargs="*", help="640x480 居中直道原始帧")
    ap.add_argument("--param", action="store_true", help="不用帧, 纯参数生成")
    ap.add_argument("--tol", type=float, default=0.08, help="死区(半宽比例)")
    ap.add_argument("--band", type=float, default=0.0,
                    help="给了就出平行四边形(带状)版: 带宽=该值*半宽; "
                         "0(默认)=直角梯形, 从死区一直填到中线")
    ap.add_argument("--ylo", type=float, default=0.45,
                    help="评估区上界(0=消失点 1=底边)")
    ap.add_argument("--slope", type=float, default=1.33,
                    help="--param 模式: 半宽斜率")
    ap.add_argument("--vy", type=float, default=71.0,
                    help="--param 模式: 消失点行")
    ap.add_argument("--axis", type=float, default=None,
                    help="对称轴 x(默认取去畸变主点; 相机确实装歪才手动给)")
    ap.add_argument("--no-symmetric", action="store_true",
                    help="不对称化, 直接用拟合线(一般不要用)")
    ap.add_argument("--maps", default=os.path.join(pkg,
                                                   "config/maps_640.npz"))
    ap.add_argument("--out", default=os.path.join(pkg,
                                                  "config/red_template.png"))
    a = ap.parse_args()
    assert 0.0 <= a.tol < 0.9, "tol 是半宽比例, 要在 0~0.9"

    axis = a.axis if a.axis is not None else principal_axis(a.maps)
    if a.param or not a.frames:
        print("纯参数模式(建议之后用真实居中帧重新生成)")
        k, vy = a.slope, a.vy
    else:
        nomL, nomR = lines_from_frames(a.frames, a.maps)
        print("拟合: 左 x=%.3f*y%+.1f   右 x=%.3f*y%+.1f"
              % (nomL[0], nomL[1], nomR[0], nomR[1]))
        k, vy, vx_raw = symmetrize(nomL, nomR, axis)
        if a.no_symmetric:
            print("!! 跳过对称化")
        else:
            d = vx_raw - axis
            print("原始消失点 x=%.1f, 对称轴 x=%.1f, 偏 %+.1f px -> 已对称化"
                  % (vx_raw, axis, d))
            if abs(d) > 25:
                print("!! 偏差偏大: 采样帧里车可能没摆正(或相机真的装歪),"
                      " 建议重采更正的帧再跑一次")
    img = build(axis, k, vy, a.tol, a.ylo, a.out, a.band)
    cv2.imwrite(a.out.rsplit(".", 1)[0] + "_preview.jpg",
                cv2.resize(img, (IN_W * 2, IN_H * 2),
                           interpolation=cv2.INTER_NEAREST))


if __name__ == "__main__":
    main()
