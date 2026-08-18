#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
lane_common.py — 巡线判决的公共部分 (不依赖 rospy, 离线测试同样使用)
====================================================================
模板约定: 一张 320x192(或任意分辨率, 会缩放)的图, 纯红像素=触发区,
其余不管。以画面竖直中线分左右: 左半红区=左触发区, 右半=右触发区。
触发区是两个**背靠背的直角梯形**: 斜边=标称车道线(向内让开 tol 死区),
竖直边=画面中线, 中间填满。

判决 = 侵入深度(不是像素个数, 见 Decider.decide):
  每一行看该行触发区里**最靠中线**的那个线像素, 它从"斜边(标称线)"
  走到"竖直边(中线)"走了百分之几 —— 这就是该行的侵入深度 ∈[0,1];
  没有线像素的行记 0。I = 所有评估行的平均。
  这样 I 有明确物理含义: 线从标称位置朝中线偏了多少(占半车道的比例),
  而不是"碰到了没有"。浅偏->小 az, 快压中线->满舵。
  (用像素个数会几乎变成开关量: 线一进触发区就是那么粗几个像素,
   深浅几乎不影响计数。)
  az = kp*(I右 - I左)
"""
from __future__ import print_function
import math
import os
import numpy as np
import cv2

from trackseg import IN_W, IN_H


def yellow_mask(bgr, b_min=145, y_min=90):
    """黄胶带掩码。用 **Lab 的 b 通道**, 不用 HSV ——
    b 轴的两端正好就是"蓝 vs 黄"(b<128 偏蓝, b>128 偏黄), 而这个场地恰好
    是蓝垫子 + 黄胶带, 判别方向和物理色差完全对齐, 阈值只要卡在 128 上面
    一点就行。白线和挡板是中性色(b≈128)自动排除, 蓝垫更是往反方向跑。
    关键是**它对曝光不敏感**: 你这两张图明显过曝, HSV 的 S 会被冲淡
    (亮黄接近白), H 也开始飘, 而 b 通道量的是色度差, 过曝下依然分得开。
    y_min 只是再挡一下阴影里的噪点。"""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
    return (lab[:, :, 2] >= b_min) & (lab[:, :, 0] >= y_min)


def yellow_line(bgr, x_frac=0.34, b_min=145, v_min=90,
                row_frac=0.15, min_px=150):
    """在**中央列带**里找那条黄色起跑线, 返回 (行号 or None, 黄像素数)。

    只看中间 x_frac 宽: 两边鱼眼畸变残留大、还容易扫到场地里别的黄色东西,
    而我们要量的是"车正前方那条线离车多远", 中间这段最准。
    行号取**最靠下的那一簇** —— 画面里可能同时看到不止一条黄带, 离车最近
    的(最下面那条)才是要对齐的。"""
    h, w = bgr.shape[:2]
    x0 = int(w * (0.5 - x_frac / 2.0))
    x1 = int(w * (0.5 + x_frac / 2.0))
    m = yellow_mask(bgr[:, x0:x1], b_min, v_min)
    npx = int(m.sum())
    if npx < min_px:
        return None, npx
    cnt = m.sum(axis=1)                       # 每行有几个黄像素
    need = max(3, int(row_frac * (x1 - x0)))
    rows = np.where(cnt >= need)[0]
    if len(rows) < 2:
        return None, npx
    grp = [rows[-1]]                          # 从最下面往上收一簇
    for r in rows[-2::-1]:
        if grp[-1] - r <= 3:
            grp.append(r)
        else:
            break
    return float(np.mean(grp)), npx


def map_az(err, gain=1.2, dead=0.02, az_min=0.12, az_max=0.50):
    """侵入深度误差 -> 角速度。**只有 gain 一个旋钮**, 曲线由它算出来:

        |err| <= dead :  az = 0                     死区, 纯噪声不追
        否则          :  az = az_min + gain*(|err| - dead), 上限 az_max

    gain 就是"每单位侵入深度给多少 rad/s", 大 = 猛。满舵深度可以反算:
        sat = dead + (az_max - az_min)/gain     gain=1.2 -> sat≈0.34

    为什么下端从 az_min 起而不是 0: 底盘 MCU 有旋转死区, 指令低于
    ~0.12 rad/s 电机根本不转(原工程实测 0.073 指令 2.5s 只转 0.004 rad)。
    所以"最慢"物理上就是 az_min; 想更慢只能靠占空比抖动。"""
    a = abs(err)
    if a <= dead:
        return 0.0
    mag = min(az_max, az_min + gain * (a - dead))
    return mag if err > 0 else -mag


def curve_text(gain=1.2, dead=0.02, az_min=0.12, az_max=0.50):
    pts = [0.05, 0.10, 0.20, 0.30, 0.40]
    sat = dead + (az_max - az_min) / max(1e-6, gain)
    return ("gain=%.2f -> 死区%.2f, 满舵深度%.2f;  " % (gain, dead, sat) +
            " ".join("%.2f:%.2f" % (d, map_az(d, gain, dead, az_min, az_max))
                     for d in pts))


def line_on_field(mask, r=10):
    """只保留"上下 r 行内都有场地"的线像素。挡板底边、挡板上的白色箭头条纹
    上方是墙(其他类), 会被剔掉 —— 和 auto_label.drop_wall_base 同一招。
    不做这步的话, 车怼近挡板时那些条纹会被当成一条大横线。"""
    field = (mask == 1).astype(np.uint8)
    line = (mask == 2).astype(np.uint8)
    k = np.ones((r, 1), np.uint8)
    below = cv2.dilate(field, k, anchor=(0, 0))
    above = cv2.dilate(field, k, anchor=(0, r - 1))
    return (line & (below > 0) & (above > 0)).astype(np.uint8)


def goal_block(mask, y_lo=0.78, half=60, r=10, n_side=32, max_deg=35,
               n_samp=48, tol=1, line_cover=0.75):
    """终点框/岔口横线检测。返回
       (列覆盖率, 最佳单线覆盖率, 超阈直线条数, 最近行, ..., 被箱体遮挡列比例)。

    两个互补的证据, 节点侧取与:
      1) 列覆盖率 — 底部条带中央 2*half 列里, 有多少列被线穿过 (0~1)。
         对线的角度完全不敏感, 但**不要求那些列共线**: 墙角处两条车道线
         汇聚也能盖满所有列。
      2) 直线搜索 — 条带左右边各取 n_side 个候选端点, 两两连线(筛掉
         倾角 > max_deg 的、以及中点还没进底部条带的), 沿线采样看被线
         像素覆盖的比例。这个**强制共线**, 正好补上第 1 条的漏洞。
         同时给出"覆盖率 >= line_cover 的候选线有几条" —— 真有一条实线
         时, 附近一堆略微不同的候选都会超阈(实测 12 条); 偶然对齐只会
         有零星几条。

    实测(255 张任意位姿正常帧 + 终点框帧):
      只用列覆盖 >=0.80          误报 0.8%
      只用最佳线 >=0.75          误报 1.2%
      三者与门(0.80/0.75/8条)    误报 0.4%, 且终点框余量很大
    注: 正例只有一帧, 阈值别卡太死; 真正的兜底是连续 goal_confirm 帧。"""
    lof = line_on_field(mask, r)
    # 红绿灯箱体(类3)会**实打实地挡住**它后面那段线 —— Y 岔口那个 LED 箱
    # 就杵在扫描带正中间, 实测能盖掉 ±60 列里的 78%。被挡住的列/采样点
    # 一律算"看不见", 从**分母里去掉**, 而不是算成"没有线":
    # 不这么做的话, 那几十列永远补不上, 覆盖率封顶 0.4, 阈值 0.80 永远
    # 到不了 —— 车就会直接开过 Y 岔口的横线(实车就是这么冲过去的)。
    blk = (mask == 3).astype(np.uint8)
    cx = IN_W // 2
    xL, xR = max(0, cx - half), min(IN_W - 1, cx + half)
    y0 = int(IN_H * y_lo)
    strip = lof[y0:, xL:xR + 1]
    if strip.size == 0:
        return 0.0, 0.0, 0, -1, [], None, [], [], 0.0
    hit = strip.any(axis=0)
    blocked = (~hit) & blk[y0:, xL:xR + 1].any(axis=0)
    blk_frac = float(blocked.mean())
    n_use = int((~blocked).sum())
    # 可用列太少(基本全被箱体糊住)就退回原始口径 —— 宁可不触发, 也不能
    # 拿三五列凑出个 1.00 来
    cover = (float(hit.sum()) / n_use) if n_use >= max(8, 0.15 * hit.size) \
        else float(hit.mean())
    rows = np.where(strip.any(axis=1))[0]
    y_near = int(y0 + rows.max()) if len(rows) else -1

    kd = np.ones((2 * tol + 1, 1), np.uint8)
    lofd = cv2.dilate(lof, kd)                        # 容许±tol行
    blkd = cv2.dilate(blk, kd)
    dx = float(xR - xL)
    ys = np.linspace(IN_H * 0.45, IN_H - 1, n_side)
    YL, YR = np.meshgrid(ys, ys, indexing="ij")
    ok = (np.abs(np.degrees(np.arctan2(YR - YL, dx))) <= max_deg) & \
         (0.5 * (YL + YR) >= y0)
    t = np.linspace(0.0, 1.0, n_samp)
    xi = np.round(xL + dx * t).astype(np.int32)
    yi = np.clip(np.round(YL[..., None] * (1 - t) + YR[..., None] * t)
                 .astype(np.int32), 0, IN_H - 1)
    on = lofd[yi, xi]
    off = blkd[yi, xi] & (1 - on)      # 被箱体挡住且那儿没线 = 不计分母
    den = n_samp - off.sum(axis=2)
    frac = np.where(ok & (den >= 0.25 * n_samp),
                    on.sum(axis=2) / np.maximum(1.0, den.astype(np.float64)),
                    0.0)
    # 过阈的候选线端点, 给 dump 画图用(不参与判决)
    ii, jj = np.nonzero(frac >= line_cover)
    segs = [((xL, int(round(ys[i]))), (xR, int(round(ys[j]))))
            for i, j in zip(ii, jj)]
    k = int(np.argmax(frac))
    bi, bj = k // n_side, k % n_side
    best_seg = ((xL, int(round(ys[bi]))), (xR, int(round(ys[bj]))))
    return (cover, float(frac.max()), int((frac >= line_cover).sum()),
            y_near, segs, best_seg, [(xL, int(round(v))) for v in ys],
            [(xR, int(round(v))) for v in ys], blk_frac)


def _row_span(z):
    """每行 True 区间的 [首列, 末列]; 没有则 (-1,-1)"""
    any_ = z.any(axis=1)
    first = np.argmax(z, axis=1)
    last = z.shape[1] - 1 - np.argmax(z[:, ::-1], axis=1)
    first = np.where(any_, first, -1)
    last = np.where(any_, last, -1)
    return first.astype(np.float32), last.astype(np.float32), any_


def load_template(path):
    """红区模板 -> (左区 bool, 右区 bool), 尺寸 IN_H x IN_W"""
    t = cv2.imread(path)
    assert t is not None, "读不到模板 " + path
    t = cv2.resize(t, (IN_W, IN_H), interpolation=cv2.INTER_NEAREST)
    b, g, r = (t[:, :, 0].astype(int), t[:, :, 1].astype(int),
               t[:, :, 2].astype(int))
    red = (r > 150) & (g < 110) & (b < 110)
    cx = IN_W // 2
    redL = red.copy()
    redL[:, cx:] = False
    redR = red.copy()
    redR[:, :cx] = False
    assert redL.any() and redR.any(), \
        "模板左右半边都要有红区(左%d 右%d)" % (redL.sum(), redR.sum())
    return redL, redR


class Decider(object):
    def __init__(self, redL, redR):
        self.redL, self.redR = redL, redR
        # 每行触发区的两端: 左区 c0=斜边(标称线侧) c1=中线侧; 右区反过来
        self.L0, self.L1, self.rowsL = _row_span(redL)
        self.R0, self.R1, self.rowsR = _row_span(redR)
        self.Lw = np.maximum(1.0, self.L1 - self.L0)
        self.Rw = np.maximum(1.0, self.R1 - self.R0)
        # "已经越过触发区内侧、更靠中线"的区域。梯形模板内侧就是中线,
        # 这块是空的; 但平行四边形(带状)模板内侧离中线还有距离, 车偏太多
        # 时线会整条穿过带子跑到里面去 —— 带里一个像素都没有, 深度反而
        # 掉回 0, 控制器以为一切正常。这块区域一旦有线, 直接按满侵入算。
        cx = IN_W // 2
        xs = np.arange(IN_W)[None, :]
        self.pastL = ((xs > self.L1[:, None]) & (xs <= cx) &
                      self.rowsL[:, None])
        self.pastR = ((xs < self.R0[:, None]) & (xs >= cx) &
                      self.rowsR[:, None])
        self.ker = np.ones((1, 3), np.uint8)     # 横向腐蚀, 杀孤立噪点

    def _depth(self, line, z, c0, c1, w, rows, from_right, past):
        """每行取触发区内最靠中线的线像素, 算它走了多少比例"""
        lz = (line & z).astype(np.uint8)
        lz = cv2.erode(lz, self.ker)             # 1~2px 的孤立噪点不算数
        has = lz.any(axis=1)
        if from_right:                           # 左区: 越靠右越深
            inner = (lz.shape[1] - 1 -
                     np.argmax(lz[:, ::-1], axis=1)).astype(np.float32)
            d = (inner - c0) / w
        else:                                    # 右区: 越靠左越深
            inner = np.argmax(lz, axis=1).astype(np.float32)
            d = (c1 - inner) / w
        d = np.clip(d, 0.0, 1.0)
        d[~has] = 0.0                            # 该行没线 = 没侵入
        d[(line & past).any(axis=1)] = 1.0       # 已穿过带子 = 满侵入
        sel = rows & np.isfinite(d)
        return float(d[sel].mean()) if sel.any() else 0.0

    def decide(self, mask):
        """mask: (192,320) uint8 分割结果 -> (IL, IR, az, nL, nR)"""
        line = mask == 2
        IL = self._depth(line, self.redL, self.L0, self.L1, self.Lw,
                         self.rowsL, True, self.pastL)
        IR = self._depth(line, self.redR, self.R0, self.R1, self.Rw,
                         self.rowsR, False, self.pastR)
        nL = int(np.count_nonzero(line & self.redL))
        nR = int(np.count_nonzero(line & self.redR))
        return IL, IR, IR - IL, nL, nR      # err>0 = 右侧侵入深 -> 要左转

    def action_text(self, az):
        if az < 0:
            return "侵左区->右转"
        if az > 0:
            return "侵右区->左转"
        return "直行"

    def dump(self, img, mask, IL, IR, az, path, scale=3, mirror=None,
             err=None, note=None, goal_y0=None, goal_half=60,
             goal_segs=None, goal_best=None, goal_pts=None):
        """诊断大图: 原图 + 白线标红 + 触发区标绿 + 侵入像素标黄 + 转向箭头。
        关键是右上角那行 mask 统计 —— 一眼分清两种"没反应":
          line=0        网络整幅图都没分出白线(相机没对着跑道/分割挂了)
          line>0 但 in=0  分出线了但没落进触发区(模板位置不对/车离得太远)"""
        H, W = IN_H, IN_W
        vis = cv2.resize(img, (W, H)).astype(np.int32)
        other = mask == 0
        vis[other] = (vis[other] * 0.30).astype(np.int32)       # 场外压暗
        zone = self.redL | self.redR
        vis[zone] = (vis[zone] * 0.55).astype(np.int32) + \
            np.array([0, 120, 0], np.int32)                     # 触发区淡绿
        line = mask == 2
        hit = line & zone
        vis[line] = np.array([0, 0, 255], np.int32)             # 白线 -> 红
        vis[hit] = np.array([0, 255, 255], np.int32)            # 侵入 -> 黄
        vis[mask == 3] = np.array([255, 0, 255], np.int32)      # 红绿灯 -> 品红
        vis = np.clip(vis, 0, 255).astype(np.uint8)
        vis = cv2.resize(vis, (W * scale, H * scale),
                         interpolation=cv2.INTER_NEAREST)

        cnt = np.bincount(mask.ravel(), minlength=4)
        nL = int(np.count_nonzero(line & self.redL))
        nR = int(np.count_nonzero(line & self.redR))
        act = ("TURN RIGHT" if az < 0 else
               ("TURN LEFT" if az > 0 else "STRAIGHT"))
        txt = [
            "depth IL=%.3f  IR=%.3f%s   az=%+.3f rad/s   %s"
            % (IL, IR, ("  err=%+.3f" % err) if err is not None else "",
               az, act),
            "line px in zone: L=%d  R=%d" % (nL, nR),
            "mask px: other=%d field=%d line=%d tl=%d"
            % (cnt[0], cnt[1], cnt[2], cnt[3]),
            ("MIRROR=%s  (barrier text readable => correct)"
             % ("ON" if mirror else "OFF")) if mirror is not None else "",
            ("PHASE: %s" % note) if note else "",
            "red=line  green=trigger zone  yellow=line INSIDE zone  dark=other",
        ]
        txt = [x for x in txt if x]
        for k, t in enumerate(txt):
            y = 22 + k * 24
            cv2.putText(vis, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (0, 0, 0), 4)
            cv2.putText(vis, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (255, 255, 255), 1)
        if goal_y0 is not None:          # 终点检测条带: 这个矩形里数"多少列
            yy = int(goal_y0 * scale)     # 被线穿过", 比例就是触发分数
            cxs = max(0, W // 2 - goal_half) * scale
            cxe = min(W, W // 2 + goal_half) * scale
            cv2.rectangle(vis, (cxs, yy), (cxe - 1, H * scale - 1),
                          (255, 255, 0), 1)
            cv2.putText(vis, "goal scan", (cxs + 4, yy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(vis, "goal scan", (cxs + 4, yy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            if goal_pts:                  # 左右两侧的候选端点(搜索网格)
                for x, y in goal_pts:
                    cv2.circle(vis, (int(x * scale), int(y * scale)), 2,
                               (200, 200, 200), -1)
            if goal_segs:                 # 所有过阈的候选直线(细白)
                ov = vis.copy()
                for (x0, y0), (x1, y1) in goal_segs:
                    cv2.line(ov, (int(x0 * scale), int(y0 * scale)),
                             (int(x1 * scale), int(y1 * scale)),
                             (255, 255, 255), 1)
                cv2.addWeighted(ov, 0.55, vis, 0.45, 0, vis)
            if goal_best:                 # 最佳那条(粗橙)
                (x0, y0), (x1, y1) = goal_best
                cv2.line(vis, (int(x0 * scale), int(y0 * scale)),
                         (int(x1 * scale), int(y1 * scale)), (0, 0, 0), 5)
                cv2.line(vis, (int(x0 * scale), int(y0 * scale)),
                         (int(x1 * scale), int(y1 * scale)), (0, 165, 255), 2)
        # 底部转向箭头
        cy = H * scale - 40
        cx = W * scale // 2
        if az != 0.0:
            d = 90 if az > 0 else -90        # +az=左转, 图像上画向左
            col = (0, 255, 0) if az > 0 else (0, 165, 255)
            cv2.arrowedLine(vis, (cx, cy), (cx - d, cy), (0, 0, 0), 12,
                            tipLength=0.35)
            cv2.arrowedLine(vis, (cx, cy), (cx - d, cy), col, 6,
                            tipLength=0.35)
        else:
            cv2.arrowedLine(vis, (cx, cy + 25), (cx, cy - 45), (0, 0, 0), 12,
                            tipLength=0.35)
            cv2.arrowedLine(vis, (cx, cy + 25), (cx, cy - 45),
                            (255, 255, 255), 6, tipLength=0.35)
        tmp = path + ".tmp.jpg"              # 原子写, 免得看到写一半的图
        cv2.imwrite(tmp, vis, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        os.rename(tmp, path)
        return vis

    def overlay(self, img, mask, IL, IR, az):
        """img: 去畸变图(任意尺寸) -> 叠加图 (红=左区 蓝=右区 黄=检出白线)"""
        vis = cv2.resize(img, (IN_W, IN_H)).copy()
        vis[self.redL] = vis[self.redL] // 2 + np.array([0, 0, 96], np.uint8)
        vis[self.redR] = vis[self.redR] // 2 + np.array([96, 0, 0], np.uint8)
        vis[mask == 2] = (0, 255, 255)
        cv2.putText(vis, "IL=%.2f IR=%.2f az=%+.2f" % (IL, IR, az), (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return vis


# ====================================================================
# 拦路板检测 (2D 激光雷达)
# ====================================================================
# 场地 5m x 2.5m, **四周都是围挡**; 拦路板和车道同宽(42cm), 横着摆在路中间。
# 难点全在"别把围挡当成板子"上。判据只有一条是本质的:
#
#     板子是**有限宽**的 —— 42cm 左右两侧是空地(或明显更远的东西);
#     围挡是**连续**的 —— 正对着看过去横向铺满整个视野。
#
# 所以核心就是量"最近那道障碍在横向上有多宽": 宽度落在 [min_w, max_w]
# 且左右两侧确实更远, 才算板子。这一条不依赖定位、不依赖 TF, 车在弯道上
# 斜着看也成立(斜着看板子只会显得更窄, 不会变宽)。
#
# 另外三道小闸(都在节点侧):
#   - 只在 FOLLOW 相位查(起点那一带正对着围挡, ALIGN/等灯时别查)
#   - 里程走够 board_min_travel 才开始查
#   - 连续 board_confirm 帧都判到才停(单帧噪声不作数)

def scan_xy(ranges, angle_min, angle_inc, range_min, range_max,
            lidar_x=-0.11, lidar_y=0.0, yaw_off=0.0,
            self_half_len=0.171, self_half_w=0.128, self_margin=0.03):
    """LaserScan -> base_link 下的 (x前, y左) 点集。不走 TF: 精简 launch 里
    TF 树可能压根没起全, 而这里只需要一个固定的安装偏移。

    自车剔除: 车上有两根很小的 WiFi 天线立在 footprint 里面, 雷达会扫到
    它们, 打出 0.1~0.3m 的近距离回波。**不能用"最小距离"一刀切** ——
    板子本身也可能停在 0.28m(实测过), 一刀切会把板子一起切掉。所以按
    **位置**剔: 落在车身矩形(半长 0.171 x 半宽 0.128, 实测 footprint)
    再加 self_margin 余量之内的点, 一律当自车结构丢掉。车外面一个点不动。

    self_margin 给 0 就只剔 footprint 内部; 天线要是稍微支出车外, 调大它。
    """
    r = np.asarray(ranges, np.float32)
    n = len(r)
    a = angle_min + angle_inc * np.arange(n, dtype=np.float32) + yaw_off
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    r, a = r[ok], a[ok]
    xs = r * np.cos(a) + lidar_x
    ys = r * np.sin(a) + lidar_y
    if self_half_len > 0.0 and self_half_w > 0.0:
        inside = (np.abs(xs) <= self_half_len + self_margin) & \
                 (np.abs(ys) <= self_half_w + self_margin)
        if inside.any():
            xs, ys = xs[~inside], ys[~inside]
    return xs, ys


def _corner_wall_candidate(xs, ys, nx, ny, max_dist=0.80,
                           sector_half_width=0.45, min_pts=8,
                           min_span=0.12, max_residual=0.025,
                           cluster_gap=0.05):
    """在指定法向的一侧找一条近墙，并返回其局部拟合量。

    ``nx, ny`` 是从车体指向墙的期望法向。停车阶段只看车附近的墙，
    不使用整场地矩形拟合：地图是否准确不会影响这个控制环。按法向
    投影分簇后，墙的投影应近似常数；沿墙方向的长条杂物会因残差过大
    被拒绝。
    """
    X = np.asarray(xs, dtype=np.float64)
    Y = np.asarray(ys, dtype=np.float64)
    if len(X) == 0:
        return None
    d = X * nx + Y * ny
    t = -ny * X + nx * Y
    keep = (d > 0.05) & (d <= max_dist) & \
        (np.abs(t) <= sector_half_width)
    if int(keep.sum()) < min_pts:
        return None
    X, Y, d, t = X[keep], Y[keep], d[keep], t[keep]
    # 不按投影排序后用固定 gap 分簇：墙角另一面墙的投影通常是连续
    # 变化的，会把排序数组接成一整簇。改用一维 RANSAC 式滑窗，寻找
    # 投影近似常数且点数最多的一段。
    best = None
    for center in d:
        group = np.nonzero(np.abs(d - center) <= max_residual)[0]
        if len(group) < min_pts:
            continue
        gd, gt = d[group], t[group]
        distance = float(np.median(gd))
        group = np.nonzero(np.abs(d - distance) <= max_residual)[0]
        if len(group) < min_pts:
            continue
        gd, gt = d[group], t[group]
        residual = float(np.sqrt(np.mean((gd - distance) ** 2)))
        span = float(gt.max() - gt.min())
        if span < min_span or residual > max_residual:
            continue

        px, py = X[group], Y[group]
        center = np.array([px.mean(), py.mean()])
        A = np.column_stack([px - center[0], py - center[1]])
        cov = np.dot(A.T, A)
        values, vectors = np.linalg.eigh(cov)
        tangent = vectors[:, int(np.argmax(values))]
        normal = np.array([-tangent[1], tangent[0]])
        expected = np.array([nx, ny])
        if float(np.dot(normal, expected)) < 0.0:
            normal = -normal
        angle_error = abs(math.atan2(
            normal[0] * expected[1] - normal[1] * expected[0],
            float(np.dot(normal, expected))))
        candidate = {
            "distance": distance,
            "normal": (float(normal[0]), float(normal[1])),
            "residual": residual,
            "span": span,
            "points": int(len(group)),
            "angle_error": angle_error,
        }
        # 点数最多优先，距离只用于同点数时打破平局；否则角落的另一
        # 面墙在投影上可能恰好比真正墙更近。
        score = (-len(group), distance, residual)
        if best is None or score < best[0]:
            best = (score, candidate)
    return None if best is None else best[1]


def corner_wall_fit(xs, ys, max_dist=0.80, sector_half_width=0.45,
                    min_pts=8, min_span=0.12, max_residual=0.025,
                    angle_tol_deg=10.0, cluster_gap=0.05):
    """拟合终点角落的两面近墙。

    返回 ``ok``、``x_wall``、``y_wall`` 和 ``why``。x/y 墙分别从
    ``+x/-x`` 与 ``+y/-y`` 四个方向中选最近的一面，因此不需要知道
    当前地图到底是左下、右下、左上还是右上角。两面墙的符号同时给出
    车体应该前后/左右移动的方向；墙法向还用于小角度航向锁定。
    """
    candidates = {}
    for name, normal in (("x+", (1.0, 0.0)), ("x-", (-1.0, 0.0)),
                         ("y+", (0.0, 1.0)), ("y-", (0.0, -1.0))):
        candidates[name] = _corner_wall_candidate(
            xs, ys, normal[0], normal[1], max_dist=max_dist,
            sector_half_width=sector_half_width, min_pts=min_pts,
            min_span=min_span, max_residual=max_residual,
            cluster_gap=cluster_gap)
    x_options = [(name, value) for name, value in candidates.items()
                 if name.startswith("x") and value is not None]
    y_options = [(name, value) for name, value in candidates.items()
                 if name.startswith("y") and value is not None]
    if not x_options or not y_options:
        return {"ok": False, "x_wall": None, "y_wall": None,
                "why": "两面墙不完整(x=%d,y=%d)" %
                (len(x_options), len(y_options))}
    x_name, x_wall = min(x_options, key=lambda item: item[1]["distance"])
    y_name, y_wall = min(y_options, key=lambda item: item[1]["distance"])
    nx = np.asarray(x_wall["normal"], dtype=np.float64)
    ny = np.asarray(y_wall["normal"], dtype=np.float64)
    orth_error = abs(math.pi / 2.0 - math.acos(
        min(1.0, max(-1.0, abs(float(np.dot(nx, ny)))))))
    if orth_error > math.radians(angle_tol_deg):
        return {"ok": False, "x_wall": x_wall, "y_wall": y_wall,
                "why": "两墙夹角异常 %.1f度" %
                math.degrees(orth_error)}
    x_sign = 1 if x_name == "x+" else -1
    y_sign = 1 if y_name == "y+" else -1
    # 与赛场四个角的约定一致：下方两角朝 -90°，上左朝 0°，
    # 上右朝 180°。这里只用于日志/核对，闭环实际以墙法向为准。
    if x_sign > 0:
        nominal_yaw_deg = -90.0
    elif y_sign > 0:
        nominal_yaw_deg = 0.0
    else:
        nominal_yaw_deg = 180.0
    return {
        "ok": True,
        "x_wall": x_wall,
        "y_wall": y_wall,
        "x_sign": x_sign,
        "y_sign": y_sign,
        "orth_error": orth_error,
        "nominal_yaw_deg": nominal_yaw_deg,
        "why": "x=%s %.3fm y=%s %.3fm" % (
            x_name, x_wall["distance"], y_name, y_wall["distance"]),
    }


def board_detect(xs, ys, lane_half=0.25, x_max=1.60, y_max=0.95,
                 min_w=0.24, max_w=0.80, min_pts=5, center_tol=0.16,
                 gap=0.12, clus_x=3.0, clus_y=2.2, fov_deg=15.0):
    """返回 (是不是板子, 详情 dict)。xs/ys 是 base_link 下的点(米), **按扫描
    角度顺序**排好。

    做法: 先把前方的点按"相邻两点离得远不远"切成若干**连通簇**(标准的 2D
    激光分割), 再挑出挡在车道走廊里、离车最近的那一簇, 量它的横向宽度。

    为什么必须连通分簇, 而不是"取同一纵深的点":
      斜着摆的围挡在固定纵深窗口里只会露出很窄一段(15° 时约 45cm),
      正好落进板子的宽度区间 —— 按纵深取点会把它误判成板子。而连通分簇
      看的是整条障碍: 围挡不管什么角度都连成一长条, 宽度必然超上限。
      反过来, 板子后面 25cm 处的围挡会被切成另一簇, 不会污染板子的宽度。
    """
    info = {"d": -1.0, "w": 0.0, "n": 0, "why": "", "yl": 0.0, "yr": 0.0,
            "nclus": 0, "L": 0.0, "tilt": 0.0}
    xs = np.asarray(xs, np.float64)
    ys = np.asarray(ys, np.float64)
    if len(xs) == 0:
        info["why"] = "无点"
        return False, info
    # 分簇窗口比判决窗口大一圈: 否则围挡会被窗口裁短, 裁成板子的宽度
    win = (xs > 0.05) & (xs < clus_x) & (np.abs(ys) < clus_y)
    if not win.any():
        info["why"] = "视野内无点"
        return False, info
    X, Y = xs[win], ys[win]

    d2 = np.diff(X) ** 2 + np.diff(Y) ** 2      # 相邻扫描点的间距
    cut = np.nonzero(d2 > gap * gap)[0] + 1
    starts = np.concatenate(([0], cut))
    ends = np.concatenate((cut, [len(X)]))
    info["nclus"] = len(starts)

    best = None
    for a, b in zip(starts, ends):
        if b - a < min_pts:
            continue
        cx, cy = X[a:b], Y[a:b]
        # "挡在路中间"要同时满足横向和角度两个窗口:
        #   横向 lane_half —— 近处管用(0.5m 处 ±15° 只有 ±13cm, 太窄)
        #   角度 fov_deg   —— 远处管用(1.2m 处 ±25cm 的横向窗只有 ±12°,
        #                     而围挡的边角很容易横向落进来)
        ang = np.abs(np.arctan2(cy, np.maximum(cx, 1e-6)))
        inlane = (np.abs(cy) < lane_half) & (cx < x_max) & \
            (ang < math.radians(fov_deg))
        if not inlane.any():                    # 没挡在路中间, 不管
            continue
        # ⚠ 距离要量到**整块板子最近的那个点**, 不是只量走廊内那一段。
        # 板子斜着摆时近端那个角会伸出走廊(|y|<lane_half 或 ±fov 之外),
        # 实车 2026-08-16 实测: 走廊内 min x = 0.302, 整块板 min x = 0.250,
        # 差 5.2cm —— 代码以为车头还有 13cm, 实际只剩 8cm, 于是"快擦到
        # 板子才开始横移"。inlane 只用来判"这簇挡没挡住路", 不该参与测距。
        d = float(cx.min())
        if best is None or d < best[0]:
            best = (d, cx, cy, b - a, float(cx[inlane].min()))
    if best is None:
        info["why"] = "走廊内没有成簇的障碍"
        return False, info

    d, cx, cy, n, d_lane = best
    yl, yr = float(cy.min()), float(cy.max())
    w = yr - yl                       # 横向跨度: 决定要横让多远
    # 真实弦长: 沿这一簇自己的主轴量, 不是量它在 y 轴上的投影。
    # 板子斜着摆的时候两者差很多 —— 实车量到 w=0.32 而板子是 0.42, 因为
    # 它斜了约 40 度(cos40 = 0.77)。拿 w 去和 min_w/max_w 比, 斜着的板子
    # 会被判成"太窄的杂物", 斜着的围挡反而可能缩进板子的宽度区间。
    # 所以**分类用弦长 L, 横让距离仍用 w**(要让开的是横向那一段)。
    px, py = cx - cx.mean(), cy - cy.mean()
    if len(px) >= 2:
        cov = np.array([[float(np.dot(px, px)), float(np.dot(px, py))],
                        [float(np.dot(px, py)), float(np.dot(py, py))]])
        ev, evec = np.linalg.eigh(cov)
        u = evec[:, int(np.argmax(ev))]         # 主轴方向
        t = px * u[0] + py * u[1]
        L = float(t.max() - t.min())
        tilt = abs(math.degrees(math.atan2(u[0], u[1])))
        if tilt > 90.0:
            tilt = 180.0 - tilt                 # 相对 y 轴(横向)的倾角
    else:
        L, tilt = w, 0.0
    info.update({"d": d, "d_lane": d_lane, "n": int(n), "w": w,
                 "yl": yl, "yr": yr, "L": L, "tilt": tilt})
    if L > max_w:
        info["why"] = "太长(%.2f>%.2f), 判为围挡" % (L, max_w)
        return False, info
    if L < min_w:
        info["why"] = "太短(%.2f<%.2f), 判为杂物/立柱" % (L, min_w)
        return False, info
    if yl > center_tol or yr < -center_tol:
        info["why"] = "没盖住中线(y %.2f..%.2f)" % (yl, yr)
        return False, info
    info["why"] = "板子"
    return True, info


def cluster_scan(xs, ys, gap=0.12, min_pts=4):
    """把一圈扫描点按相邻间距切成连通簇, 返回索引数组的列表。

    ⚠ 首尾必须接起来: 车正后方正好落在 ±180° 的接缝上, 不接的话绕障
    第三步"板子退到正后方"时, 那一簇会被劈成左右两半, 各自点数不够、
    中点也全错。"""
    n = len(xs)
    if n < min_pts:
        return []
    d2 = (xs[1:] - xs[:-1]) ** 2 + (ys[1:] - ys[:-1]) ** 2
    cut = list(np.nonzero(d2 > gap * gap)[0] + 1)
    parts = [np.arange(a, b) for a, b in zip([0] + cut, cut + [n])]
    wrap = ((xs[0] - xs[-1]) ** 2 + (ys[0] - ys[-1]) ** 2) <= gap * gap
    if wrap and len(parts) > 1:
        parts[0] = np.concatenate([parts[-1], parts[0]])
        parts.pop()
    return [p for p in parts if len(p) >= min_pts]


def seg_stats(px, py):
    """一簇点 -> (中点, 单位方向, 半长, 点数)。方向取 PCA 主轴。"""
    m = np.array([px.mean(), py.mean()])
    A = np.column_stack([px - m[0], py - m[1]])
    C = np.dot(A.T, A) / max(1.0, len(px))
    w, V = np.linalg.eigh(C)
    u = V[:, int(np.argmax(w))]
    t = np.dot(A, u)
    half = float(t.max() - t.min()) / 2.0
    mid = m + u * float(t.max() + t.min()) / 2.0
    return mid, u, half, int(len(px))


def find_seg_near(xs, ys, px, py, gate=0.35, gap=0.12, min_pts=4):
    """找离 (px,py) 最近的那一簇, 返回 seg_stats; 超出 gate 返回 None。
    用"簇里离预测点最近的那个**点**"来比, 不是用中点 —— 斜着看板子时
    只看得见一段, 中点会明显偏, 拿它做关联很容易丢。"""
    best = None
    for idx in cluster_scan(xs, ys, gap, min_pts):
        cx, cy = xs[idx], ys[idx]
        d = float(np.min((cx - px) ** 2 + (cy - py) ** 2)) ** 0.5
        if d <= gate and (best is None or d < best[0]):
            best = (d, cx, cy)
    if best is None:
        return None
    return seg_stats(best[1], best[2])


def wall_fit(xs, ys, W=5.0, H=2.5, tol=0.25, band=0.12, min_pts=15,
             step_deg=0.5, near=8.0):
    """在一圈扫描里拟合"矩形围挡", 返回**按车体方位标好**的四面墙。

    场地是 5x2.5m 的封闭矩形, 雷达在里面同时看得到四面墙 —— 定位里最好
    的一类几何。做法:
      1) 在 [0,90) 里搜朝向 th, 让四面墙轴对齐(判据: 所有点离外接框的
         距离和最小 —— 点都在墙上, 对的 th 会让它最小)
      2) 各边极值附近 band 米内的点就是那面墙, 取**中位数**当墙的位置
         (比 min/max 抗噪声)
      3) ⚠ 关键: **不能**按 u/v 的正负去认"前后左右"。搜索范围只有 90°,
         而矩形转 90° 代价完全一样, 噪声大一点 th 就会跳到 +90°, 于是
         "前墙"标成了侧墙 —— 实测噪声 3cm 时把 0.25m 的前墙报成 4.75m。
         正确做法是给每面墙算出它在**车体系**里的方位角, 谁最接近正前方
         谁才是前墙。这样和 th 落在哪个 90° 象限完全无关。
      4) 拿量出来的长宽和已知场地对一下当**质量闸**: 对不上就返回 None
         (遮挡、把中间的结构当成墙, 都会在这里被拦下)

    返回 dict: th, dims(量出的长宽), walls=[(方位角rad, 距离m, 点数)...],
    ahead=(方位角, 距离, 点数) 或 None, res(RMS 残差)。
    """
    m = (np.abs(xs) < near) & (np.abs(ys) < near)
    X, Y = xs[m], ys[m]
    if len(X) < 4 * min_pts:
        return None
    best, bth = None, 0.0
    for d in np.arange(0.0, 90.0, step_deg):
        t = math.radians(d)
        c, s = math.cos(t), math.sin(t)
        u = X * c + Y * s
        v = -X * s + Y * c
        cost = (np.minimum(u - u.min(), u.max() - u).sum()
                + np.minimum(v - v.min(), v.max() - v).sum())
        if best is None or cost < best:
            best, bth = cost, t
    c, s = math.cos(bth), math.sin(bth)
    u = X * c + Y * s
    v = -X * s + Y * c
    # 每面墙: (在车体系里从车指向该墙的单位方向, 该方向上的距离)
    uhat = (c, s)              # +u 在车体系的方向
    vhat = (-s, c)
    walls, res = [], []
    dims = [None, None]
    for arr, hat, sgn, k in ((u, uhat, +1, 0), (u, uhat, -1, 0),
                             (v, vhat, +1, 1), (v, vhat, -1, 1)):
        e = arr.max() if sgn > 0 else arr.min()
        sel = np.abs(arr - e) < band
        n = int(sel.sum())
        if n < min_pts:
            continue
        d = float(np.median(arr[sel]))
        dirv = (hat[0] * (1 if d > 0 else -1), hat[1] * (1 if d > 0 else -1))
        walls.append((math.atan2(dirv[1], dirv[0]), abs(d), n))
        res.append(arr[sel] - d)
    # 质量闸: 量出来的长宽必须和已知场地对得上
    du = float(u.max() - u.min())
    dv = float(v.max() - v.min())
    dims = (max(du, dv), min(du, dv))
    if abs(dims[0] - max(W, H)) > tol or abs(dims[1] - min(W, H)) > tol:
        return {"th": bth, "dims": dims, "walls": walls, "ahead": None,
                "res": None, "bad": "量出 %.2fx%.2f 与场地 %.2fx%.2f 不符"
                % (dims[0], dims[1], max(W, H), min(W, H))}
    ahead = None
    for a, d, n in walls:                      # 方位角最接近 0 的那面 = 前墙
        if ahead is None or abs(a) < abs(ahead[0]):
            ahead = (a, d, n)
    return {"th": bth, "dims": dims, "walls": walls, "ahead": ahead,
            "res": float(np.sqrt(np.mean(np.concatenate(res) ** 2)))
            if res else None, "bad": None}
