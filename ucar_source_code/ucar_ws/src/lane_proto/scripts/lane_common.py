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
    """终点框检测。返回 (列覆盖率, 最佳单线覆盖率, 超阈直线条数, 最近行)。

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
    cx = IN_W // 2
    xL, xR = max(0, cx - half), min(IN_W - 1, cx + half)
    y0 = int(IN_H * y_lo)
    strip = lof[y0:, xL:xR + 1]
    if strip.size == 0:
        return 0.0, 0.0, 0, -1, [], None, [], []
    cover = float(strip.any(axis=0).mean())
    rows = np.where(strip.any(axis=1))[0]
    y_near = int(y0 + rows.max()) if len(rows) else -1

    lofd = cv2.dilate(lof, np.ones((2 * tol + 1, 1), np.uint8))  # 容许±tol行
    dx = float(xR - xL)
    ys = np.linspace(IN_H * 0.45, IN_H - 1, n_side)
    YL, YR = np.meshgrid(ys, ys, indexing="ij")
    ok = (np.abs(np.degrees(np.arctan2(YR - YL, dx))) <= max_deg) & \
         (0.5 * (YL + YR) >= y0)
    t = np.linspace(0.0, 1.0, n_samp)
    xi = np.round(xL + dx * t).astype(np.int32)
    yi = np.clip(np.round(YL[..., None] * (1 - t) + YR[..., None] * t)
                 .astype(np.int32), 0, IN_H - 1)
    frac = np.where(ok, lofd[yi, xi].mean(axis=2), 0.0)
    # 过阈的候选线端点, 给 dump 画图用(不参与判决)
    ii, jj = np.nonzero(frac >= line_cover)
    segs = [((xL, int(round(ys[i]))), (xR, int(round(ys[j]))))
            for i, j in zip(ii, jj)]
    k = int(np.argmax(frac))
    bi, bj = k // n_side, k % n_side
    best_seg = ((xL, int(round(ys[bi]))), (xR, int(round(ys[bj]))))
    return (cover, float(frac.max()), int((frac >= line_cover).sum()),
            y_near, segs, best_seg, [(xL, int(round(v))) for v in ys],
            [(xR, int(round(v))) for v in ys])


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
