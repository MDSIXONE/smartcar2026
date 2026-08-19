#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Offline regression checks for the final two-wall parking fit."""

from __future__ import print_function

import math
import os
import sys
import unittest

import numpy as np

SCRIPT_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.pardir, "scripts"))
sys.path.insert(0, SCRIPT_DIR)

from lane_common import corner_wall_fit  # noqa: E402


class CornerWallFitTest(unittest.TestCase):

    @staticmethod
    def _corner(x_distance, y_distance, noise=0.0):
        t = np.linspace(-0.35, 0.35, 60)
        xs = np.r_[np.full(t.shape, x_distance), t]
        ys = np.r_[t, np.full(t.shape, y_distance)]
        if noise:
            rng = np.random.RandomState(7)
            xs = xs + rng.normal(0.0, noise, len(xs))
            ys = ys + rng.normal(0.0, noise, len(ys))
        return xs, ys

    def test_bottom_left_corner_reports_minus_90(self):
        fit = corner_wall_fit(*self._corner(0.25, 0.25))
        self.assertTrue(fit["ok"])
        self.assertEqual((fit["x_sign"], fit["y_sign"]), (1, 1))
        self.assertEqual(fit["nominal_yaw_deg"], -90.0)

    def test_wall_behind_is_treated_as_board_not_wall(self):
        """前墙在身后(x=-0.25)不成立: 车是向前开进角落的, 身后只会是板子"""
        fit = corner_wall_fit(*self._corner(-0.25, 0.25))
        self.assertFalse(fit["ok"] and not fit["partial"])
        if fit["ok"]:                        # 只剩侧墙 y=0.25, 它不在前方
            self.fail("身后的线被当成墙了: %s" % fit["why"])
        self.assertIn("身后", fit["why"])

    def test_stable_fit_accepts_small_scan_noise(self):
        fit = corner_wall_fit(*self._corner(0.25, -0.25, noise=0.004))
        self.assertTrue(fit["ok"])
        self.assertAlmostEqual(fit["x_wall"]["distance"], 0.25, delta=0.02)
        self.assertAlmostEqual(fit["y_wall"]["distance"], 0.25, delta=0.02)

    def test_one_wall_far_is_still_reported_for_fallback(self):
        fit = corner_wall_fit(*self._corner(0.25, 1.20), max_dist=1.50)
        self.assertTrue(fit["ok"])
        self.assertGreater(fit["y_wall"]["distance"], 1.0)


if __name__ == "__main__":
    unittest.main()


class CornerFrontWallOnlyTest(unittest.TestCase):
    """只拟合到前墙(侧墙没进视野)也要能用: 按前墙闭环前进, 横向不动。

    以前 x/y 少一面就 ok=False, lane_follow 那边收到就返回零速原地不动,
    一直干等到 30s 超时 —— 前墙明明看得见却不往前走。
    """

    @staticmethod
    def _front_only(x_distance):
        t = np.linspace(-0.35, 0.35, 60)
        return np.full(t.shape, x_distance), t         # 只有 x=常数那面墙

    def test_front_wall_only_is_ok_and_partial(self):
        fit = corner_wall_fit(*self._front_only(0.60))
        self.assertTrue(fit["ok"], fit["why"])
        self.assertTrue(fit["partial"])
        self.assertIsNone(fit["y_wall"])
        self.assertEqual(fit["x_sign"], 1)
        self.assertAlmostEqual(fit["x_wall"]["distance"], 0.60, places=2)

    def test_full_fit_is_not_partial(self):
        t = np.linspace(-0.35, 0.35, 60)
        xs = np.r_[np.full(t.shape, 0.25), t]
        ys = np.r_[t, np.full(t.shape, 0.25)]
        fit = corner_wall_fit(xs, ys)
        self.assertTrue(fit["ok"])
        self.assertFalse(fit["partial"])

    def test_no_front_wall_is_not_ok(self):
        """只有侧墙、没有前墙 -> 闭环无从谈起, 必须 ok=False"""
        t = np.linspace(-0.35, 0.35, 60)
        fit = corner_wall_fit(t, np.full(t.shape, 0.30))
        self.assertFalse(fit["ok"])
        self.assertIn("前墙", fit["why"])


class CornerObliqueAndFourPosesTest(unittest.TestCase):
    """斜着进角落 + 四种进点姿态。

    实车 2026-08-18: "两墙夹角异常 36.4度 / 17.4度"。旧的候选拟合按"沿期望
    法向投影近似常数"取点, 默认墙和车体轴垂直; 车斜着进时那一层会同时切到
    前墙一段和侧墙一段, PCA 把它们连成一条线。现在是 RANSAC + 沿墙连续。
    """

    @staticmethod
    def _corner_pts(dx, dy, sx=1, sy=1, yaw_deg=0.0, noise=0.01, seed=3):
        """两面墙: x 墙在 sx*dx 处(垂直 x 轴), y 墙在 sy*dy 处; 整体绕车心
        转 yaw_deg 模拟车斜着进。返回车体系点云。"""
        t = np.linspace(-0.45, 0.45, 90)
        # x 墙: x=sx*dx, y 从 -0.45..0.45 ; y 墙: y=sy*dy, x 从 0..sx*dx 方向
        xw = np.column_stack([np.full(t.shape, sx * dx), t])
        u = np.linspace(0.0, dx, 90)
        yw = np.column_stack([sx * u, np.full(u.shape, sy * dy)])
        P = np.vstack([xw, yw])
        th = math.radians(yaw_deg)
        R = np.array([[math.cos(th), -math.sin(th)],
                      [math.sin(th), math.cos(th)]])
        P = np.dot(P, R.T)
        rng = np.random.RandomState(seed)
        P = P + rng.normal(0.0, noise, P.shape)
        return P[:, 0], P[:, 1]

    def test_oblique_36deg_still_fits_perpendicular_walls(self):
        xs, ys = self._corner_pts(0.90, 0.30, yaw_deg=36.0)
        fit = corner_wall_fit(xs, ys, max_dist=2.0)
        self.assertTrue(fit["ok"], fit["why"])
        self.assertFalse(fit["partial"])
        # 距离是**垂直**距离, 和转多少度无关
        self.assertAlmostEqual(fit["x_wall"]["distance"], 0.90, delta=0.03)
        self.assertAlmostEqual(fit["y_wall"]["distance"], 0.30, delta=0.03)
        # 两墙法向必须还是垂直的
        nx, ny = np.array(fit["x_wall"]["normal"]), np.array(fit["y_wall"]["normal"])
        self.assertLess(abs(float(np.dot(nx, ny))), math.cos(math.radians(80)))
        # 法向应指向 36° 方向(前墙)
        self.assertAlmostEqual(math.degrees(math.atan2(nx[1], nx[0])), 36.0,
                               delta=4.0)

    def test_oblique_20deg_other_side(self):
        xs, ys = self._corner_pts(0.80, 0.40, sy=-1, yaw_deg=-20.0)
        fit = corner_wall_fit(xs, ys, max_dist=2.0)
        self.assertTrue(fit["ok"], fit["why"])
        self.assertEqual((fit["x_sign"], fit["y_sign"]), (1, -1))
        self.assertAlmostEqual(fit["x_wall"]["distance"], 0.80, delta=0.03)
        self.assertAlmostEqual(fit["y_wall"]["distance"], 0.40, delta=0.03)

    def test_four_entry_poses(self):
        """前左 / 前右 两种进点姿态, 符号必须跟着走(车向前开进角落, 墙不会
        在身后 —— 身后 back_excl 锥内的线按拦路板处理, 不当墙)"""
        for sx, sy in ((1, 1), (1, -1)):
            xs, ys = self._corner_pts(0.60, 0.35, sx=sx, sy=sy, yaw_deg=8.0)
            fit = corner_wall_fit(xs, ys, max_dist=2.0)
            self.assertTrue(fit["ok"], "(%d,%d): %s" % (sx, sy, fit["why"]))
            self.assertEqual((fit["x_sign"], fit["y_sign"]), (sx, sy),
                             "(%d,%d) 符号错" % (sx, sy))
            self.assertAlmostEqual(fit["x_wall"]["distance"], 0.60, delta=0.03)
            self.assertAlmostEqual(fit["y_wall"]["distance"], 0.35, delta=0.03)

    def test_two_segments_cannot_be_glued_into_one_line(self):
        """专治旧 bug: 前墙一小段 + 侧墙一小段, 不许拼成一条 '墙'"""
        from lane_common import _corner_wall_candidate
        # 只给 x+ 扇区喂: 前墙在 x=0.90 只有 y∈[-0.10,0.10] 一小段,
        # 加上侧墙(y=0.30)在 x∈[0.80,0.90] 一小段 —— 旧实现会把两段连起来
        a = np.column_stack([np.full(20, 0.90), np.linspace(-0.10, 0.10, 20)])
        b = np.column_stack([np.linspace(0.80, 0.90, 20), np.full(20, 0.30)])
        P = np.vstack([a, b])
        c = _corner_wall_candidate(P[:, 0], P[:, 1], 1.0, 0.0, max_dist=2.0)
        if c is not None:
            # 要么拟到前墙那一段(法向 ≈ (1,0)), 要么因为太短被拒; 绝不能是斜的
            ang = math.degrees(math.atan2(c["normal"][1], c["normal"][0]))
            self.assertLess(abs(ang), 8.0, "拟出斜法向 %.1f°: 两段被拼起来了" % ang)


class CornerPairFallbackTest(unittest.TestCase):
    """最近的 x/y 配对不垂直时要试别的配对, 而且拒的时候候选要全列出来。

    实车 2026-08-18 第一帧 "两墙夹角异常 42.9度" 只有一个数, 定不了是哪两
    面墙。以后拒帧的 why 里带全部候选(方向/距离/法向/点数)。
    """

    def _pts(self):
        rng = np.random.RandomState(9)
        t = np.linspace(-0.45, 0.45, 90)
        u = np.linspace(0.0, 0.78, 90)
        front = np.column_stack([np.full(90, 0.78), t])
        side = np.column_stack([u, np.full(90, 0.32)])
        # 身后 0.60m 一块斜 40° 的板子: 离得比前墙近, 会被当成"最近 x"
        th = math.radians(40.0)
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        board = np.dot(np.column_stack([np.full(30, -0.60),
                                        np.linspace(-0.21, 0.21, 30)]), R.T)
        P = np.vstack([front, side, board]) + rng.normal(0, 0.008, (210, 2))
        return P[:, 0], P[:, 1]

    def test_falls_back_to_the_perpendicular_pair(self):
        xs, ys = self._pts()
        fit = corner_wall_fit(xs, ys, max_dist=2.0)
        self.assertTrue(fit["ok"], fit["why"])
        # 最近的 x 是身后那块斜板(0.60), 但它和侧墙不垂直, 应该跳过它选前墙
        self.assertEqual(fit["x_sign"], 1)
        self.assertAlmostEqual(fit["x_wall"]["distance"], 0.78, delta=0.03)
        self.assertAlmostEqual(fit["y_wall"]["distance"], 0.32, delta=0.03)

    def test_failure_message_lists_all_candidates(self):
        # 只给一块斜板和一面侧墙, 怎么配都不垂直 -> 必须拒, 且 why 里列候选
        rng = np.random.RandomState(4)
        u = np.linspace(0.0, 0.9, 90)
        side = np.column_stack([u, np.full(90, 0.32)])
        th = math.radians(40.0)
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        board = np.dot(np.column_stack([np.full(40, 0.70),
                                        np.linspace(-0.25, 0.25, 40)]), R.T)
        P = np.vstack([side, board]) + rng.normal(0, 0.008, (130, 2))
        fit = corner_wall_fit(P[:, 0], P[:, 1], max_dist=2.0)
        self.assertFalse(fit["ok"])
        self.assertIn("候选:", fit["why"])
        self.assertIn("法向", fit["why"])


class CornerApexEntryTest(unittest.TestCase):
    """实车 2026-08-18 corner_trace_01: 顶着角尖 45° 进场, 左墙 0.32m 法向
    +58°, 右墙 0.77m 法向 -32°, 身后 0.82m 一块板子(法向 -165°)。旧的按 ±x/±y
    分扇区: x+ 扇区里一条 45° 斜线在两面墙上各切一段(内点 32)压过真右墙(19),
    左墙 +58° 又超出 45° 锥 -> "两墙夹角异常 ~45°" 23 帧全拒, 1.5s 后退回盲推。
    """

    @staticmethod
    def _apex(seed=11):
        rng = np.random.RandomState(seed)
        def wall(d, ang_deg, half=0.9, n=100):
            th = math.radians(ang_deg)
            nm = np.array([math.cos(th), math.sin(th)])
            tg = np.array([-nm[1], nm[0]])
            t = np.linspace(-half, half, n)
            return d * nm + t[:, None] * tg
        left = wall(0.32, 58.0)
        right = wall(0.77, -32.0)
        board = wall(0.82, -165.0, half=0.21, n=22)
        P = np.vstack([left, right, board])
        # 只留雷达真能看见的(离车 <2m), 加 5mm 噪声
        P = P + rng.normal(0, 0.005, P.shape)
        r = np.hypot(P[:, 0], P[:, 1])
        P = P[r < 2.0]
        return P[:, 0], P[:, 1]

    def test_apex_entry_finds_both_walls(self):
        xs, ys = self._apex()
        fit = corner_wall_fit(xs, ys, max_dist=2.0)
        self.assertTrue(fit["ok"], fit["why"])
        self.assertFalse(fit["partial"])
        ds = sorted([fit["x_wall"]["distance"], fit["y_wall"]["distance"]])
        self.assertAlmostEqual(ds[0], 0.32, delta=0.03)
        self.assertAlmostEqual(ds[1], 0.77, delta=0.03)
        # 身后那块板子不能混进来
        for w in (fit["x_wall"], fit["y_wall"]):
            self.assertGreater(w["normal"][0], -0.5, "板子被当成墙: %s" % fit["why"])
        # 航向残差: 转 -32°(或 +58°) 才和墙平行 -> 折到 ±45° 是 -32°
        self.assertAlmostEqual(fit["yaw_err_deg"], -32.0, delta=4.0)

    def test_replay_real_trace_if_present(self):
        """有 corner_trace_01.jsonl 就整段回放, 每帧都得拟合通过"""
        import json
        from lane_common import scan_xy
        for fn in ("/home/claude/corner_trace_01.jsonl",
                   os.path.join(os.path.dirname(__file__), "data", "corner_trace_01.jsonl")):
            if os.path.exists(fn):
                break
        else:
            self.skipTest("没有实车 trace")
        bad = []
        for line in open(fn):
            fr = json.loads(line)
            sc = fr["scan"]
            mm = np.array(sc["mm"], float)
            r = mm / 1000.0
            r[mm == 0] = float("inf")
            xs, ys = scan_xy(r, sc["amin"], sc["ainc"], max(0.05, sc["rmin"]),
                             min(16.0, sc["rmax"]), lidar_x=fr["lidar_x"],
                             yaw_off=fr["yaw_off"], self_margin=0.03)
            fit = corner_wall_fit(xs, ys, max_dist=2.0)
            if not fit["ok"] or fit["partial"]:
                bad.append((fr["elapsed"], fit["why"]))
        self.assertEqual(bad, [], "拒帧: %r" % bad[:3])
