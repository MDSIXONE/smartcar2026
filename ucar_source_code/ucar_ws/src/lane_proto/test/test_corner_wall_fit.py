#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Offline regression checks for the final two-wall parking fit."""

from __future__ import print_function

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

    def test_top_left_and_top_right_nominal_yaw(self):
        left = corner_wall_fit(*self._corner(-0.25, 0.25))
        right = corner_wall_fit(*self._corner(-0.25, -0.25))
        self.assertEqual(left["nominal_yaw_deg"], 0.0)
        self.assertEqual(right["nominal_yaw_deg"], 180.0)

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
