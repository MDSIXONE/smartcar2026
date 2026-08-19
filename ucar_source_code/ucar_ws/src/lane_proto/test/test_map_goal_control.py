#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Offline checks for the map-coordinate final parking controller."""

from __future__ import print_function

import math
import json
import os
import sys
import unittest

SCRIPT_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.pardir, "scripts"))
sys.path.insert(0, SCRIPT_DIR)

from lane_common import goal_pose_from_branch, map_error_to_body  # noqa: E402


class MapGoalControlTest(unittest.TestCase):

    POINT_111 = (-2.25, -2.75)
    POINT_120 = (2.25, -2.75)

    def test_left_branch_uses_point_120_and_minus_90(self):
        x, y, yaw, side = goal_pose_from_branch(
            60.0, self.POINT_111, self.POINT_120)
        self.assertEqual((x, y, side), (2.25, -2.75, "left"))
        self.assertAlmostEqual(yaw, math.radians(-90.0))

    def test_right_branch_uses_point_111_and_minus_90(self):
        x, y, yaw, side = goal_pose_from_branch(
            -60.0, self.POINT_111, self.POINT_120)
        self.assertEqual((x, y, side), (-2.25, -2.75, "right"))
        self.assertAlmostEqual(yaw, math.radians(-90.0))

    def test_middle_branch_uses_point_111_and_180(self):
        x, y, yaw, side = goal_pose_from_branch(
            0.0, self.POINT_111, self.POINT_120)
        self.assertEqual((x, y, side), (-2.25, -2.75, "middle"))
        self.assertAlmostEqual(yaw, math.radians(180.0))

    def test_national_grid_contains_requested_goal_points(self):
        grid_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), os.pardir, os.pardir,
            "ucar_2026_national", "config",
            "production_full_grid_all_numbered.json"))
        with open(grid_path, "r") as handle:
            points = {int(item["number"]):
                      (float(item["x_m"]), float(item["y_m"]))
                      for item in json.load(handle)["points"]}
        self.assertEqual(points[111], self.POINT_111)
        self.assertEqual(points[120], self.POINT_120)

    def test_map_error_rotates_into_forward_left_body_axes(self):
        body_x, body_y = map_error_to_body(1.0, 0.0, math.radians(90.0))
        self.assertAlmostEqual(body_x, 0.0, places=6)
        self.assertAlmostEqual(body_y, -1.0, places=6)


if __name__ == "__main__":
    unittest.main()
