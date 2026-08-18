#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import ast
import json
import os
import unittest


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))
SCRIPT_PATH = os.path.join(
    PACKAGE_ROOT, "scripts", "national_sprint_speed_debug.py")
GRID_PATH = os.path.join(
    PACKAGE_ROOT, "config", "production_full_grid_all_numbered.json")


class NationalSprintSpeedDebugTest(unittest.TestCase):
    def test_debug_program_is_python2_parseable(self):
        with open(SCRIPT_PATH, "rb") as handle:
            ast.parse(handle.read(), filename=SCRIPT_PATH)

    def test_start_and_slope_top_points_exist_in_national_grid(self):
        with open(GRID_PATH, "rb") as handle:
            points = dict(
                (int(item["number"]), (item["x_m"], item["y_m"]))
                for item in json.load(handle)["points"])
        self.assertIn(70, points)
        self.assertEqual(
            ((points[67][0] + points[290][0]) / 2.0,
             (points[67][1] + points[290][1]) / 2.0),
            (0.875, 1.75))


if __name__ == "__main__":
    unittest.main()
