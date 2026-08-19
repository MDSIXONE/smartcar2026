#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import ast
import io
import json
import os
import unittest


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))
SCRIPT_PATH = os.path.join(
    PACKAGE_ROOT, "scripts", "national_sprint_speed_debug.py")
TASK_SCRIPT_PATH = os.path.join(
    PACKAGE_ROOT, "scripts", "production_task_2026.py")
LAUNCH_PATH = os.path.join(PACKAGE_ROOT, "launch", "2026.launch")
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
        self.assertEqual(points[70], (2.25, 1.75))
        self.assertEqual(
            ((points[67][0] + points[290][0]) / 2.0,
            (points[67][1] + points[290][1]) / 2.0),
            (0.875, 1.75))

    def test_sprint_end_has_independent_arrival_tolerance(self):
        with io.open(LAUNCH_PATH, "r", encoding="utf-8") as handle:
            launch_text = handle.read()
        self.assertIn(
            '<param name="arrival_tolerance" value="0.12"/>',
            launch_text)
        self.assertIn(
            '<param name="sprint_arrival_tolerance" value="0.30"/>',
            launch_text)
        with io.open(TASK_SCRIPT_PATH, "r", encoding="utf-8") as handle:
            task_text = handle.read()
        self.assertIn(
            "arrival_tolerance_override=self.sprint_arrival_tolerance",
            task_text)


if __name__ == "__main__":
    unittest.main()
