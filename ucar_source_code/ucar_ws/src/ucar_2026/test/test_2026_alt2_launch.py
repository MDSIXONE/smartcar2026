#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import unittest
import xml.etree.ElementTree as ET
from io import open


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))
ALT2_LAUNCH_PATH = os.path.join(
    PACKAGE_ROOT, "launch", "2026_alt2.launch")
ALT2_SCRIPT_PATH = os.path.join(
    PACKAGE_ROOT, "scripts", "production_task_2026_alt2.py")
ORIGINAL_LAUNCH_PATH = os.path.join(PACKAGE_ROOT, "launch", "2026.launch")
ORIGINAL_SCRIPT_PATH = os.path.join(
    PACKAGE_ROOT, "scripts", "production_task_2026.py")


class Alt2LaunchTest(unittest.TestCase):
    def test_alt2_launch_runs_alt2_script_with_direct_cmd_vel(self):
        root = ET.parse(ALT2_LAUNCH_PATH).getroot()
        nodes = list(root.iter("node"))
        task_nodes = [
            node for node in nodes if node.get("type", "").endswith(
                "production_task_2026_alt2.py")]
        self.assertEqual(len(task_nodes), 1, "exactly one alt2 task node")
        params = {
            param.get("name"): param.get("value")
            for node in task_nodes for param in node.iter("param")}
        self.assertEqual(params.get("cmd_vel_topic"), "/cmd_vel")
        self.assertEqual(params.get("ocr_alignment_timeout"), "15.0")
        self.assertEqual(params.get("ocr_scan_rotation_speed"), "1.5")
        self.assertEqual(params.get("ocr_scan_positions"), "8")
        self.assertEqual(params.get("ocr_scan_dwell_seconds"), "1.0")
        self.assertIn(".ros/ucar_2026_alt2",
                      params.get("result_directory") or "")

    def test_alt2_script_uses_fixed_heading_scan(self):
        with open(ALT2_SCRIPT_PATH, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn("def scan_ocr_positions", text)
        self.assertIn("ocr_scan_positions", text)
        self.assertIn("ocr_scan_dwell_seconds", text)
        self.assertIn("PRODUCTION_OCR_POSITIONS_START", text)
        self.assertIn("OcrAlignmentTimeout", text)
        self.assertIn("while not rospy.is_shutdown() and time.time() < deadline",
                      text)
        self.assertIn("PRODUCTION_ALT1_OCR_NOT_ALIGNED", text)

    def test_alt2_preserves_wall_range_and_parking_decision(self):
        with open(ALT2_SCRIPT_PATH, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn("wait_for_fresh_front_distance", text)
        self.assertIn("forward_ray_wall_intersection", text)
        self.assertIn("nearest_numbered_point", text)
        self.assertIn("wall_point_number", text)

    def test_original_files_unchanged(self):
        with open(ORIGINAL_LAUNCH_PATH, "r", encoding="utf-8") as handle:
            launch_text = handle.read()
        with open(ORIGINAL_SCRIPT_PATH, "r", encoding="utf-8") as handle:
            script_text = handle.read()
        self.assertNotIn("production_task_2026_alt2", launch_text)
        self.assertNotIn("scan_ocr_positions", script_text)


if __name__ == "__main__":
    unittest.main()