#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import unittest
import xml.etree.ElementTree as ET
from io import open


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))
LAUNCH_PATH = os.path.join(PACKAGE_ROOT, "launch", "ocr_search.launch")
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, "scripts", "ocr_search.py")
ORIGINAL_LAUNCH_PATH = os.path.join(PACKAGE_ROOT, "launch", "2026.launch")


class OcrSearchLaunchTest(unittest.TestCase):
    def test_launch_xml_is_valid_and_uses_independent_node(self):
        root = ET.parse(LAUNCH_PATH).getroot()
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("type"), "ocr_search.py")
        self.assertEqual(nodes[0].get("name"), "ocr_search")

    def test_start_pose_defaults_are_point_3_facing_point_13(self):
        with open(LAUNCH_PATH, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="start_point_number" value="3"', text)
        self.assertIn('name="start_heading_point_number" value="13"', text)

    def test_search_route_and_rotation_speed_are_the_simple_debug_values(self):
        with open(LAUNCH_PATH, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn(
            '[428, 429, 430, 431, 432, 433, 434, 435, 436, 445, 444, 437, 419, 427]',
            text)
        self.assertIn('name="ocr_scan_rotation_speed" value="1.5"', text)
        self.assertIn('name="ocr_scan_positions" value="8"', text)
        self.assertIn('name="ocr_scan_dwell_seconds" value="1.0"', text)
        self.assertIn('name="obstacle_scan_topic" value="/scan_global_obstacles"',
                      text)
        self.assertIn('name="destination_heading_point_number" value="170"',
                      text)

    def test_alignment_is_continuous_for_fifteen_seconds_without_pixel_relaxation(self):
        with open(LAUNCH_PATH, "r", encoding="utf-8") as handle:
            launch_text = handle.read()
        with open(SCRIPT_PATH, "r", encoding="utf-8") as handle:
            script_text = handle.read()
        self.assertIn('name="ocr_alignment_timeout" value="15.0"', launch_text)
        self.assertIn('name="ocr_alignment_tolerance_px" value="30.0"',
                      launch_text)
        self.assertIn("def align_ocr_candidate", script_text)
        self.assertIn("def finish_candidate", script_text)
        self.assertIn("capture_ocr_while_turning", script_text)
        self.assertIn("OCR_SEARCH_OCR_NOT_ALIGNED", script_text)
        self.assertIn("wall-clock budget", script_text)
        self.assertNotIn("ocr_alignment_retry_tolerance_increment_px",
                         launch_text + script_text)
        self.assertNotIn("ocr_alignment_attempts", launch_text + script_text)

    def test_original_provincial_launch_does_not_start_search_node(self):
        with open(ORIGINAL_LAUNCH_PATH, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.assertNotIn("ocr_search.py", text)


if __name__ == "__main__":
    unittest.main()
