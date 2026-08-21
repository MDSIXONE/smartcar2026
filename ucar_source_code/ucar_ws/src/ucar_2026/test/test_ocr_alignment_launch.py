#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import unittest
import xml.etree.ElementTree as ET
from io import open


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))
LAUNCH_PATH = os.path.join(
    PACKAGE_ROOT, "launch", "ocr_alignment.launch")
SCRIPT_PATH = os.path.join(
    PACKAGE_ROOT, "scripts", "ocr_alignment.py")
ORIGINAL_LAUNCH_PATH = os.path.join(PACKAGE_ROOT, "launch", "2026.launch")
ORIGINAL_SCRIPT_PATH = os.path.join(
    PACKAGE_ROOT, "scripts", "production_task_2026.py")


class OcrAlignmentLaunchTest(unittest.TestCase):
    def test_launch_xml_uses_only_the_alignment_node(self):
        root = ET.parse(LAUNCH_PATH).getroot()
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("type"), "ocr_alignment.py")
        self.assertEqual(nodes[0].get("name"), "ocr_alignment")

    def test_launch_has_fifteen_second_alignment_and_tts(self):
        with open(LAUNCH_PATH, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="alignment_timeout" value="15.0"', text)
        self.assertIn('name="alignment_tolerance_px" value="30.0"', text)
        self.assertIn('name="tts_enabled" value="true"', text)
        self.assertIn('name="tts_helper_path" value="/home/ucar/wake/tts_say.py"',
                      text)
        self.assertIn('name="ocr_scan_rotation_speed" value="1.5"', text)
        self.assertIn('name="ocr_scan_positions" value="8"', text)
        self.assertIn('name="ocr_scan_dwell_seconds" value="1.0"', text)

    def test_script_continuously_turns_and_announces_completion(self):
        with open(SCRIPT_PATH, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn("def capture_ocr_while_turning", text)
        self.assertIn("def speak_wait", text)
        self.assertIn(u'u"对齐完成"', text)
        self.assertIn('"ALIGNMENT_COMPLETED"', text)
        self.assertIn("alignment_timeout", text)
        self.assertIn("def scan_positions", text)
        self.assertIn("OCR_ALIGNMENT_POSITIONS_START", text)

    def test_original_provincial_files_do_not_reference_alignment_node(self):
        with open(ORIGINAL_LAUNCH_PATH, "r", encoding="utf-8") as handle:
            launch_text = handle.read()
        with open(ORIGINAL_SCRIPT_PATH, "r", encoding="utf-8") as handle:
            script_text = handle.read()
        self.assertNotIn("ocr_alignment.py", launch_text)
        self.assertNotIn("OCR_ALIGNMENT_STATE", script_text)


if __name__ == "__main__":
    unittest.main()
