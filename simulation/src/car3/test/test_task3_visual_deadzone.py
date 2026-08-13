#!/usr/bin/env python3
"""Regression contract for visual servo commands above the base deadzone."""

from pathlib import Path
import unittest

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
VISION_CONFIG = PACKAGE_DIR / "config" / "task3_vision.yaml"
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"


class Task3VisualDeadzoneTest(unittest.TestCase):
    def test_visual_speed_limits_clear_the_measured_base_deadzone(self):
        config = yaml.safe_load(VISION_CONFIG.read_text(encoding="utf-8"))
        self.assertGreaterEqual(config["vision_min_forward_speed"], 0.15)
        self.assertGreaterEqual(
            config["vision_max_forward_speed"],
            config["vision_min_forward_speed"],
        )
        self.assertGreaterEqual(config["vision_min_lateral_speed"], 0.15)
        self.assertGreaterEqual(
            config["vision_max_lateral_speed"],
            config["vision_min_lateral_speed"],
        )
        self.assertNotIn("vision_min_angular_speed", config)
        self.assertNotIn("vision_max_angular_speed", config)

    def test_visual_servo_uses_configured_deadzone_compensation(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('rospy.get_param("~vision_min_forward_speed"', source)
        self.assertIn('rospy.get_param("~vision_min_lateral_speed"', source)
        self.assertIn(
            "command.linear.y, self.vision_min_lateral", source
        )
        self.assertIn(
            "command.linear.x, self.vision_min_forward", source
        )
        align_body = source[
            source.index("def _vision_align"):
            source.index("def _classify_aligned_cube")
        ]
        self.assertNotIn("angular.z", align_body)
        self.assertIn("command.linear.y", align_body)


if __name__ == "__main__":
    unittest.main()
