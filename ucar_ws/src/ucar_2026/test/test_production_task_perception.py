#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))
SCRIPT_ROOT = os.path.join(PACKAGE_ROOT, "scripts")
if SCRIPT_ROOT not in sys.path:
    sys.path.insert(0, SCRIPT_ROOT)

from production_task_perception import (  # noqa: E402
    front_scan_distance,
    horizontal_pixel_error,
    is_navigation_ocr_candidate,
    nearest_numbered_point,
    odom_velocity_is_stopped,
    projected_wall_hit,
    select_alignment_detection,
    select_three_observations,
)


class FakeScan(object):
    angle_min = -0.2
    angle_increment = 0.1
    range_min = 0.05
    range_max = 10.0
    ranges = [float("inf"), 2.2, 2.0, 2.1, float("nan")]


class ProductionTaskPerceptionTest(unittest.TestCase):
    def test_navigation_candidate_requires_text_and_confidence(self):
        response = {
            "ok": True,
            "detection": {"text": "食品加工车间", "confidence": 73.0},
        }
        self.assertTrue(is_navigation_ocr_candidate(response, 60.0))
        self.assertFalse(is_navigation_ocr_candidate(response, 80.0))
        self.assertFalse(is_navigation_ocr_candidate(
            {"ok": True, "detection": None}, 60.0))
        self.assertFalse(is_navigation_ocr_candidate(
            {"ok": True, "detection": {"text": "", "confidence": 99.0}},
            60.0))

    def test_odom_stop_gate_checks_all_planar_axes(self):
        self.assertTrue(odom_velocity_is_stopped(
            (0.01, -0.02, 0.015), 0.02))
        self.assertFalse(odom_velocity_is_stopped(
            (0.01, -0.021, 0.0), 0.02))
        self.assertFalse(odom_velocity_is_stopped(None, 0.02))

    def test_alignment_prefers_confident_near_centre_detection(self):
        detections = [
            {"text": "left", "confidence": 90.0,
             "bbox": (10, 10, 40, 20)},
            {"text": "centre", "confidence": 88.0,
             "bbox": (300, 10, 40, 20)},
        ]
        self.assertEqual(
            select_alignment_detection(detections, 640)["text"],
            "centre")

    def test_horizontal_error_uses_box_and_image_centres(self):
        detection = {"bbox": (350, 20, 40, 30)}
        self.assertEqual(horizontal_pixel_error(detection, 640), 50.0)

    def test_front_scan_uses_median_of_finite_forward_beams(self):
        self.assertAlmostEqual(
            front_scan_distance(FakeScan(), 0.11), 2.1)

    def test_project_and_match_wall_point(self):
        hit = projected_wall_hit((0.0, 0.0, math.pi / 2.0), 1.49)
        match = nearest_numbered_point(
            hit, {297: (0.0, 1.5), 313: (2.25, -0.5)})
        self.assertEqual(match[0], 297)
        self.assertAlmostEqual(match[2], 0.01, places=6)

    def test_three_results_deduplicate_wall_points_by_confidence(self):
        observations = [
            {"wall_point_number": 297, "text": "A", "confidence": 60,
             "wall_match_error_m": 0.02},
            {"wall_point_number": 297, "text": "A", "confidence": 90,
             "wall_match_error_m": 0.03},
            {"wall_point_number": 313, "text": "B", "confidence": 80,
             "wall_match_error_m": 0.01},
            {"wall_point_number": 452, "text": "C", "confidence": 70,
             "wall_match_error_m": 0.01},
            {"wall_point_number": 459, "text": "D", "confidence": 40,
             "wall_match_error_m": 0.01},
        ]
        selected = select_three_observations(observations)
        self.assertEqual(
            [item["wall_point_number"] for item in selected],
            [297, 313, 452])


if __name__ == "__main__":
    unittest.main()
