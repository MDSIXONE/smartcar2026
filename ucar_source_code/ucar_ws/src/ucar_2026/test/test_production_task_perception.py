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
    alignment_angular_speed,
    front_scan_distance,
    forward_ray_wall_intersection,
    horizontal_pixel_error,
    is_navigation_ocr_candidate,
    nearest_numbered_point,
    normalize_production_category,
    odom_velocity_is_stopped,
    projected_wall_hit,
    select_alignment_detection,
    select_three_observations,
    select_three_processing_observations,
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

    def test_mirrored_ocr_error_uses_the_same_post_processed_sign(self):
        self.assertAlmostEqual(
            alignment_angular_speed(-40.0, 0.0025, 0.00035, 0.22, False),
            0.10)
        self.assertAlmostEqual(
            alignment_angular_speed(-40.0, 0.0025, 0.00035, 0.22, True),
            0.10)
        self.assertAlmostEqual(
            alignment_angular_speed(999.0, 0.0025, 0.00035, 0.22, True),
            -0.22)

    def test_derivative_term_brakes_a_fast_error_reversal(self):
        # The post-processed image is mirrored but its bbox error is already
        # in the control convention.  Crossing centre must reverse yaw fast.
        speed = alignment_angular_speed(
            45.0, 0.0025, 0.00035, 0.22, True,
            previous_error_pixels=-176.0, sample_seconds=0.7)
        self.assertLess(speed, 0.0)

    def test_front_scan_uses_median_of_finite_forward_beams(self):
        self.assertAlmostEqual(
            front_scan_distance(FakeScan(), 0.11), 2.1)

    def test_project_and_match_wall_point(self):
        hit = projected_wall_hit((0.0, 0.0, math.pi / 2.0), 1.49)
        match = nearest_numbered_point(
            hit, {297: (0.0, 1.5), 313: (2.25, -0.5)})
        self.assertEqual(match[0], 297)
        self.assertAlmostEqual(match[2], 0.01, places=6)

    def test_production_categories_are_a_strict_three_class_whitelist(self):
        self.assertEqual(normalize_production_category(u"食品加工车间"), u"食品")
        self.assertEqual(normalize_production_category(u"日用品"), u"日用品")
        self.assertEqual(normalize_production_category(u"电子产品加工车间"),
                         u"电子产品")
        self.assertIsNone(normalize_production_category(u"机械加工车间"))

    def test_forward_ray_hits_known_boundary_not_noisy_range_endpoint(self):
        walls = {154: (-2.5, 1.5), 164: (2.5, 1.5),
                 165: (-2.5, -0.5), 175: (2.5, -0.5)}
        distance, hit = forward_ray_wall_intersection(
            (0.0, 0.0, math.pi / 2.0), walls)
        self.assertAlmostEqual(distance, 1.5, places=6)
        self.assertAlmostEqual(hit[0], 0.0, places=6)
        self.assertAlmostEqual(hit[1], 1.5, places=6)

    def test_processing_results_deduplicate_categories(self):
        observations = [
            {"processing_category": u"日用品", "wall_point_number": 297,
             "confidence": 60},
            {"processing_category": u"日用品", "wall_point_number": 298,
             "confidence": 90},
            {"processing_category": u"食品", "wall_point_number": 313,
             "confidence": 80},
            {"processing_category": u"电子产品", "wall_point_number": 452,
             "confidence": 70},
        ]
        selected = select_three_processing_observations(observations)
        self.assertEqual([item["processing_category"] for item in selected],
                         [u"日用品", u"电子产品", u"食品"])
        self.assertEqual(selected[0]["wall_point_number"], 298)

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
