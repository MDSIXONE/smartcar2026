#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import json
import os
import sys
import threading
import time
import unittest
from collections import deque


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))
SCRIPT_ROOT = os.path.join(PACKAGE_ROOT, "scripts")
if SCRIPT_ROOT not in sys.path:
    sys.path.insert(0, SCRIPT_ROOT)

from production_task_geometry import (  # noqa: E402
    DEFAULT_QR_OBSERVATION_NUMBERS,
    DEFAULT_FALLBACK_PRODUCTION_OBSERVATION_HEADINGS_DEG,
    DEFAULT_FALLBACK_PRODUCTION_ROUTE,
    DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG,
    DEFAULT_PRODUCTION_ROUTE,
    DEFAULT_PRODUCTION_ROUTE_GROUPS,
    TaskDefinitionError,
    bearing,
    build_straight_segments,
    load_middle_target_guard_points,
    load_middle_zone_geometry,
    load_numbered_points,
    load_wall_reference_points,
    needs_recenter,
    normalize_angle,
    normalize_production_route_groups,
    position_error,
    positive_turn_increment,
    require_points,
    shortest_yaw_delta,
    stop_point_for_measured_wall_hit,
    stop_point_for_wall_point,
)
from production_task_perception import target_guard_scan_matches  # noqa: E402

try:
    import production_task_2026 as task_module  # noqa: E402
except ImportError:
    task_module = None


class ProductionTaskGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid_path = os.path.join(
            PACKAGE_ROOT, "config",
            "production_full_grid_all_numbered.json")
        cls.points = load_numbered_points(cls.grid_path)

    def assertAngleAlmostEqual(self, actual, expected):
        self.assertAlmostEqual(
            normalize_angle(actual - expected), 0.0, places=6)

    def test_requested_points_have_expected_coordinates(self):
        expected = {
            41: (-2.25, 2.75),
            43: (-1.25, 2.75),
            52: (-1.75, 2.25),
            61: (-2.25, 1.75),
            262: (-2.50, 2.25),
            232: (-1.75, 3.00),
            295: (-1.75, 1.50),
            12: (-1.75, 0.75),
            24: (-0.75, 0.25),
            16: (0.25, 0.75),
            28: (1.25, 0.25),
            19: (1.75, 0.75),
            297: (-0.75, 1.50),
            419: (-2.00, 1.00),
            427: (2.00, 1.00),
            428: (-2.00, 0.50),
            436: (2.00, 0.50),
            437: (-2.00, 0.00),
            445: (2.00, 0.00),
            452: (-2.50, 1.25),
            459: (2.50, -0.25),
        }
        self.assertEqual(
            require_points(self.points, expected.keys()), expected)

    def test_qr_observation_headings_are_from_staging_point_52(self):
        staging = self.points[52]
        self.assertEqual(
            DEFAULT_QR_OBSERVATION_NUMBERS,
            [262, 232, 295, 61, 41, 43])
        expected_headings = [
            math.pi, math.pi / 2.0, -math.pi / 2.0,
            -3.0 * math.pi / 4.0, 3.0 * math.pi / 4.0,
            math.pi / 4.0,
        ]
        for number, expected in zip(
                DEFAULT_QR_OBSERVATION_NUMBERS, expected_headings):
            self.assertAngleAlmostEqual(
                bearing(staging, self.points[number]), expected)

    def test_middle_completion_counts_and_wall_references(self):
        with open(self.grid_path, "r") as handle:
            document = json.load(handle)
        self.assertEqual(len(document["points"]), 459)
        self.assertEqual(document["counts"]["middle_line_endpoints"], 27)
        wall_points = load_wall_reference_points(self.grid_path)
        self.assertEqual(len(wall_points), 56)
        for number in (154, 158, 164, 165, 175, 294, 297, 303,
                       304, 313, 446, 451, 452, 459):
            self.assertIn(number, wall_points)

    def test_production_route_and_observation_headings_are_exact(self):
        self.assertEqual(
            DEFAULT_PRODUCTION_ROUTE,
            [11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
             30, 29, 28, 27, 26, 25, 24, 23, 22, 21])
        self.assertEqual(
            DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG,
            [-45, 45] * 10)
        # One heading per navigation leg (staging leg + route legs).
        self.assertEqual(
            len(DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG),
            len(DEFAULT_PRODUCTION_ROUTE))
        self.assertEqual(
            DEFAULT_PRODUCTION_ROUTE_GROUPS,
            [[11, 12, 21, 22], [13, 14, 23, 24],
             [15, 16, 25, 26], [17, 18, 27, 28],
             [19, 20, 29, 30]])
        self.assertEqual(
            normalize_production_route_groups(
                DEFAULT_PRODUCTION_ROUTE_GROUPS, DEFAULT_PRODUCTION_ROUTE),
            DEFAULT_PRODUCTION_ROUTE_GROUPS)
        self.assertEqual(
            DEFAULT_FALLBACK_PRODUCTION_ROUTE,
            list(range(1, 11)) + [20, 30, 40] +
            list(range(39, 30, -1)) + [21, 11])
        self.assertEqual(
            DEFAULT_FALLBACK_PRODUCTION_OBSERVATION_HEADINGS_DEG,
            [-90] * 10 + [180] * 3 + [90] * 9 + [0] * 2)
        self.assertEqual(
            len(DEFAULT_FALLBACK_PRODUCTION_OBSERVATION_HEADINGS_DEG),
            len(DEFAULT_FALLBACK_PRODUCTION_ROUTE))

    def test_production_route_collapses_to_exact_straight_segments(self):
        self.assertEqual(
            build_straight_segments(
                DEFAULT_PRODUCTION_ROUTE, self.points,
                math.radians(1.0)),
            [(11, 20), (20, 30), (30, 21)])

    def test_middle_target_guard_mapping_and_filtered_scan_match(self):
        guards = load_middle_target_guard_points(
            self.grid_path, DEFAULT_PRODUCTION_ROUTE)
        self.assertEqual(
            dict((number, sorted(points))
                 for number, points in guards.items()),
            {
                11: [419, 428, 446, 448],
                12: [419, 420, 428, 429],
                22: [428, 429, 437, 438],
                13: [420, 421, 429, 430],
                23: [429, 430, 438, 439],
                14: [421, 422, 430, 431],
                24: [430, 431, 439, 440],
                15: [422, 423, 431, 432],
                25: [431, 432, 440, 441],
                16: [423, 424, 432, 433],
                26: [432, 433, 441, 442],
                17: [424, 425, 433, 434],
                27: [433, 434, 442, 443],
                18: [425, 426, 434, 435],
                28: [434, 435, 443, 444],
                19: [426, 427, 435, 436],
                20: [427, 436, 447, 449],
                21: [428, 437, 448, 450],
                29: [435, 436, 444, 445],
                30: [436, 445, 449, 451],
            })

        class Scan(object):
            angle_min = 0.0
            angle_increment = math.pi / 2.0
            range_min = 0.05
            range_max = 5.0
            ranges = [0.20, float("inf")]

        matches = target_guard_scan_matches(
            Scan(), (-2.20, 1.00, 0.0), guards[12], 0.08)
        self.assertEqual(matches, {419: 0.0})

    def test_straight_segments_break_on_reverse_leg_and_turn(self):
        reverse_points = {
            1: (0.0, 0.0),
            2: (1.0, 0.0),
            3: (0.25, 0.0),
        }
        self.assertEqual(
            build_straight_segments([1, 2, 3], reverse_points),
            [(1, 2), (2, 3)])

        turning_points = {
            1: (0.0, 0.0),
            2: (1.0, 0.0),
            3: (1.0, 1.0),
        }
        self.assertEqual(
            build_straight_segments([1, 2, 3], turning_points),
            [(1, 2), (2, 3)])

    def test_straight_segments_reject_duplicate_route_point(self):
        points = {
            1: (0.0, 0.0),
            2: (1.0, 0.0),
        }
        with self.assertRaises(TaskDefinitionError):
            build_straight_segments([1, 2, 2], points)

        duplicate_coordinate_points = {
            1: (0.0, 0.0),
            2: (1.0, 0.0),
            3: (0.0, 0.0),
        }
        with self.assertRaises(TaskDefinitionError):
            build_straight_segments(
                [1, 2, 3], duplicate_coordinate_points)

    def test_positive_turn_progress_crosses_pi_boundary(self):
        first = positive_turn_increment(
            math.radians(179.0), math.radians(-179.0), 1.0)
        second = positive_turn_increment(
            math.radians(-179.0), math.radians(-170.0), 1.0)
        self.assertAlmostEqual(
            first + second, math.radians(11.0), places=6)

    def test_shortest_yaw_delta_prefers_clockwise_negative_transition(self):
        self.assertAlmostEqual(
            shortest_yaw_delta(
                math.radians(-90.0), math.radians(-135.0)),
            math.radians(-45.0), places=6)

    def test_reverse_jitter_does_not_increase_turn_progress(self):
        self.assertEqual(
            positive_turn_increment(0.5, 0.49, 1.0), 0.0)

    def test_post_turn_position_error_uses_planar_distance(self):
        self.assertAlmostEqual(
            position_error((0.18, 0.826), (0.25, 0.75)),
            math.hypot(0.07, -0.076),
            places=9)

    def test_post_turn_recenter_triggers_only_above_tolerance(self):
        self.assertFalse(needs_recenter(0.060, 0.060))
        self.assertTrue(needs_recenter(0.061, 0.060))
        self.assertTrue(needs_recenter(0.102, 0.060))

    def test_middle_zone_geometry_matches_grid_document(self):
        x_min, x_max, y_min, y_max, side = load_middle_zone_geometry(
            self.grid_path)
        self.assertEqual(
            (x_min, x_max, y_min, y_max, side),
            (-2.5, 2.5, -0.5, 1.5, 0.5))

    def test_stop_point_for_wall_point_uses_explicit_25cm_offset(self):
        bounds = (-2.5, 2.5, -0.5, 1.5)
        cases = [
            # wall intersection -> processing-area stop point:
            ((0.75, 1.5), (0.75, 1.25)),    # 300 -> point 7
            ((2.5, 0.75), (2.25, 0.75)),    # 455 -> point 20
            ((-2.5, 0.75), (-2.25, 0.75)),  # 454 -> point 11
            ((-2.5, 0.5), (-2.25, 0.5)),    # 448 -> midpoint of 11 and 21
            ((-0.75, -0.5), (-0.75, -0.25)),  # 307 -> point 34
        ]
        for wall_point, expected_stop in cases:
            actual_stop = stop_point_for_wall_point(
                wall_point, 0.25, bounds)
            self.assertAlmostEqual(actual_stop[0], expected_stop[0], places=9)
            self.assertAlmostEqual(actual_stop[1], expected_stop[1], places=9)

    def test_stop_point_for_wall_point_rejects_off_boundary_point(self):
        bounds = (-2.5, 2.5, -0.5, 1.5)
        with self.assertRaises(TaskDefinitionError):
            stop_point_for_wall_point((0.0, 0.0), 0.25, bounds)
        with self.assertRaises(TaskDefinitionError):
            stop_point_for_wall_point((0.75, 0.75), 0.25, bounds)
        with self.assertRaises(TaskDefinitionError):
            stop_point_for_wall_point((-2.5, 2.5), 0.25, bounds)

    def test_stop_point_for_measured_wall_hit_uses_measured_along_wall_position(self):
        bounds = (-2.5, 2.5, -0.5, 1.5)
        actual_stop = stop_point_for_measured_wall_hit(
            (0.82, 1.48), (0.75, 1.50), 0.25, bounds)
        self.assertAlmostEqual(actual_stop[0], 0.82, places=9)
        self.assertAlmostEqual(actual_stop[1], 1.23, places=9)


@unittest.skipIf(
    task_module is None,
    "ROS Python modules are only available in the vehicle workspace")
class ProductionTaskRecenteringPolicyTest(unittest.TestCase):
    def setUp(self):
        task_module.rospy.rostime.set_rostime_initialized(True)
        self.task = object.__new__(task_module.ProductionTask2026)
        self.task.points = {16: (0.25, 0.75)}
        self.task.arrival_tolerance = 0.10
        self.task.current_map_pose = lambda _context: (0.18, 0.826, 0.0)
        self.warnings = []
        self.original_logwarn = task_module.rospy.logwarn
        self.original_loginfo = task_module.rospy.loginfo
        task_module.rospy.logwarn = self.capture_warning
        task_module.rospy.loginfo = lambda *_args: None
        self.task.qr_classifications = []
        self.task.api_events = deque()
        self.task.first_qr_item_by_code = {}
        self.task.observations = []
        self.task.expected_item_text = u""
        self.task.expected_production_category = None
        self.task.expected_real_item_text = u""
        self.task.expected_sim_item_text = u""
        self.task.expected_real_category = None
        self.task.expected_sim_category = None
        self.task.served_wall_points = set()
        self.task._ocr_turn_stop_flag = False
        self.task.processing_dwell_seconds = 0.0
        self.task.middle_zone_square_side = 0.5
        self.task.ocr_stop_offset_m = 0.25
        self.task.middle_zone_bounds = (-2.5, 2.5, -0.5, 1.5)
        self.task.wall_match_max_error = 0.18
        self.task.ocr_alignment_min_speed = 0.12
        self.task.spark_classify_enabled = False
        self.task.tts_enabled = False

    def tearDown(self):
        task_module.rospy.logwarn = self.original_logwarn
        task_module.rospy.loginfo = self.original_loginfo

    def capture_warning(self, message, *args):
        self.warnings.append(message % args)

    def test_same_position_navigation_uses_shortest_measured_rotation(self):
        calls = []
        self.task.current_map_pose = (
            lambda _context: (-1.75, 2.25, math.radians(-90.0)))
        self.task.require_safe = lambda: None
        self.task.rotate_in_place_to_yaw = (
            lambda yaw, context: calls.append((yaw, context)))

        result = self.task.navigate_coordinates(
            -1.75, 2.25, math.radians(-135.0),
            "QR face point 61", require_plan=False)

        self.assertTrue(result)
        self.assertEqual(calls[0][0], math.radians(-135.0))
        self.assertIn("shortest same-position rotation", calls[0][1])

    def test_same_position_navigation_uses_arrival_tolerance_for_rotation(self):
        calls = []
        self.task.current_map_pose = (
            lambda _context: (-1.75 + 0.068, 2.25, math.radians(90.0)))
        self.task.require_safe = lambda: None
        self.task.rotate_in_place_to_yaw = (
            lambda yaw, context: calls.append((yaw, context)))

        result = self.task.navigate_coordinates(
            -1.75, 2.25, math.radians(-90.0),
            "QR face point 295", require_plan=False)

        self.assertTrue(result)
        self.assertEqual(calls[0][0], math.radians(-90.0))
        self.assertIn("shortest same-position rotation", calls[0][1])

    def test_target_guard_fallback_tries_next_candidate_after_navigation_failure(self):
        calls = []
        self.task.target_guard_points = {
            11: {428: (-2.0, 0.5), 445: (2.0, 0.0),
                 446: (-2.5, 1.0)}}
        self.task.production_route_numbers = [11, 12]
        self.task.grouped_route_attempt_index = 0

        def navigate(*_args, **kwargs):
            candidate = kwargs["navigation_point_number"]
            calls.append((candidate, kwargs["fallback_navigation"]))
            if len(calls) == 1:
                return "target_navigation_failed"
            self.task.last_target_guard_fallback_candidates = []
            return "target_guard_skipped"

        self.task.navigate_target_and_scan = navigate
        outcome = self.task.try_grouped_target_guard_fallback(
            "forward", 0, 11, 3, math.radians(-45.0), u"日用品", None,
            12, [428, 446, 445])

        self.assertEqual(outcome, "target_guard_skipped")
        self.assertEqual(calls, [(428, True), (445, True)])

    def test_target_guard_fallback_excludes_wall_reference_points(self):
        wall_points = dict(
            (number, (-2.5, 0.0))
            for number in (446, 447, 448, 449, 450, 451))
        guard_points = {428: (-2.0, 0.5), 445: (2.0, 0.0)}
        guard_points.update(wall_points)
        monitor = {"guard_points": guard_points, "hit_counts": {}}

        self.assertEqual(
            self.task.target_guard_fallback_candidates(11, monitor),
            [428, 445])

    def test_target_guard_fallback_uses_short_timeout_and_returns_failure(self):
        navigation = []
        self.task.points = {428: (-2.0, 0.5)}
        self.task.target_guard_fallback_timeout = 25.0
        self.task.publish_state = lambda _state: None
        self.task.new_target_guard_monitor = (
            lambda *_args: {"guard_points": {}, "hit_counts": {}})
        self.task.wait_for_target_guard_precheck = lambda _monitor: None
        self.task.poll_target_guard = lambda _monitor: None
        self.task.target_guard_scan_expired = lambda _monitor: False

        def navigate(*_args, **kwargs):
            navigation.append(kwargs)
            return False

        self.task.navigate_coordinates = navigate
        self.task.navigate_target_and_scan(
            1, 3, 11, math.radians(-45.0),
            route_leg_count=1,
            navigation_point_number=428,
            guard_points_override={}, fallback_navigation=True)

        self.assertEqual(len(navigation), 1)
        self.assertFalse(navigation[0]["require_plan"])
        self.assertFalse(navigation[0]["abort_on_navigation_failure"])
        self.assertEqual(navigation[0]["goal_timeout"], 25.0)

    def test_processing_parking_profile_keeps_point_mode_and_restores_inflation(self):
        messages = []

        class NavigationModePublisher(object):
            def get_num_connections(self):
                return 1

            def publish(self, message):
                messages.append(message.data)

        inflation_state = {
            "/move_base/local_costmap/inflation_layer": 0.224,
            "/move_base/global_costmap/inflation_layer": 0.224,
        }

        class InflationClient(object):

            def __init__(self, namespace, timeout=None):
                self.namespace = namespace
                self.timeout = timeout

            def get_configuration(self):
                return {"inflation_radius": inflation_state[self.namespace]}

            def update_configuration(self, configuration):
                inflation_state[self.namespace] = float(
                    configuration["inflation_radius"])
                return {"inflation_radius": inflation_state[self.namespace]}

        self.task.publish_state = lambda _state: None
        self.task.require_safe = lambda: None
        self.task.navigation_mode_pub = NavigationModePublisher()
        self.task.navigation_mode_connect_timeout = 0.5
        self.task.local_costmap_layer_control_enabled = True
        self.task.local_costmap_inflation_layer = (
            "/move_base/local_costmap/inflation_layer")
        self.task.global_costmap_inflation_layer = (
            "/move_base/global_costmap/inflation_layer")
        self.task.local_costmap_reconfigure_timeout = 0.5
        self.task.processing_parking_inflation_radius_m = 0.07
        self.task._processing_parking_original_inflation_radius_m = None
        self.task._processing_parking_original_global_inflation_radius_m = None

        original_client = task_module.DynamicReconfigureClient
        original_sleep = task_module.rospy.sleep
        task_module.DynamicReconfigureClient = InflationClient
        task_module.rospy.sleep = lambda _duration: None
        try:
            self.task.enter_processing_parking_profile()
            self.assertEqual(
                inflation_state["/move_base/local_costmap/inflation_layer"],
                0.07)
            self.assertEqual(
                inflation_state["/move_base/global_costmap/inflation_layer"],
                0.07)
            self.assertEqual(messages, [
                "point", "point", "point"])

            self.task.exit_processing_parking_profile()
            self.assertEqual(
                inflation_state["/move_base/local_costmap/inflation_layer"],
                0.224)
            self.assertEqual(
                inflation_state["/move_base/global_costmap/inflation_layer"],
                0.224)
            self.assertEqual(messages, [
                "point", "point", "point",
                "point", "point", "point"])
            self.assertIsNone(
                self.task._processing_parking_original_inflation_radius_m)
            self.assertIsNone(
                self.task._processing_parking_original_global_inflation_radius_m)
        finally:
            task_module.DynamicReconfigureClient = original_client
            task_module.rospy.sleep = original_sleep

    def test_processing_parking_approaches_recorded_point_before_low_inflation(self):
        events = []
        observation = {
            "route_point_number": 16,
            "ocr_aligned_pose_map": [0.31, 0.74, -1.20],
            "wall_point_number": 300,
            "wall_point_coordinate": [0.75, 1.50],
            "forward_ray_wall_intersection_map": [0.75, 1.50],
            "measured_wall_hit_map": [0.80, 1.50],
        }
        self.task.points = {
            16: (0.25, 0.75),
        }
        self.task.processing_parking_profile_enabled = True
        self.task.last_recorded_observation = lambda _category: observation
        self.task.log_safe_text = lambda value: value
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.publish_state = lambda _state: None
        self.task.navigate_coordinates = (
            lambda x, y, yaw, label, require_plan=True: events.append(
                ("navigate", x, y, yaw, label, require_plan)))
        self.task.enter_processing_parking_profile = (
            lambda: events.append(("enter_profile",)))
        self.task.exit_processing_parking_profile = (
            lambda: events.append(("exit_profile",)))

        self.task.park_at_recorded_production_category(
            u"毛巾", u"日用品", announce=False)

        self.assertEqual(
            events,
            [
                ("navigate", 0.31, 0.74, -1.20,
                 "processing observation point 16", True),
                ("enter_profile",),
                ("navigate", 0.80, 1.25, math.pi / 2.0,
                 "processing stop point 300", True),
                ("exit_profile",),
            ])

    def test_observe_wall_records_ocr_aligned_pose(self):
        self.task.camera_width = 100
        self.task.ocr_alignment_attempts = 1
        self.task.ocr_alignment_tolerance_px = 30.0
        self.task.require_safe = lambda: None
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.capture_ocr = lambda _label, _attempt: {
            "image_path": "frame.png",
            "width": 100,
            "detection": {
                "text": u"毛巾",
                "confidence": 0.99,
                "bbox": [40, 10, 20, 20],
            },
        }
        self.task.current_map_pose = (
            lambda _context: (1.10, 2.20, 0.70))
        self.task.wait_for_fresh_front_distance = (
            lambda: (object(), 0.50))
        self.task.laser_map_pose = (
            lambda _scan: (1.10, 2.20, 0.70))
        self.task.wall_reference_points = {300: (0.75, 1.50)}
        self.task.lidar_forward_offset = 0.0
        self.task.ray_range_agreement = 1.0

        original_intersection = (
            task_module.forward_ray_wall_intersection)
        original_nearest = task_module.nearest_numbered_point
        task_module.forward_ray_wall_intersection = (
            lambda *_args: (1.0, (0.75, 1.50)))
        task_module.nearest_numbered_point = (
            lambda *_args: (300, (0.75, 1.50), 0.0))
        try:
            observation = self.task.observe_wall(16, "pose record")
        finally:
            task_module.forward_ray_wall_intersection = (
                original_intersection)
            task_module.nearest_numbered_point = original_nearest

        self.assertEqual(
            observation["ocr_aligned_pose_map"], [1.10, 2.20, 0.70])

    def test_observe_wall_expands_tolerance_after_five_attempts(self):
        self.task.camera_width = 100
        self.task.ocr_alignment_attempts = 6
        self.task.ocr_alignment_tolerance_px = 30.0
        self.task.ocr_alignment_retry_tolerance_increment_px = 30.0
        self.task.ocr_alignment_kp = 0.0025
        self.task.ocr_alignment_kd = 0.00035
        self.task.ocr_alignment_max_speed = 0.22
        self.task.camera_mirror = False
        self.task.require_safe = lambda: None
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        captures = []

        def capture(_label, attempt):
            captures.append(attempt)
            return {
                "image_path": "frame-%d.png" % attempt,
                "width": 100,
                "detection": {
                    "text": u"毛巾",
                    "confidence": 0.99,
                    "bbox": [80, 10, 20, 20],
                },
            }

        self.task.capture_ocr = capture
        self.task.capture_ocr_while_turning = (
            lambda _speed, label, attempt: capture(label, attempt))
        self.task.current_map_pose = (
            lambda _context: (1.10, 2.20, 0.70))
        self.task.wait_for_fresh_front_distance = (
            lambda: (object(), 0.50))
        self.task.laser_map_pose = (
            lambda _scan: (1.10, 2.20, 0.70))
        self.task.wall_reference_points = {300: (0.75, 1.50)}
        self.task.lidar_forward_offset = 0.0
        self.task.ray_range_agreement = 1.0

        original_intersection = (
            task_module.forward_ray_wall_intersection)
        original_nearest = task_module.nearest_numbered_point
        task_module.forward_ray_wall_intersection = (
            lambda *_args: (1.0, (0.75, 1.50)))
        task_module.nearest_numbered_point = (
            lambda *_args: (300, (0.75, 1.50), 0.0))
        try:
            observation = self.task.observe_wall(16, "retry tolerance")
        finally:
            task_module.forward_ray_wall_intersection = (
                original_intersection)
            task_module.nearest_numbered_point = original_nearest

        self.assertTrue(observation["aligned"])
        self.assertEqual(captures, [1, 2, 3, 4, 5, 6])

    def test_point_three_global_inflation_is_applied_and_verified(self):
        inflation_state = [0.20]
        namespaces = []

        class InflationClient(object):

            def __init__(self, namespace, timeout=None):
                namespaces.append(namespace)

            def update_configuration(self, configuration):
                inflation_state[0] = float(configuration["inflation_radius"])
                return {"inflation_radius": inflation_state[0]}

        self.task.local_costmap_layer_control_enabled = True
        self.task.global_costmap_inflation_layer = (
            "/move_base/global_costmap/inflation_layer")
        self.task.local_costmap_reconfigure_timeout = 0.5

        original_client = task_module.DynamicReconfigureClient
        task_module.DynamicReconfigureClient = InflationClient
        try:
            applied = self.task.set_global_costmap_inflation_radius(
                0.224, "reached_point_3")
            self.assertEqual(applied, 0.224)
            self.assertEqual(inflation_state[0], 0.224)
            self.assertEqual(namespaces, [
                "/move_base/global_costmap/inflation_layer"])
        finally:
            task_module.DynamicReconfigureClient = original_client

    def test_rosout_imu_and_ahrs_crc_are_warnings_but_head_len_stays_critical(self):
        self.task.lock = threading.RLock()
        self.task.critical_error = ""
        warnings = []
        original_throttle = task_module.rospy.logwarn_throttle
        task_module.rospy.logwarn_throttle = (
            lambda _period, message, *args: warnings.append(message % args))
        try:
            self.task.rosout_cb(type("LogMessage", (object,), {
                "msg": "check crc16 faild(imu)."})())
            self.assertEqual(self.task.critical_error, "")
            self.assertTrue(any("PRODUCTION_IMU_CRC_IGNORED" in item
                                for item in warnings))

            self.task.rosout_cb(type("LogMessage", (object,), {
                "msg": "check crc16 faild(ahrs)."})())
            self.assertEqual(self.task.critical_error, "")
            self.assertTrue(any("PRODUCTION_AHRS_CRC_IGNORED" in item
                                for item in warnings))

            self.task.rosout_cb(type("LogMessage", (object,), {
                "msg": "base driver check head_len faild"})())
            self.assertEqual(
                self.task.critical_error,
                "base driver check head_len faild")
        finally:
            task_module.rospy.logwarn_throttle = original_throttle

    def test_post_turn_position_excess_warns_and_continues(self):
        self.assertFalse(
            self.task.verify_position(
                16, "post-turn point 16", warn_only=True))
        self.assertEqual(len(self.warnings), 1)
        self.assertIn("continuing mission", self.warnings[0])

    def test_strict_position_check_still_aborts(self):
        with self.assertRaises(task_module.MissionAbort):
            self.task.verify_position(16, "normal point 16")

    def test_async_ocr_response_keeps_capture_pose_metadata(self):
        completed = threading.Event()
        completed.set()

        class CompletedThread(object):
            def join(self):
                pass

        self.task.ocr_capture_timeout = 0.1
        response = {"ok": True, "detection": {"text": "test"}}
        result = self.task.finish_async_motion_ocr({
            "done": completed,
            "thread": CompletedThread(),
            "response": response,
            "error": None,
            "capture_requested_at": "2026-08-03T00:00:00+0800",
            "capture_requested_pose_map": [-1.75, 0.75, 0.5],
        })

        self.assertEqual(
            result["capture_requested_pose_map"], [-1.75, 0.75, 0.5])
        self.assertEqual(
            result["capture_requested_at"], "2026-08-03T00:00:00+0800")

    def test_target_guard_requires_two_fresh_matching_scans(self):
        def scan_at(stamp):
            class Header(object):
                pass

            class Scan(object):
                angle_min = 0.0
                angle_increment = math.pi / 2.0
                range_min = 0.05
                range_max = 5.0
                ranges = [0.20, float("inf")]

            Scan.header = Header()
            Scan.header.stamp = stamp
            return Scan()

        self.task.lock = threading.RLock()
        self.task.target_guard_points = {
            12: {419: (-2.0, 1.0), 420: (-1.5, 1.0),
                 428: (-2.0, 0.5), 429: (-1.5, 0.5)}}
        self.task.target_guard_scan_sequence = 0
        self.task.target_guard_match_radius = 0.08
        self.task.target_guard_confirmation_scans = 2
        self.task.target_guard_scan_max_age = 1.0
        self.task.laser_map_pose = lambda _scan: (-2.20, 1.00, 0.0)

        monitor = self.task.new_target_guard_monitor(12)
        first_stamp = task_module.rospy.Time.now()
        self.task.latest_target_guard_scan = scan_at(first_stamp)
        self.task.latest_target_guard_scan_receipt = task_module.rospy.Time.now()
        self.task.target_guard_scan_sequence = 1
        self.assertIsNone(self.task.poll_target_guard(monitor))

        self.task.latest_target_guard_scan = scan_at(
            first_stamp + task_module.rospy.Duration(0.01))
        self.task.latest_target_guard_scan_receipt = task_module.rospy.Time.now()
        self.task.target_guard_scan_sequence = 2
        self.assertEqual(self.task.poll_target_guard(monitor), 419)

    def test_target_guard_drops_an_unconfirmed_hit_after_a_clean_scan(self):
        def scan_at(stamp, ranges):
            class Header(object):
                pass

            class Scan(object):
                angle_min = 0.0
                angle_increment = math.pi / 2.0
                range_min = 0.05
                range_max = 5.0

            Scan.header = Header()
            Scan.header.stamp = stamp
            Scan.ranges = ranges
            return Scan()

        self.task.lock = threading.RLock()
        self.task.target_guard_points = {
            12: {419: (-2.0, 1.0), 420: (-1.5, 1.0),
                 428: (-2.0, 0.5), 429: (-1.5, 0.5)}}
        self.task.target_guard_scan_sequence = 0
        self.task.target_guard_match_radius = 0.08
        self.task.target_guard_confirmation_scans = 2
        self.task.target_guard_scan_max_age = 1.0
        self.task.laser_map_pose = lambda _scan: (-2.20, 1.00, 0.0)

        monitor = self.task.new_target_guard_monitor(12)
        first_stamp = task_module.rospy.Time.now()
        self.task.latest_target_guard_scan = scan_at(
            first_stamp, [0.20, float("inf")])
        self.task.latest_target_guard_scan_receipt = task_module.rospy.Time.now()
        self.task.target_guard_scan_sequence = 1
        self.assertIsNone(self.task.poll_target_guard(monitor))

        self.task.latest_target_guard_scan = scan_at(
            first_stamp + task_module.rospy.Duration(0.01), [float("inf")])
        self.task.latest_target_guard_scan_receipt = task_module.rospy.Time.now()
        self.task.target_guard_scan_sequence = 2
        self.assertIsNone(self.task.poll_target_guard(monitor))
        self.assertEqual(monitor["hit_counts"], {})

    def test_target_guard_rejects_a_republished_scan_stamp(self):
        class Header(object):
            pass

        class Scan(object):
            angle_min = 0.0
            angle_increment = math.pi / 2.0
            range_min = 0.05
            range_max = 5.0
            ranges = [0.20, float("inf")]

        stamp = task_module.rospy.Time.now()
        Scan.header = Header()
        Scan.header.stamp = stamp
        self.task.lock = threading.RLock()
        self.task.target_guard_points = {
            12: {419: (-2.0, 1.0), 420: (-1.5, 1.0),
                 428: (-2.0, 0.5), 429: (-1.5, 0.5)}}
        self.task.target_guard_scan_sequence = 0
        self.task.target_guard_match_radius = 0.08
        self.task.target_guard_confirmation_scans = 2
        self.task.target_guard_scan_max_age = 1.0
        self.task.laser_map_pose = lambda _scan: (-2.20, 1.00, 0.0)

        monitor = self.task.new_target_guard_monitor(12)
        self.task.latest_target_guard_scan = Scan()
        self.task.latest_target_guard_scan_receipt = task_module.rospy.Time.now()
        self.task.target_guard_scan_sequence = 1
        self.assertIsNone(self.task.poll_target_guard(monitor))

        self.task.latest_target_guard_scan = Scan()
        self.task.latest_target_guard_scan_receipt = task_module.rospy.Time.now()
        self.task.target_guard_scan_sequence = 2
        self.assertIsNone(self.task.poll_target_guard(monitor))
        self.assertEqual(monitor["hit_counts"], {})

    def test_target_guard_resets_hits_after_a_long_source_gap(self):
        class Header(object):
            pass

        class Scan(object):
            angle_min = 0.0
            angle_increment = math.pi / 2.0
            range_min = 0.05
            range_max = 5.0
            ranges = [0.20, float("inf")]

        self.task.lock = threading.RLock()
        self.task.target_guard_points = {
            12: {419: (-2.0, 1.0), 420: (-1.5, 1.0),
                 428: (-2.0, 0.5), 429: (-1.5, 0.5)}}
        self.task.target_guard_scan_sequence = 0
        self.task.target_guard_match_radius = 0.08
        self.task.target_guard_confirmation_scans = 2
        self.task.target_guard_scan_max_age = 0.10
        self.task.laser_map_pose = lambda _scan: (-2.20, 1.00, 0.0)

        monitor = self.task.new_target_guard_monitor(12)
        Scan.header = Header()
        Scan.header.stamp = task_module.rospy.Time.now()
        self.task.latest_target_guard_scan = Scan()
        self.task.latest_target_guard_scan_receipt = task_module.rospy.Time.now()
        self.task.target_guard_scan_sequence = 1
        self.assertIsNone(self.task.poll_target_guard(monitor))

        time.sleep(0.12)
        Scan.header = Header()
        Scan.header.stamp = task_module.rospy.Time.now()
        self.task.latest_target_guard_scan = Scan()
        self.task.latest_target_guard_scan_receipt = task_module.rospy.Time.now()
        self.task.target_guard_scan_sequence = 2
        self.assertIsNone(self.task.poll_target_guard(monitor))
        self.assertEqual(monitor["hit_counts"], {419: 1})

    def test_fallback_target_without_middle_guard_uses_navigation_layers(self):
        self.task.lock = threading.RLock()
        self.task.target_guard_points = {12: {419: (-2.0, 1.0)}}
        self.task.target_guard_scan_sequence = 0

        monitor = self.task.new_target_guard_monitor(1)

        self.assertFalse(monitor["guard_enabled"])
        self.assertIsNone(self.task.wait_for_target_guard_precheck(monitor))
        self.assertFalse(self.task.target_guard_scan_expired(monitor))

    def test_target_guard_scan_expiry_cancels_then_aborts_a_target_leg(self):
        events = []
        self.task.points = {12: (-1.75, 0.75)}
        self.task.production_navigation_legs = [(52, 12), (12, 23)]
        self.task.publish_state = lambda _state: None
        self.task.new_target_guard_monitor = lambda _target: {"test": True}
        self.task.wait_for_target_guard_precheck = lambda _monitor: None
        self.task.poll_target_guard = lambda _monitor: None
        self.task.target_guard_scan_expired = lambda _monitor: True
        self.task.scan_production_point = lambda *_args: self.fail(
            "the OCR scan must not start after guard scan loss")

        def navigate_with_guard(*_args, **kwargs):
            self.assertTrue(kwargs["require_plan"])
            self.assertTrue(kwargs["guard_callback"]())
            events.append("cancelled_by_navigation_supervisor")
            return False

        self.task.navigate_coordinates = navigate_with_guard
        with self.assertRaises(task_module.MissionAbort):
            self.task.navigate_target_and_scan(1, 52, 12, 0.0)
        self.assertEqual(events, ["cancelled_by_navigation_supervisor"])

    def test_target_guard_precheck_aborts_without_a_usable_scan(self):
        self.task.target_guard_precheck_timeout = 0.001
        self.task.require_safe = lambda: None
        self.task.poll_target_guard = lambda _monitor: None
        monitor = {
            "target_number": 12,
            "usable_scan_seen": False,
            "clean_scan_seen": False,
            "hit_counts": {},
        }

        with self.assertRaises(task_module.MissionAbort):
            self.task.wait_for_target_guard_precheck(monitor)

    def test_target_guard_before_goal_skips_without_sending_move_base_goal(self):
        events = []
        self.task.points = {12: (-1.75, 0.75)}
        self.task.production_navigation_legs = [(52, 12), (12, 23)]
        self.task.publish_state = lambda state: events.append(("state", state))
        self.task.new_target_guard_monitor = lambda _target: {"test": True}
        self.task.wait_for_target_guard_precheck = lambda _monitor: 419
        self.task.stop_motion = lambda: events.append(("stop",))
        self.task.wait_for_chassis_stop = lambda label: events.append(
            ("stopped", label))
        self.task.record_target_guard_skip = (
            lambda *_args: events.append(("guard_skip", _args[3], _args[4])))
        self.task.navigate_coordinates = lambda *_args, **_kwargs: self.fail(
            "a pre-goal guard must not send a move_base target")

        outcome = self.task.navigate_target_and_scan(1, 52, 12, 0.0)

        self.assertEqual(outcome, "target_guard_skipped")
        self.assertIn(("guard_skip", 419, "before_goal"), events)

    def test_navigation_guard_cancels_goal_from_supervisor_loop(self):
        events = []

        class MoveBase(object):
            def send_goal(self, _goal):
                events.append("send_goal")

        self.task.move_base = MoveBase()
        self.task.goal_timeout = 1.0
        self.task.require_safe = lambda: None
        self.task.cancel_navigation_for_observation = lambda label: events.append(
            ("cancel_and_stop", label))

        result = self.task.navigate_coordinates(
            -1.75, 0.75, 0.0, "guarded target", require_plan=False,
            guard_callback=lambda: True)

        self.assertFalse(result)
        self.assertEqual(events, [
            "send_goal", ("cancel_and_stop", "guarded target target guard")])

    def test_point_mode_is_published_without_body_projection(self):
        messages = []

        class NavigationModePublisher(object):
            def get_num_connections(self):
                return 1

            def publish(self, message):
                messages.append(message.data)

        self.task.navigation_mode_pub = NavigationModePublisher()
        self.task.navigation_mode_connect_timeout = 0.5
        self.task.require_safe = lambda: None
        self.task.publish_state = lambda _state: None

        self.task.switch_to_point_mode()

        self.assertEqual(messages, ["point", "point", "point"])

    def test_destination_mode_is_published_for_final_441_approach(self):
        messages = []

        class NavigationModePublisher(object):
            def get_num_connections(self):
                return 1

            def publish(self, message):
                messages.append(message.data)

        self.task.navigation_mode_pub = NavigationModePublisher()
        self.task.navigation_mode_connect_timeout = 0.5
        self.task.require_safe = lambda: None
        self.task.publish_state = lambda _state: None

        self.task.switch_to_destination_mode()

        self.assertEqual(messages, [
            "destination", "destination", "destination"])

    def test_point_mode_is_selected_before_staging_navigation(self):
        events = []

        class StopAtStaging(Exception):
            pass

        class MoveBase(object):
            def wait_for_server(self, _timeout):
                return True

        self.task.move_base = MoveBase()
        self.task.move_base_ready_timeout = 1.0
        self.task.publish_state = lambda _state: None
        self.task.wait_for_item_inputs = (
            lambda: (events.append("item_input"), (u"苹果", u"手机"))[1])
        self.task.wait_for_safe_start = lambda: events.append("safe_start")
        self.task.switch_to_point_mode = lambda: events.append("point_mode")
        self.task.resume_production_only = False
        self.task.staging_point_number = 52
        self.task.qr_observation_numbers = [262]
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}
        self.task.prepare_result_directory = lambda: None

        def navigate_to(*_args, **_kwargs):
            events.append("staging_navigation")
            raise StopAtStaging()

        self.task.navigate_to = navigate_to
        with self.assertRaises(StopAtStaging):
            self.task.run_mission()

        self.assertEqual(
            events,
            ["item_input", "safe_start", "point_mode",
             "staging_navigation"])

    def test_point_mode_is_selected_before_resumed_production_navigation(self):
        events = []

        class StopAtFirstProductionLeg(Exception):
            pass

        class MoveBase(object):
            def wait_for_server(self, _timeout):
                return True

        class QrPublisher(object):
            def publish(self, _message):
                pass

        self.task.move_base = MoveBase()
        self.task.move_base_ready_timeout = 1.0
        self.task.publish_state = lambda _state: None
        self.task.wait_for_item_inputs = (
            lambda: (events.append("item_input"), (u"苹果", u"手机"))[1])
        self.task.wait_for_safe_start = lambda: events.append("safe_start")
        self.task.switch_to_point_mode = lambda: events.append("point_mode")
        self.task.resume_production_only = True
        self.task.global_costmap_inflation_radius_m = 0.224
        self.task.set_global_costmap_inflation_radius = (
            lambda *_args: events.append("global_inflation"))
        self.task.qr_enable_pub = QrPublisher()

        def classify_qr_text(observation_number, qr_text):
            self.task.qr_classifications.append({
                "observation": observation_number,
                "qr_text": qr_text.decode("utf-8"),
                "category": u"日用品",
                "source": "stub",
                "attempts": 0,
                "model": "test",
                "error": "",
            })

        self.task.classify_qr_text = classify_qr_text
        self.task.prepare_result_directory = lambda: None
        self.task.use_ros_camera_for_ocr = True
        self.task.camera_image_topic = "/usb_cam/image_raw"
        self.task.start_native_ocr = lambda: None
        self.task.production_route_numbers = [12]
        self.task.production_navigation_legs = [(52, 12)]
        self.task.production_observation_headings = [0.0]

        def navigate_target_and_scan(*_args, **_kwargs):
            events.append("production_navigation")
            raise StopAtFirstProductionLeg()

        self.task.navigate_target_and_scan = navigate_target_and_scan
        with self.assertRaises(StopAtFirstProductionLeg):
            self.task.run_mission()

        self.assertEqual(
            events,
            ["item_input", "safe_start", "point_mode",
             "global_inflation",
             "production_navigation"])

    def test_qr_completion_does_not_switch_before_first_production_leg(self):
        events = []

        class StopAtFirstProductionLeg(Exception):
            pass

        class MoveBase(object):
            def wait_for_server(self, _timeout):
                return True

            def cancel_all_goals(self):
                pass

        class QrPublisher(object):
            def publish(self, _message):
                pass

        self.task.move_base = MoveBase()
        self.task.move_base_ready_timeout = 1.0
        self.task.publish_state = lambda _state: None
        self.task.wait_for_item_inputs = (
            lambda: (events.append("item_input"), (u"苹果", u"手机"))[1])
        self.task.wait_for_safe_start = lambda: events.append("safe_start")
        self.task.switch_to_point_mode = lambda: events.append("point_mode")
        self.task.switch_to_body_projection = lambda: self.fail(
            "QR completion must not select body_projection")
        self.task.resume_production_only = False
        self.task.staging_point_number = 52
        self.task.qr_observation_numbers = [262]
        self.task.points = {
            52: (-1.75, 2.25), 262: (-2.50, 2.25), 3: (-1.25, 1.25),
        }
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.wait_for_qr_scanner = lambda: None
        self.task.start_ros_camera_and_wait = lambda _context: None
        self.task.qr_enable_pub = QrPublisher()
        self.task.stop_qr_classifier = lambda: None
        self.task.post_qr_waypoint_number = 3
        self.task.post_qr_waypoint_heading_point_number = 0

        qr_codes = iter([u"苹果", u"手机"])

        def scan_observation_point(observation_number, accept_text=None,
                                   allow_revolution=True):
            events.append("qr_scan")
            try:
                return next(qr_codes)
            except StopIteration:
                return None

        self.task.scan_observation_point = scan_observation_point

        def classify_qr_text(observation_number, qr_text):
            self.task.qr_classifications.append({
                "observation": observation_number,
                "qr_text": qr_text.decode("utf-8"),
                "category": u"日用品",
                "source": "stub",
                "attempts": 0,
                "model": "test",
                "error": "",
            })

        self.task.classify_qr_text = classify_qr_text
        self.task.stop_ros_camera_streaming = lambda required=True: None
        self.task.prepare_result_directory = lambda: None
        self.task.use_ros_camera_for_ocr = True
        self.task.camera_image_topic = "/usb_cam/image_raw"
        self.task.start_native_ocr = lambda: events.append("ocr_start")
        self.task.speak_wait = (
            lambda text, timeout=None: events.append(("announce", text)))
        self.task.navigate_coordinates = (
            lambda _x, _y, _yaw, label, **_kwargs:
            events.append(("navigate", label)))
        self.task.production_route_numbers = [12]
        self.task.production_navigation_legs = [(52, 12)]
        self.task.production_observation_headings = [0.0]

        def navigate_to(_number, _yaw, label):
            events.append(label)

        self.task.navigate_to = navigate_to

        def navigate_target_and_scan(*_args, **_kwargs):
            events.append("production_navigation")
            raise StopAtFirstProductionLeg()

        self.task.navigate_target_and_scan = navigate_target_and_scan
        with self.assertRaises(StopAtFirstProductionLeg):
            self.task.run_mission()

        self.assertEqual(
            events,
            ["item_input", "safe_start", "point_mode", "STAGING_52",
             "qr_scan", "qr_scan",
             ("announce", u"取得*苹果*属于*日用品*应放置在*日用品加工车间"),
             ("announce", u"仿真环境中取得*手机*属于*日用品*应放置在*日用品加工车间"),
             ("navigate", "post-QR waypoint 3"), "ocr_start",
             "production_navigation"])

    def test_camera_stream_start_and_stop_are_idempotent(self):
        events = []
        self.task.use_ros_camera_for_ocr = True
        self.task.lock = threading.RLock()
        self.task.camera_streaming = False
        self.task.latest_camera_image = object()
        self.task.latest_camera_receipt = object()
        def call_camera_service(service_name, _context, required=True):
            events.append((service_name, required))
            return True
        self.task.call_camera_service = call_camera_service
        self.task.camera_start_service = "/usb_cam/start_capture"
        self.task.camera_stop_service = "/usb_cam/stop_capture"

        self.task.set_ros_camera_streaming(True)
        self.task.set_ros_camera_streaming(True)
        self.task.set_ros_camera_streaming(False, required=False)
        self.task.set_ros_camera_streaming(False, required=False)

        self.assertEqual(
            events,
            [
                ("/usb_cam/start_capture", True),
                ("/usb_cam/stop_capture", False),
            ])
        self.assertIsNone(self.task.latest_camera_image)
        self.assertIsNone(self.task.latest_camera_receipt)

    def test_camera_start_waits_for_requested_fresh_frames(self):
        self.task.lock = threading.RLock()
        self.task.camera_sequence = 4
        self.task.latest_camera_receipt = None
        self.task.camera_warmup_frames = 2
        self.task.camera_service_timeout = 0.5
        self.task.require_safe = lambda: None
        self.task.set_ros_camera_streaming = (
            lambda _enabled, required=True: True)

        def publish_frames():
            time.sleep(0.03)
            self.task.camera_image_cb(object())
            self.task.camera_image_cb(object())

        worker = threading.Thread(target=publish_frames)
        worker.start()
        self.task.start_ros_camera_and_wait("test")
        worker.join()
        self.assertEqual(self.task.camera_sequence, 6)

    def test_ocr_capture_waits_for_next_camera_frame(self):
        self.task.lock = threading.RLock()
        self.task.camera_sequence = 7
        old_frame = object()
        new_frame = object()
        self.task.latest_camera_image = old_frame
        self.task.latest_camera_receipt = task_module.rospy.Time.now()
        self.task.camera_frame_timeout = 0.5
        self.task.require_safe = lambda: None

        class Bridge(object):
            def __init__(self):
                self.messages = []

            def imgmsg_to_cv2(self, message, desired_encoding=None):
                self.messages.append(message)
                return object()

        bridge = Bridge()
        self.task.cv_bridge = bridge
        original_imwrite = task_module.cv2.imwrite
        task_module.cv2.imwrite = lambda *_args: True

        def publish_next_frame():
            time.sleep(0.03)
            self.task.camera_image_cb(new_frame)

        worker = threading.Thread(target=publish_next_frame)
        worker.start()
        try:
            self.task.save_latest_ros_camera_frame("unused.png")
        finally:
            worker.join()
            task_module.cv2.imwrite = original_imwrite

        self.assertEqual(bridge.messages, [new_frame])
        self.assertEqual(self.task.camera_sequence, 8)

    def test_camera_start_without_fresh_frame_aborts(self):
        self.task.lock = threading.RLock()
        self.task.camera_sequence = 0
        self.task.latest_camera_receipt = None
        self.task.camera_warmup_frames = 1
        self.task.camera_service_timeout = 0.05
        self.task.require_safe = lambda: None
        self.task.set_ros_camera_streaming = (
            lambda _enabled, required=True: True)
        with self.assertRaises(task_module.MissionAbort):
            self.task.start_ros_camera_and_wait("test")

    def test_fresh_qr_callback_wakes_search_without_fixed_hold(self):
        self.task.lock = threading.RLock()
        self.task.api_sequence = 0
        self.task.latest_api_text = ""
        self.task.used_qr_codes = set()
        self.task.require_distinct_qr_codes = True
        self.task.api_event = threading.Event()
        self.task.require_safe = lambda: None

        class ApiMessage(object):
            data = ('{"ok": true, "key": "a", '
                    '"qr_text": "http://192.168.8.1:3663/a", '
                    '"response": {"code": 200, "result": "\u82f9\u679c"}}')

        def publish_qr():
            time.sleep(0.01)
            self.task.qr_api_result_cb(ApiMessage())

        worker = threading.Thread(target=publish_qr)
        worker.start()
        started = time.time()
        detected = self.task.wait_for_fresh_qr(0, 1.0)
        elapsed = time.time() - started
        worker.join()

        self.assertEqual(detected, u"苹果")
        self.assertLess(elapsed, 0.20)

    def test_qr_api_queue_preserves_multiple_results_and_first_item(self):
        self.task.lock = threading.RLock()
        self.task.api_sequence = 0
        self.task.latest_api_text = ""
        self.task.used_qr_codes = set()
        self.task.require_distinct_qr_codes = True
        self.task.api_event = threading.Event()
        self.task.require_safe = lambda: None

        class ApiMessage(object):
            pass

        messages = [
            ("http://192.168.8.1:3663/a", u"苹果"),
            ("http://192.168.8.1:3663/b", u"手机"),
            ("http://192.168.8.1:3663/a", u"香蕉"),
        ]
        for qr_text, item in messages:
            message = ApiMessage()
            message.data = json.dumps({
                "ok": True,
                "qr_text": qr_text,
                "response": {"code": 200, "result": item},
            }, ensure_ascii=False)
            self.task.qr_api_result_cb(message)

        first = self.task.wait_for_fresh_qr(0, 0.1)
        self.task.used_qr_codes.add(first)
        second = self.task.wait_for_fresh_qr(0, 0.1)

        self.assertEqual(first, u"苹果")
        self.assertEqual(second, u"手机")
        self.assertEqual(
            self.task.first_qr_item_by_code["http://192.168.8.1:3663/a"],
            u"苹果")

    def test_qr_seen_while_facing_is_accepted_without_search_wait(self):
        self.task.lock = threading.RLock()
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}
        self.task.staging_point_number = 52
        self.task.api_sequence = 0
        self.task.latest_api_text = ""
        self.task.used_qr_codes = set()
        self.task.qr_observation_numbers = [262]
        self.task.require_distinct_qr_codes = True
        self.task.api_event = threading.Event()
        self.task.publish_state = lambda _state: None

        class ApiMessage(object):
            data = ('{"ok": true, "key": "a", '
                    '"qr_text": "http://192.168.8.1:3663/a", '
                    '"response": {"code": 200, "result": "\u82f9\u679c"}}')

        def navigate(*_args, **_kwargs):
            self.task.qr_api_result_cb(ApiMessage())

        self.task.navigate_coordinates = navigate
        self.task.wait_for_fresh_qr = (
            lambda *_args: self.fail("must not wait after a turn-time QR"))

        self.task.scan_observation_point(262)

        self.assertEqual(self.task.used_qr_codes, set([u"苹果"]))

    def test_scan_observation_point_turns_full_revolution_after_timeout(self):
        self.task.lock = threading.RLock()
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}
        self.task.staging_point_number = 52
        self.task.api_sequence = 0
        self.task.latest_api_text = ""
        self.task.api_event = threading.Event()
        self.task.publish_state = lambda _state: None
        self.task.qr_search_timeout = 1.0
        self.task.qr_rotation_speed = 0.18
        self.task.navigate_coordinates = lambda *_args, **_kwargs: None
        self.task.accepted_qr_after = lambda *_args, **_kwargs: (None, False)
        self.task.wait_for_fresh_qr = lambda *_args, **_kwargs: (None, False)
        turns = []

        def rotate_full_revolution(label, speed, stop_for_qr, qr_baseline,
                                   qr_accept=None,
                                   qr_observation_number=None):
            turns.append((label, speed, stop_for_qr, qr_baseline,
                          qr_accept is not None, qr_observation_number))
            return None

        self.task.rotate_full_revolution = rotate_full_revolution

        # One full turn still finds no code: the face yields None and the
        # collection loop moves on to the next face.
        self.assertIsNone(
            self.task.scan_observation_point(262, lambda raw: True))
        self.assertEqual(turns, [(
            "QR observation point 262", 0.18, True, 0,
            True, 262)])

    def test_scan_observation_point_skips_revolution_when_disabled(self):
        self.task.lock = threading.RLock()
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}
        self.task.staging_point_number = 52
        self.task.api_sequence = 0
        self.task.latest_api_text = ""
        self.task.api_event = threading.Event()
        self.task.publish_state = lambda _state: None
        self.task.qr_search_timeout = 1.0
        self.task.qr_rotation_speed = 0.18
        self.task.navigate_coordinates = lambda *_args, **_kwargs: None
        self.task.accepted_qr_after = lambda *_args, **_kwargs: (None, False)
        self.task.wait_for_fresh_qr = lambda *_args, **_kwargs: (None, False)
        turns = []

        def rotate_full_revolution(label, speed, stop_for_qr, qr_baseline,
                                   qr_accept=None,
                                   qr_observation_number=None):
            turns.append((label, speed, stop_for_qr, qr_baseline,
                          qr_accept is not None, qr_observation_number))
            return None

        self.task.rotate_full_revolution = rotate_full_revolution

        # With the revolution fallback disabled this face only faces and
        # waits: no 360-degree turn happens and None lets the collection
        # loop advance to the next fixed direction.
        self.assertIsNone(
            self.task.scan_observation_point(
                262, lambda raw: True, allow_revolution=False))
        self.assertEqual(turns, [])

    def test_scan_observation_point_advances_after_non_target_qr(self):
        self.task.lock = threading.RLock()
        self.task.points = {52: (-1.75, 2.25), 232: (-1.75, 3.00)}
        self.task.staging_point_number = 52
        self.task.api_sequence = 0
        self.task.latest_api_text = ""
        self.task.api_event = threading.Event()
        self.task.publish_state = lambda _state: None
        self.task.navigate_coordinates = lambda *_args, **_kwargs: None
        self.task.accepted_qr_after = lambda *_args, **_kwargs: (None, True)
        self.task.wait_for_fresh_qr = (
            lambda *_args, **_kwargs: self.fail(
                "a non-target QR must not wait at the current face"))
        self.task.rotate_full_revolution = (
            lambda *_args, **_kwargs: self.fail(
                "a non-target QR must advance to the next fixed face"))

        self.assertIsNone(
            self.task.scan_observation_point(
                232, lambda text: text == u"苹果"))

    def test_accepted_qr_after_reports_non_target_qr(self):
        rejected = []
        self.task.fresh_qr_after = lambda _baseline: u"毛巾"
        self.task._reject_qr_code = (
            lambda text, observation: rejected.append((text, observation)))

        result = self.task.accepted_qr_after(
            7, lambda text: text == u"苹果", 232,
            report_rejection=True)

        self.assertEqual(result, (None, True))
        self.assertEqual(rejected, [(u"毛巾", 232)])

    def test_scan_observation_point_accepts_code_during_fallback_turn(self):
        self.task.lock = threading.RLock()
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}
        self.task.staging_point_number = 52
        self.task.api_sequence = 0
        self.task.latest_api_text = ""
        self.task.api_event = threading.Event()
        self.task.publish_state = lambda _state: None
        self.task.qr_search_timeout = 1.0
        self.task.qr_rotation_speed = 0.18
        self.task.used_qr_codes = set()
        self.task.navigate_coordinates = lambda *_args, **_kwargs: None
        self.task.accepted_qr_after = lambda *_args, **_kwargs: (None, False)
        self.task.wait_for_fresh_qr = lambda *_args, **_kwargs: (None, False)
        self.task.rotate_full_revolution = (
            lambda *_args, **_kwargs: u"苹果")

        detected = self.task.scan_observation_point(262)

        self.assertEqual(detected, u"苹果")
        self.assertEqual(self.task.used_qr_codes, set([u"苹果"]))

    def test_camera_stop_retries_once_before_succeeding(self):
        calls = []
        self.task.lock = threading.RLock()
        self.task.camera_streaming = True
        self.task.latest_camera_image = object()
        self.task.latest_camera_receipt = object()
        self.task.camera_start_service = "/usb_cam/start_capture"
        self.task.camera_stop_service = "/usb_cam/stop_capture"

        def call_camera_service(service_name, _context, required=True):
            calls.append((service_name, required))
            return len(calls) == 2

        self.task.call_camera_service = call_camera_service
        self.assertTrue(self.task.stop_ros_camera_streaming(required=True))
        self.assertEqual(
            calls,
            [
                ("/usb_cam/stop_capture", False),
                ("/usb_cam/stop_capture", False),
            ])
        self.assertFalse(self.task.camera_streaming)
        self.assertIsNone(self.task.latest_camera_image)
        self.assertIsNone(self.task.latest_camera_receipt)

    def test_camera_final_cleanup_kills_exact_node_after_service_failure(self):
        commands = []
        self.task.lock = threading.RLock()
        self.task.camera_streaming = True
        self.task.latest_camera_image = object()
        self.task.latest_camera_receipt = object()
        self.task.camera_node_name = "/usb_cam"
        self.task.camera_open_timeout = 4.0
        self.task.stop_ros_camera_streaming = (
            lambda required=False: False)

        def run_subprocess(command, timeout, context):
            commands.append((command, timeout, context))
            return 0, b"", b""

        self.task.run_subprocess = run_subprocess
        self.assertTrue(self.task.ensure_ros_camera_released())
        self.assertEqual(
            commands,
            [(["rosnode", "kill", "/usb_cam"],
              4.0, "final ROS camera shutdown")])
        self.assertFalse(self.task.camera_streaming)
        self.assertIsNone(self.task.latest_camera_image)
        self.assertIsNone(self.task.latest_camera_receipt)

    def test_ocr_observation_waits_for_cancel_ack_before_stop_gate(self):
        events = []

        class FakeMoveBase(object):
            def __init__(self):
                self.states = [
                    task_module.GoalStatus.ACTIVE,
                    task_module.GoalStatus.PREEMPTED,
                ]

            def cancel_goal(self):
                events.append("cancel")

            def get_state(self):
                events.append("state")
                return self.states.pop(0)

        self.task.move_base = FakeMoveBase()
        self.task.goal_cancel_timeout = 0.5
        self.task.stop_motion = lambda: events.append("zero")
        self.task.require_safe = lambda: None
        self.task.wait_for_chassis_stop = (
            lambda _context: events.append("stop_gate"))
        self.task.cancel_navigation_for_observation("test")
        self.assertEqual(
            events,
            ["state", "cancel", "zero", "state", "stop_gate"])

    def test_ocr_observation_rejects_aborted_navigation(self):
        class FakeMoveBase(object):
            def __init__(self):
                self.states = [
                    task_module.GoalStatus.ACTIVE,
                    task_module.GoalStatus.ABORTED,
                ]

            def cancel_goal(self):
                pass

            def get_state(self):
                return self.states.pop(0)

        self.task.move_base = FakeMoveBase()
        self.task.goal_cancel_timeout = 0.5
        self.task.stop_motion = lambda: None
        self.task.require_safe = lambda: None
        with self.assertRaises(task_module.MissionAbort):
            self.task.cancel_navigation_for_observation("test")

    def test_arrival_scan_without_ocr_records_empty_and_continues(self):
        events = []
        self.task.use_ros_camera_for_ocr = True
        self.task.target_scan_events = []
        self.task.observations = []
        self.task.start_ros_camera_and_wait = (
            lambda label: events.append(("camera_start", label)))
        self.task.stop_ros_camera_streaming = (
            lambda required=True: events.append(("camera_stop", required)))
        self.task.rotate_full_revolution_for_ocr = (
            lambda _label, candidate_handler=None: (None, 2.0 * math.pi))
        self.task.save_observation_summary = lambda: events.append("save")
        self.task.publish_state = lambda state: events.append(("state", state))

        self.assertIsNone(self.task.scan_production_point(1, 52, 12, "target"))
        self.assertEqual(
            self.task.target_scan_events[0]["outcome"],
            "ocr_full_turn_complete")
        self.assertEqual(events[1][0], "camera_start")
        self.assertEqual(events[-1], ("camera_stop", True))

    def test_arrival_scan_observes_from_current_yaw_without_restore(self):
        calls = []
        response = {
            "image_path": "turn.png",
            "capture_requested_at": "now",
            "capture_requested_pose_map": [1.0, 2.0, 0.3],
            "detection": {"text": u"日用品加工车间", "confidence": 91.0},
        }
        self.task.use_ros_camera_for_ocr = False
        self.task.target_scan_events = []
        self.task.observations = []
        self.task.publish_state = lambda _state: None
        def turn_one_circle(_label, candidate_handler=None, **_kwargs):
            self.assertIsNotNone(candidate_handler)
            self.assertTrue(candidate_handler(response, 1.2))
            return None, 2.0 * math.pi

        self.task.rotate_full_revolution_for_ocr = turn_one_circle
        self.task.restore_ocr_capture_yaw = (
            lambda *_args: self.fail("OCR candidate must not restore old yaw"))
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None

        def observe(point_number, _label, **_kwargs):
            calls.append(("observe", point_number))
            return {
                "aligned": True,
                "wall_point_number": 297,
                "wall_point_coordinate": [-0.75, 1.5],
                "text": u"日用品加工车间",
            }

        self.task.observe_wall = observe
        self.task.save_observation_summary = lambda: calls.append("save")
        self.assertIsNone(
            self.task.scan_production_point(1, 52, 12, "target"))

        self.assertEqual(calls[0], ("observe", 12))
        self.assertEqual(calls[1], "save")
        self.assertEqual(calls[2], "save")
        self.assertEqual(
            self.task.observations[0]["turn_detection_pose_map"],
            [1.0, 2.0, 0.3])
        self.assertEqual(self.task.target_scan_events[0]["wall_point_number"], 297)
        self.assertEqual(
            self.task.target_scan_events[0]["outcome"],
            "processing_category_recorded")

    def test_arrival_scan_keeps_turning_until_all_record_categories_found(self):
        calls = []
        responses = [
            {
                "image_path": "real.png",
                "capture_requested_at": "now",
                "capture_requested_pose_map": [1.0, 2.0, 0.3],
                "detection": {
                    "text": u"电子产品生产车间", "confidence": 91.0},
            },
            {
                "image_path": "sim.png",
                "capture_requested_at": "later",
                "capture_requested_pose_map": [1.0, 2.0, 0.5],
                "detection": {
                    "text": u"食品加工车间", "confidence": 92.0},
            },
        ]
        self.task.use_ros_camera_for_ocr = False
        self.task.target_scan_events = []
        self.task.observations = []
        self.task.publish_state = lambda _state: None
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.save_observation_summary = lambda: calls.append("save")

        def observe(point_number, _label, **_kwargs):
            index = len([entry for entry in calls
                         if isinstance(entry, tuple) and
                         entry[0] == "observe"])
            calls.append(("observe", point_number))
            wall_point_number = [168, 297][index]
            return {
                "aligned": True,
                "wall_point_number": wall_point_number,
                "wall_point_coordinate": [-0.75, 1.5],
                "text": responses[index]["detection"]["text"],
            }

        self.task.observe_wall = observe

        def turn_one_circle(_label, candidate_handler=None, **_kwargs):
            self.assertIsNotNone(candidate_handler)
            self.assertTrue(candidate_handler(responses[0], 0.2))
            self.assertFalse(self.task._ocr_turn_stop_flag)
            self.assertTrue(candidate_handler(responses[1], 0.4))
            self.assertTrue(self.task._ocr_turn_stop_flag)
            return None, 0.4

        self.task.rotate_full_revolution_for_ocr = turn_one_circle
        self.assertIsNone(self.task.scan_production_point(
            1, 52, 12, "target", target_category=u"电子产品",
            record_categories=set([u"电子产品", u"食品"])))
        self.assertEqual(
            [entry for entry in calls if isinstance(entry, tuple)],
            [("observe", 12), ("observe", 12)])
        self.assertEqual(len(self.task.observations), 2)

    def test_arrival_scan_skips_repeated_target_category_after_first_candidate(self):
        calls = []
        responses = [
            {
                "image_path": "electronic-1.png",
                "capture_requested_at": "now",
                "capture_requested_pose_map": [1.0, 2.0, 0.3],
                "detection": {
                    "text": u"电子产品生产车间", "confidence": 91.0},
            },
            {
                "image_path": "electronic-2.png",
                "capture_requested_at": "later",
                "capture_requested_pose_map": [1.0, 2.0, 0.4],
                "detection": {
                    "text": u"电子产品生产车间", "confidence": 91.0},
            },
            {
                "image_path": "food.png",
                "capture_requested_at": "latest",
                "capture_requested_pose_map": [1.0, 2.0, 0.5],
                "detection": {
                    "text": u"食品加工车间", "confidence": 92.0},
            },
        ]
        self.task.use_ros_camera_for_ocr = False
        self.task.target_scan_events = []
        self.task.observations = []
        self.task.publish_state = lambda _state: None
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.save_observation_summary = lambda: calls.append("save")

        def observe(point_number, _label, **_kwargs):
            index = len([entry for entry in calls
                         if isinstance(entry, tuple) and
                         entry[0] == "observe"])
            calls.append(("observe", point_number))
            return {
                "aligned": True,
                "wall_point_number": [168, 168, 297][index],
                "wall_point_coordinate": [-0.75, 1.5],
                "text": responses[index]["detection"]["text"],
            }

        self.task.observe_wall = observe

        def turn_one_circle(_label, candidate_handler=None, **_kwargs):
            self.assertIsNotNone(candidate_handler)
            self.assertTrue(candidate_handler(responses[0], 0.2))
            self.assertFalse(candidate_handler(responses[1], 0.3))
            self.assertFalse(self.task._ocr_turn_stop_flag)
            self.assertTrue(candidate_handler(responses[2], 0.4))
            self.assertTrue(self.task._ocr_turn_stop_flag)
            return None, 0.4

        self.task.rotate_full_revolution_for_ocr = turn_one_circle
        self.assertIsNone(self.task.scan_production_point(
            1, 52, 12, "target", target_category=u"电子产品",
            record_categories=set([u"电子产品", u"食品"])))
        self.assertEqual(
            [entry for entry in calls if isinstance(entry, tuple)],
            [("observe", 12), ("observe", 12)])
        self.assertEqual(len(self.task.observations), 2)

    def test_rejected_candidate_is_not_reprocessed_in_same_turn(self):
        calls = []
        response = {
            "image_path": "turn.png",
            "capture_requested_at": "now",
            "capture_requested_pose_map": [1.0, 2.0, 0.3],
            "detection": {
                "text": u"电子产品生产车间", "confidence": 91.0},
        }
        self.task.use_ros_camera_for_ocr = False
        self.task.target_scan_events = []
        self.task.observations = []
        self.task.publish_state = lambda _state: None

        def turn_one_circle(_label, candidate_handler=None):
            self.assertTrue(candidate_handler(response, 1.2))
            self.assertFalse(candidate_handler(response, 1.3))
            return None, 2.0 * math.pi

        self.task.rotate_full_revolution_for_ocr = turn_one_circle
        self.task.restore_ocr_capture_yaw = (
            lambda *_args: self.fail("OCR candidate must not restore old yaw"))
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.observe_wall = (
            lambda _point, _label: (
                calls.append("observe"), {
                    "aligned": False,
                    "wall_point_number": None,
                    "text": u"电子产品生产车间",
                })[1])
        self.task.save_observation_summary = lambda: calls.append("save")

        self.assertIsNone(self.task.scan_production_point(
            1, 52, 12, "target", target_category=u"电子产品",
            record_categories=set([u"电子产品"])))
        self.assertEqual(calls, ["observe", "save", "save"])
        self.assertEqual(
            self.task.target_scan_events[0]["outcome"],
            "processing_category_rejected")
        self.assertEqual(
            self.task.target_scan_events[1]["outcome"],
            "ocr_full_turn_complete")

    def test_cancel_race_returns_succeeded_to_caller(self):
        class FakeMoveBase(object):
            def __init__(self):
                self.states = [
                    task_module.GoalStatus.ACTIVE,
                    task_module.GoalStatus.SUCCEEDED,
                ]

            def cancel_goal(self):
                pass

            def get_state(self):
                return self.states.pop(0)

        self.task.move_base = FakeMoveBase()
        self.task.goal_cancel_timeout = 0.5
        self.task.stop_motion = lambda: None
        self.task.require_safe = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.assertEqual(
            self.task.cancel_navigation_for_observation("test"),
            task_module.GoalStatus.SUCCEEDED)

    def test_moving_ocr_request_is_asynchronous(self):
        release = threading.Event()
        self.task.current_map_pose = lambda _context: (1.0, 2.0, 0.5)

        def delayed_capture(_label, _attempt):
            release.wait(1.0)
            return {"ok": True, "image_path": "unused.png"}

        self.task.capture_ocr = delayed_capture
        self.task.ocr_capture_timeout = 1.0
        task = self.task.start_async_motion_ocr("test")
        self.assertFalse(task["done"].is_set())
        release.set()
        self.assertEqual(
            self.task.finish_async_motion_ocr(task)["image_path"],
            "unused.png")
        self.assertEqual(
            task["capture_requested_pose_map"], [1.0, 2.0, 0.5])

    def test_alignment_ocr_keeps_publishing_turn_command_until_frame_returns(self):
        class Publisher(object):
            def __init__(self):
                self.commands = []

            def publish(self, message):
                self.commands.append(message.angular.z)

        task = {"done": threading.Event()}
        publisher = Publisher()
        original_rate = task_module.rospy.Rate

        class FakeRate(object):
            def __init__(self, _hz):
                pass

            def sleep(self):
                task["done"].set()

        try:
            task_module.rospy.Rate = FakeRate
            self.task.rotation_control_rate = 20.0
            self.task.ocr_alignment_min_speed = 0.12
            self.task.ocr_capture_timeout = 1.0
            self.task.cmd_vel_pub = publisher
            self.task.require_safe = lambda: None
            self.task.start_async_motion_ocr = lambda _label: task
            response = {"ok": True, "image_path": "moving.png"}
            self.task.finish_async_motion_ocr = lambda _task: response

            self.assertEqual(
                self.task.capture_ocr_while_turning(
                    0.05, "test", 2), response)
        finally:
            task_module.rospy.Rate = original_rate

        # 0.05 rad/s is lifted over the chassis dead zone and no zero-speed
        # command is sent until the caller decides the OCR alignment is done.
        self.assertEqual(publisher.commands, [0.12, 0.12])

    def test_alignment_uses_measured_yaw_with_mirrored_ocr_bbox(self):
        calls = []
        self.task.ocr_alignment_max_speed = 0.22
        self.task.ocr_alignment_kp = 0.0025
        self.task.ocr_alignment_kd = 0.00035
        self.task.ocr_alignment_step_seconds = 0.30
        self.task.camera_mirror = True
        self.task.rotate_in_place_for_yaw = (
            lambda speed, delta, context: calls.append(
                (speed, delta, context)))
        self.task.rotate_for_pixel_error(-40.0, "test")
        self.assertEqual(len(calls), 1)
        self.assertAlmostEqual(calls[0][0], 0.10)
        self.assertAlmostEqual(calls[0][1], 0.03)
        self.assertEqual(calls[0][2], "test protected alignment")

    def test_alignment_timeout_settle_accepts_final_positive_odom_sample(self):
        self._assert_alignment_timeout_settle(1.0, 0.030)

    def test_alignment_timeout_settle_accepts_final_negative_odom_sample(self):
        self._assert_alignment_timeout_settle(-1.0, -0.030)

    def test_alignment_timeout_settle_rejects_insufficient_final_odom_sample(self):
        calls = self._configure_timeout_settle_rotation(1.0, 0.028)
        with self.assertRaises(task_module.MissionAbort) as raised:
            self.task.rotate_in_place_for_yaw(0.13, 0.039, "test")
        self.assertIn("actual=0.028", str(raised.exception))
        self.assertIn("required=0.029", str(raised.exception))
        self.assertEqual(calls["wait"], [
            "test start", "test timeout settle"])
        self.assertGreaterEqual(calls["stops"], 2)

    def _assert_alignment_timeout_settle(self, direction, final_yaw):
        calls = self._configure_timeout_settle_rotation(direction, final_yaw)
        self.task.rotate_in_place_for_yaw(
            0.13 * direction, 0.039, "test")
        self.assertEqual(calls["wait"], [
            "test start", "test timeout settle"])
        self.assertGreaterEqual(calls["stops"], 2)
        self.assertEqual(len(calls["commands"]), 1)

    def _configure_timeout_settle_rotation(self, direction, final_yaw):
        class FakeTime(object):
            now_value = 0.0

            def __init__(self, value):
                self.value = value

            @classmethod
            def now(cls):
                return cls(cls.now_value)

            def __add__(self, duration):
                return FakeTime(self.value + duration)

            def __lt__(self, other):
                return self.value < other.value

        class FakeRate(object):
            def __init__(self, _hz):
                pass

            def sleep(self):
                FakeTime.now_value += 0.3

        class FakeMoveBase(object):
            def cancel_all_goals(self):
                pass

        class FakePublisher(object):
            def __init__(self, commands):
                self.commands = commands

            def publish(self, message):
                self.commands.append(message.angular.z)

        original_time = task_module.rospy.Time
        original_duration = task_module.rospy.Duration
        original_rate = task_module.rospy.Rate
        original_is_shutdown = task_module.rospy.is_shutdown
        yaws = [0.0, 0.028 * direction, final_yaw]
        calls = {"wait": [], "commands": [], "stops": 0}
        self.task.move_base = FakeMoveBase()
        self.task.ocr_alignment_yaw_tolerance = 0.01
        self.task.ocr_alignment_turn_timeout = 0.2
        self.task.rotation_control_rate = 20.0
        self.task.stop_motion = lambda: calls.__setitem__(
            "stops", calls["stops"] + 1)
        self.task.wait_for_chassis_stop = lambda context: calls["wait"].append(
            context)
        self.task.current_odom_yaw = lambda _context: yaws.pop(0)
        self.task.require_safe = lambda: None
        self.task.cmd_vel_pub = FakePublisher(calls["commands"])
        task_module.rospy.Time = FakeTime
        task_module.rospy.Duration = lambda seconds: seconds
        task_module.rospy.Rate = FakeRate
        task_module.rospy.is_shutdown = lambda: False
        self.addCleanup(setattr, task_module.rospy, "Time", original_time)
        self.addCleanup(setattr, task_module.rospy, "Duration", original_duration)
        self.addCleanup(setattr, task_module.rospy, "Rate", original_rate)
        self.addCleanup(
            setattr, task_module.rospy, "is_shutdown", original_is_shutdown)
        return calls


@unittest.skipIf(
    task_module is None,
    "ROS Python modules are only available in the vehicle workspace")
class ProductionTaskDualItemTest(unittest.TestCase):
    def setUp(self):
        task_module.rospy.rostime.set_rostime_initialized(True)
        self.task = object.__new__(task_module.ProductionTask2026)
        self.task.points = {16: (0.25, 0.75)}
        self.task.arrival_tolerance = 0.10
        self.task.current_map_pose = lambda _context: (0.18, 0.826, 0.0)
        self.warnings = []
        self.original_logwarn = task_module.rospy.logwarn
        self.original_loginfo = task_module.rospy.loginfo
        task_module.rospy.logwarn = self.capture_warning
        task_module.rospy.loginfo = lambda *_args: None
        self.task.qr_classifications = []
        self.task.observations = []
        self.task.expected_item_text = u""
        self.task.expected_production_category = None
        self.task.expected_real_item_text = u""
        self.task.expected_sim_item_text = u""
        self.task.expected_real_category = None
        self.task.expected_sim_category = None
        self.task.served_wall_points = set()
        self.task._ocr_turn_stop_flag = False
        self.task.processing_dwell_seconds = 0.0
        self.task.middle_zone_square_side = 0.5
        self.task.ocr_stop_offset_m = 0.25
        self.task.middle_zone_bounds = (-2.5, 2.5, -0.5, 1.5)
        self.task.ocr_alignment_min_speed = 0.12
        self.task.spark_classify_enabled = False
        self.task.tts_enabled = False

    def tearDown(self):
        task_module.rospy.logwarn = self.original_logwarn
        task_module.rospy.loginfo = self.original_loginfo

    def capture_warning(self, message, *args):
        self.warnings.append(message % args)

    def _patch_stdin_lines(self, lines):
        """Swap sys.stdin for the given lines and sys.stdout to devnull."""
        iterator = iter(lines)

        class FakeStdin(object):
            def readline(self):
                return next(iterator)

        original_stdin = task_module.sys.stdin
        original_stdout = task_module.sys.stdout
        task_module.sys.stdin = FakeStdin()
        task_module.sys.stdout = open(os.devnull, "w")

        def restore():
            task_module.sys.stdout.close()
            task_module.sys.stdin = original_stdin
            task_module.sys.stdout = original_stdout

        return restore

    def test_wait_for_item_inputs_reads_two_distinct_items(self):
        restore = self._patch_stdin_lines([u"苹果\n", u"手机\n"])
        try:
            real_item, sim_item = self.task.wait_for_item_inputs()
        finally:
            restore()
        self.assertEqual((real_item, sim_item), (u"苹果", u"手机"))
        self.assertEqual(self.task.expected_real_item_text, u"苹果")
        self.assertEqual(self.task.expected_sim_item_text, u"手机")
        self.assertEqual(self.task.expected_item_text, u"苹果")

    def test_wait_for_item_inputs_rejects_empty_first_item(self):
        restore = self._patch_stdin_lines([u"\n", u"手机\n"])
        try:
            with self.assertRaises(task_module.MissionAbort) as raised:
                self.task.wait_for_item_inputs()
        finally:
            restore()
        self.assertIn("no item text was provided", str(raised.exception))

    def test_wait_for_item_inputs_rejects_identical_items(self):
        restore = self._patch_stdin_lines([u"苹果\n", u"苹果\n"])
        try:
            with self.assertRaises(task_module.MissionAbort) as raised:
                self.task.wait_for_item_inputs()
        finally:
            restore()
        self.assertIn("real and simulation items must be different",
                      str(raised.exception))

    def test_voice_item_inputs_store_requested_categories(self):
        self.task.item_input_mode = "voice"
        self.task.wait_for_voice_item_categories = (
            lambda: (u"日用品", u"食品"))
        real_category, sim_category = self.task.wait_for_item_inputs()
        self.assertEqual((real_category, sim_category), (u"日用品", u"食品"))
        self.assertEqual(self.task.requested_real_category, u"日用品")
        self.assertEqual(self.task.requested_sim_category, u"食品")
        # The actual item names are intentionally deferred until QR matching.
        self.assertEqual(self.task.expected_real_item_text, u"")
        self.assertEqual(self.task.expected_sim_item_text, u"")

    def test_parse_voice_listener_message_accepts_complete_json_slots(self):
        line = json.dumps({
            "ok": True,
            "slots": {u"取件类别": u"日用品", u"仿真类别": u"食品"},
        }, ensure_ascii=True)
        self.assertEqual(
            self.task.parse_voice_listener_message(line),
            (u"日用品", u"食品"))

    def test_voice_qr_collection_resolves_category_to_actual_item(self):
        self.task.qr_observation_numbers = [262, 232, 295]
        detected_items = iter([u"牙刷", u"蛋糕"])
        category_for_item = {u"牙刷": u"日用品", u"蛋糕": u"食品"}

        def scan(observation_number, accept_text=None,
                 allow_revolution=True):
            item = next(detected_items)
            if accept_text is not None and not accept_text(item):
                return None
            return item

        def classify(observation_number, item_text):
            if isinstance(item_text, bytes):
                item_text = item_text.decode("utf-8")
            self.task.qr_classifications.append({
                "observation": observation_number,
                "qr_text": item_text,
                "category": category_for_item[item_text],
            })

        self.task.scan_observation_point = scan
        self.task.classify_qr_text = classify
        collected = self.task.collect_target_qr_codes_by_category(
            set([u"日用品", u"食品"]))
        self.assertEqual(
            collected,
            {
                u"日用品": {"item": u"牙刷", "observation": 262},
                u"食品": {"item": u"蛋糕", "observation": 232},
            })

    def test_classifier_ignores_late_response_from_previous_request(self):
        class FakeStdin(object):
            def __init__(self):
                self.payloads = []

            def write(self, payload):
                self.payloads.append(payload)

            def flush(self):
                pass

        class FakeStdout(object):
            def __init__(self, lines):
                self.lines = iter(lines)

            def readline(self):
                try:
                    return next(self.lines)
                except StopIteration:
                    return b""

        class FakeProcess(object):
            def __init__(self):
                self.stdin = FakeStdin()
                self.stdout = FakeStdout([
                    json.dumps({
                        "request_id": 1,
                        "category": u"电子产品",
                        "source": "spark",
                        "attempts": 3,
                        "model": "spark-x",
                        "error": "",
                    }, ensure_ascii=True).encode("utf-8") + b"\n",
                    json.dumps({
                        "request_id": 2,
                        "category": u"日用品",
                        "source": "local",
                        "attempts": 0,
                        "model": "spark-x",
                        "error": "",
                    }, ensure_ascii=True).encode("utf-8") + b"\n",
                ])

            def poll(self):
                return None

        process = FakeProcess()
        self.task.spark_classify_enabled = True
        self.task.spark_request_sequence = 1
        self.task.spark_process = process
        self.task.spark_log_handle = None
        self.task.spark_timeout = 0.5
        self.task.spark_model = "spark-x"
        self.task.start_qr_classifier = lambda: True
        self.task.stop_qr_classifier = lambda: None
        original_select = task_module.select.select
        task_module.select.select = (
            lambda *_args: ([process.stdout], [], []))
        try:
            self.task.classify_qr_text(295, u"毛巾".encode("utf-8"))
        finally:
            task_module.select.select = original_select

        sent = json.loads(process.stdin.payloads[0].decode("utf-8"))
        self.assertEqual(sent["request_id"], 2)
        self.assertEqual(self.task.qr_classifications[-1]["category"],
                         u"日用品")

    def test_voice_qr_collection_retries_after_unclassified_item(self):
        self.task.qr_observation_numbers = [262, 232]
        self.task.used_qr_codes = set()
        detections = iter([u"耳机", u"耳机"])
        categories = iter([None, u"电子产品"])

        def scan(_observation_number, accept_text=None,
                 allow_revolution=True):
            item = next(detections)
            if accept_text is not None and not accept_text(item):
                return None
            self.task.used_qr_codes.add(item)
            return item

        def classify(observation_number, item_text):
            if isinstance(item_text, bytes):
                item_text = item_text.decode("utf-8")
            self.task.qr_classifications.append({
                "observation": observation_number,
                "qr_text": item_text,
                "category": next(categories),
            })

        self.task.scan_observation_point = scan
        self.task.classify_qr_text = classify
        collected = self.task.collect_target_qr_codes_by_category(
            set([u"电子产品"]), rounds=1)
        self.assertEqual(
            collected,
            {u"电子产品": {"item": u"耳机", "observation": 232}})
    def test_qr_collection_filters_targets_and_takes_first_only(self):
        self.task.qr_observation_numbers = [262, 232, 295]
        faces = []
        codes = iter([u"苹果", u"无关码", u"苹果", u"手机"])

        def scan(observation_number, accept_text=None,
                 allow_revolution=True):
            faces.append(observation_number)
            code = next(codes)
            if accept_text is not None and not accept_text(code):
                return None
            return code

        self.task.scan_observation_point = scan
        collected = self.task.collect_target_qr_codes(
            set([u"苹果", u"手机"]))

        # The non-target code is skipped, the repeated 苹果 keeps its first
        # face, and the scan stops as soon as both targets are collected.
        # Stage 1 faces 262 (collects 苹果), 232 (rejected), 295 (already
        # collected), then stage 2 faces 262 again and collects 手机.
        self.assertEqual(collected, {u"苹果": 262, u"手机": 262})
        self.assertEqual(faces, [262, 232, 295, 262])

    def test_qr_collection_scans_two_full_rounds_when_targets_missing(self):
        self.task.qr_observation_numbers = [262, 232, 295]
        faces = []

        def scan(observation_number, accept_text=None,
                 allow_revolution=True):
            faces.append(observation_number)
            return None

        self.task.scan_observation_point = scan
        collected = self.task.collect_target_qr_codes(
            set([u"苹果", u"手机"]))
        self.assertEqual(collected, {})
        # Nothing found in either stage, so both rounds run fully: 3 faces
        # without revolution + 3 revolution fallbacks per round.
        self.assertEqual(faces, [
            262, 232, 295, 262, 232, 295,
            262, 232, 295, 262, 232, 295])

    def test_qr_collection_faces_all_directions_before_revolution(self):
        self.task.qr_observation_numbers = [262, 232, 295]
        calls = []

        def scan(observation_number, accept_text=None,
                 allow_revolution=True):
            calls.append((observation_number, allow_revolution))
            return None

        self.task.scan_observation_point = scan
        collected = self.task.collect_target_qr_codes(
            set([u"苹果", u"手机"]))
        self.assertEqual(collected, {})
        # Stage 1 always faces all three directions without revolution
        # before stage 2 applies the revolution fallback, in both rounds.
        self.assertEqual(calls, [
            (262, False), (232, False), (295, False),
            (262, True), (232, True), (295, True),
            (262, False), (232, False), (295, False),
            (262, True), (232, True), (295, True)])

    def test_run_mission_aborts_when_qr_codes_not_all_collected(self):
        class MoveBase(object):
            def wait_for_server(self, _timeout):
                return True

            def cancel_all_goals(self):
                pass

        class QrPublisher(object):
            def publish(self, _message):
                pass

        self.task.move_base = MoveBase()
        self.task.move_base_ready_timeout = 1.0
        self.task.publish_state = lambda _state: None
        self.task.wait_for_item_inputs = (
            lambda: (u"苹果", u"手机"))
        self.task.wait_for_safe_start = lambda: None
        self.task.switch_to_point_mode = lambda: None
        self.task.switch_to_destination_mode = lambda: None
        self.task.resume_production_only = False
        self.task.staging_point_number = 52
        self.task.qr_observation_numbers = [262]
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.wait_for_qr_scanner = lambda: None
        self.task.start_ros_camera_and_wait = lambda _context: None
        self.task.qr_enable_pub = QrPublisher()
        self.task.navigate_to = lambda *_args, **_kwargs: None
        self.task.prepare_result_directory = lambda: None
        self.task.scan_observation_point = (
            lambda _observation_number, _accept_text=None,
                   allow_revolution=True: None)

        with self.assertRaises(task_module.MissionAbort) as raised:
            self.task.run_mission()
        self.assertIn("not all target QR codes", str(raised.exception))

    def test_run_mission_records_simulation_first_but_announces_real_first(self):
        events = []

        class MoveBase(object):
            def wait_for_server(self, _timeout):
                return True

            def cancel_all_goals(self):
                pass

        class QrPublisher(object):
            def publish(self, _message):
                pass

        self.task.move_base = MoveBase()
        self.task.move_base_ready_timeout = 1.0
        self.task.publish_state = lambda _state: None
        self.task.wait_for_item_inputs = (
            lambda: (events.append("item_input"), (u"苹果", u"手机"))[1])
        self.task.wait_for_safe_start = lambda: None
        self.task.switch_to_point_mode = lambda: None
        self.task.switch_to_destination_mode = lambda: None
        self.task.resume_production_only = False
        self.task.staging_point_number = 52
        self.task.qr_observation_numbers = [262]
        self.task.points = {
            52: (-1.75, 2.25), 262: (-2.50, 2.25),
            12: (-1.75, 0.75), 13: (-1.75, 0.75), 14: (-1.75, 0.75),
            170: (1.0, 0.0), 319: (1.0, 1.0),
        }
        self.task.destination_midpoint_point_numbers = []
        self.task.destination_point_number = 170
        self.task.destination_heading_point_number = 319
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.wait_for_qr_scanner = lambda: None
        self.task.start_ros_camera_and_wait = lambda _context: None
        self.task.qr_enable_pub = QrPublisher()
        self.task.navigate_to = lambda *_args, **_kwargs: None
        self.task.collect_target_qr_codes = (
            lambda targets: {u"苹果": 262, u"手机": 262})
        self.task.stop_qr_classifier = lambda: None
        self.task.stop_ros_camera_streaming = lambda required=True: None
        self.task.post_qr_waypoint_number = 0

        def classify_qr_text(observation_number, qr_text):
            item = qr_text.decode("utf-8")
            self.task.qr_classifications.append({
                "observation": observation_number,
                "qr_text": item,
                "category": (
                    u"日用品" if item == u"苹果" else u"电子产品"),
                "source": "stub",
                "attempts": 0,
                "model": "test",
                "error": "",
            })

        self.task.classify_qr_text = classify_qr_text
        self.task.prepare_result_directory = lambda: None
        self.task.use_ros_camera_for_ocr = True
        self.task.camera_image_topic = "/usb_cam/image_raw"
        self.task.start_native_ocr = lambda: None
        self.task.production_route_numbers = [12, 13, 14]
        self.task.production_navigation_legs = [
            (52, 12), (12, 13), (13, 14)]
        self.task.production_observation_headings = [0.0, 0.0, 0.0]
        self.task.fallback_navigation_legs = [(14, 12)]
        self.task.fallback_production_observation_headings = [0.0]
        self.task.observations = []

        def navigate_target_and_scan(leg_index, start_number, end_number,
                                     target_yaw, target_category=None,
                                     record_categories=None, **_kwargs):
            events.append((
                "nav", leg_index, start_number, end_number,
                target_category, record_categories))
            # The simulation workshop is seen first.  The route must record
            # it, but cannot park there until after the real-item announcement.
            if end_number == 12:
                self.task.observations.append({
                    "processing_category": u"电子产品",
                    "wall_point_number": 200,
                    "wall_point_coordinate": [1.0, 1.5],
                    "forward_ray_wall_intersection_map": [1.0, 1.5],
                    "measured_wall_hit_map": [1.0, 1.5],
                })
            if end_number == 13:
                self.task.observations.append({
                    "processing_category": u"日用品",
                    "wall_point_number": 100,
                    "wall_point_coordinate": [0.0, 1.5],
                    "forward_ray_wall_intersection_map": [0.0, 1.5],
                    "measured_wall_hit_map": [0.0, 1.5],
                })

        self.task.navigate_target_and_scan = navigate_target_and_scan

        def navigate_coordinates(x_value, y_value, yaw, label,
                                 require_plan, **_kwargs):
            events.append(("nav_coord", label))
            return True

        self.task.navigate_coordinates = navigate_coordinates
        self.task.speak_wait = (
            lambda text, timeout=None: events.append(("announce", text)))
        self.task.stop_native_ocr = lambda: events.append("ocr_stop")
        self.task.ensure_ros_camera_released = (
            lambda: events.append("camera_release"))
        self.task.save_observation_summary = lambda: events.append("save")
        self.task.simulation_request_start = (
            lambda item, category: (
                events.append(("sim_start", item, category)), False)[1])
        self.task.simulation_wait_done = (
            lambda: (events.append("sim_wait_timeout"), False)[1])
        self.task.simulation_done_timeout = 75.0
        self.task.publish_result = (
            lambda success, reason: events.append(("result", success)))
        self.task.lane_handoff_enabled = False

        # The real run_mission ends with rospy.signal_shutdown() to hand the
        # vehicle over to lane_proto.  In the shared nosetests process that
        # would set the global rospy shutdown flag and break every later
        # test that waits on rospy.is_shutdown(); stub it out here.
        original_signal_shutdown = task_module.rospy.signal_shutdown
        task_module.rospy.signal_shutdown = (
            lambda _reason: events.append("signal_shutdown"))
        try:
            self.task.run_mission()
        finally:
            task_module.rospy.signal_shutdown = original_signal_shutdown

        self.assertEqual(events, [
            "item_input",
            ("announce", u"取得*苹果*属于*日用品*应放置在*日用品加工车间"),
            ("announce", u"仿真环境中取得*手机*属于*电子产品*应放置在*电子产品生产车间"),
            ("nav", 1, 52, 12, u"日用品", set([u"日用品", u"电子产品"])),
            ("nav", 2, 12, 13, u"日用品", set([u"日用品", u"电子产品"])),
            ("nav_coord", "processing stop point 100"),
            ("announce", u"已将苹果放入日用品"),
            ("nav_coord", "processing stop point 200"),
            "ocr_stop",
            "camera_release",
            "save",
            ("sim_start", u"手机", u"电子产品"),
            "sim_wait_timeout",
            ("announce", u"仿真任务已完成，已将手机放入电子产品"),
            ("nav_coord", "destination point 170"),
            ("result", True),
            "signal_shutdown",
        ])

    def test_run_mission_continues_to_destination_when_fallback_misses_sim(self):
        events = []
        class MoveBase(object):
            def wait_for_server(self, _timeout):
                return True

            def cancel_all_goals(self):
                pass

        class QrPublisher(object):
            def publish(self, _message):
                pass

        self.task.move_base = MoveBase()
        self.task.move_base_ready_timeout = 1.0
        self.task.publish_state = lambda _state: None
        self.task.wait_for_item_inputs = (
            lambda: (u"苹果", u"手机"))
        self.task.wait_for_safe_start = lambda: None
        self.task.switch_to_point_mode = lambda: None
        self.task.resume_production_only = False
        self.task.staging_point_number = 52
        self.task.qr_observation_numbers = [262]
        self.task.points = {
            52: (-1.75, 2.25), 262: (-2.50, 2.25),
            12: (-1.75, 0.75), 13: (-1.25, 0.75), 14: (-0.75, 0.75),
            170: (1.0, 0.0), 319: (1.0, 1.0),
        }
        self.task.destination_midpoint_point_numbers = []
        self.task.destination_point_number = 170
        self.task.destination_heading_point_number = 319
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.wait_for_qr_scanner = lambda: None
        self.task.start_ros_camera_and_wait = lambda _context: None
        self.task.navigate_to = lambda *_args, **_kwargs: None
        self.task.collect_target_qr_codes = (
            lambda targets: {u"苹果": 262, u"手机": 262})
        self.task.qr_enable_pub = QrPublisher()
        self.task.stop_qr_classifier = lambda: None
        self.task.stop_ros_camera_streaming = lambda required=True: None
        self.task.post_qr_waypoint_number = 0
        self.task.prepare_result_directory = lambda: None
        self.task.use_ros_camera_for_ocr = True
        self.task.camera_image_topic = "/usb_cam/image_raw"
        self.task.start_native_ocr = lambda: None
        self.task.production_route_numbers = [12, 13]
        self.task.production_navigation_legs = [(52, 12), (12, 13)]
        self.task.production_observation_headings = [0.0, 0.0]
        self.task.fallback_navigation_legs = [(13, 14)]
        self.task.fallback_production_observation_headings = [0.0]
        self.task.observations = []

        def classify_qr_text(observation_number, qr_text):
            item = qr_text.decode("utf-8")
            self.task.qr_classifications.append({
                "observation": observation_number,
                "qr_text": item,
                "category": (
                    u"日用品" if item == u"苹果" else u"电子产品"),
                "source": "stub",
                "attempts": 0,
                "model": "test",
                "error": "",
            })

        self.task.classify_qr_text = classify_qr_text

        def navigate_target_and_scan(leg_index, start_number, end_number,
                                     target_yaw, target_category=None,
                                     record_categories=None, **_kwargs):
            events.append(("scan", start_number, end_number,
                           target_category, record_categories))
            # The real item is found on the very last leg.
            if end_number == 13:
                self.task.observations.append({
                    "processing_category": u"日用品",
                    "wall_point_number": 100,
                    "wall_point_coordinate": [0.0, 1.5],
                    "forward_ray_wall_intersection_map": [0.0, 1.5],
                    "measured_wall_hit_map": [0.0, 1.5],
                })

        self.task.navigate_target_and_scan = navigate_target_and_scan
        self.task.navigate_coordinates = (
            lambda _x, _y, _yaw, label, **_kwargs:
            (events.append(("navigate", label)), True)[1])
        self.task.speak_wait = lambda text, timeout=None: None
        self.task.stop_native_ocr = lambda: events.append("ocr_stop")
        self.task.ensure_ros_camera_released = (
            lambda: events.append("camera_release"))
        self.task.save_observation_summary = lambda: events.append("save")
        self.task.publish_result = (
            lambda success, reason: events.append(("result", success, reason)))
        self.task.lane_handoff_enabled = False
        original_signal_shutdown = task_module.rospy.signal_shutdown
        task_module.rospy.signal_shutdown = (
            lambda _reason: events.append("signal_shutdown"))
        try:
            self.task.run_mission()
        finally:
            task_module.rospy.signal_shutdown = original_signal_shutdown

        self.assertIn(
            ("scan", 13, 14, u"电子产品", set([u"电子产品"])), events)
        self.assertIn(("navigate", "destination point 170"), events)
        self.assertTrue(any(
            event[0] == "result" and event[1] is True and
            "production routes exhausted" in event[2]
            for event in events))

    def test_speak_wait_timeout_terminates_helper_and_continues(self):
        calls = []

        class FakeProcess(object):
            def __init__(self):
                self.returncode = None

            def poll(self):
                return None

            def terminate(self):
                calls.append("terminate")

            def kill(self):
                calls.append("kill")

        self.task.tts_enabled = True
        self.task.tts_python = "/usr/bin/python3"
        self.task.tts_helper_path = "/home/ucar/wake/tts_say.py"
        self.task.speak_wait_timeout = 0.2
        original_popen = task_module.subprocess.Popen
        original_is_shutdown = task_module.rospy.is_shutdown
        task_module.subprocess.Popen = (
            lambda *_args, **_kwargs: FakeProcess())
        task_module.rospy.is_shutdown = lambda: False
        try:
            self.task.speak_wait(u"测试播报")
        finally:
            task_module.subprocess.Popen = original_popen
            task_module.rospy.is_shutdown = original_is_shutdown
        self.assertIn("terminate", calls)
        self.assertIn("kill", calls)
        self.assertTrue(any(
            "PRODUCTION_TASK_TTS_TIMEOUT" in warning
            for warning in self.warnings))

    def test_speak_wait_returns_after_helper_exits(self):
        calls = []

        class FakeProcess(object):
            def __init__(self):
                self.returncode = 0
                self.poll_count = 0

            def poll(self):
                self.poll_count += 1
                if self.poll_count >= 2:
                    return 0
                return None

            def terminate(self):
                calls.append("terminate")

            def kill(self):
                calls.append("kill")

        self.task.tts_enabled = True
        self.task.tts_python = "/usr/bin/python3"
        self.task.tts_helper_path = "/home/ucar/wake/tts_say.py"
        self.task.speak_wait_timeout = 5.0
        original_popen = task_module.subprocess.Popen
        original_is_shutdown = task_module.rospy.is_shutdown
        task_module.subprocess.Popen = (
            lambda *_args, **_kwargs: FakeProcess())
        task_module.rospy.is_shutdown = lambda: False
        try:
            self.task.speak_wait(u"测试播报")
        finally:
            task_module.subprocess.Popen = original_popen
            task_module.rospy.is_shutdown = original_is_shutdown
        self.assertEqual(calls, [])
        self.assertFalse(any(
            "PRODUCTION_TASK_TTS_TIMEOUT" in warning
            for warning in self.warnings))

    def test_resolve_simulation_host_requires_explicit_value(self):
        self.task.simulation_host = ""
        with self.assertRaises(task_module.MissionAbort):
            self.task.resolve_simulation_host()

    def test_resolve_simulation_host_explicit_value_wins(self):
        self.task.simulation_host = "10.0.0.2"
        self.assertEqual(self.task.resolve_simulation_host(), "10.0.0.2")

    def test_resolve_simulation_host_aborts_without_source(self):
        self.task.simulation_host = ""
        original = os.environ.get("ROS_MASTER_URI")
        os.environ.pop("ROS_MASTER_URI", None)
        try:
            with self.assertRaises(task_module.MissionAbort):
                self.task.resolve_simulation_host()
        finally:
            if original is not None:
                os.environ["ROS_MASTER_URI"] = original

    def test_simulation_request_start_posts_item_and_category(self):
        self.task.simulation_host = "192.168.1.5"
        self.task.simulation_port = 11313
        self.task.simulation_start_timeout = 1.0
        self.task.simulation_start_retries = 3
        self.task.require_safe = lambda: None

        sent = {}

        def fake_urlopen(request, timeout=None):
            sent["url"] = request.get_full_url()
            sent["body"] = request.get_data()
            sent["content_type"] = request.get_header("Content-type")
            self.assertEqual(timeout, 1.0)

            class Response(object):
                def read(self):
                    return '{"accepted": true}'

                def close(self):
                    pass

            return Response()

        original_urlopen = task_module.urllib2.urlopen
        task_module.urllib2.urlopen = fake_urlopen
        try:
            self.task.simulation_request_start(u"手机", u"电子产品")
        finally:
            task_module.urllib2.urlopen = original_urlopen

        self.assertEqual(sent["url"], "http://192.168.1.5:11313/start")
        self.assertEqual(sent["content_type"], "application/json")
        payload = json.loads(sent["body"].decode("utf-8"))
        self.assertEqual(
            payload, {"item_name": u"手机", "category": u"电子产品"})

    def test_simulation_request_start_409_continues_to_status_wait(self):
        self.task.simulation_host = "192.168.1.5"
        self.task.simulation_port = 11313
        self.task.simulation_start_timeout = 1.0
        self.task.simulation_start_retries = 3
        self.task.require_safe = lambda: None

        def fake_urlopen(request, timeout=None):
            raise task_module.urllib2.HTTPError(
                request.get_full_url(), 409, "Conflict", {}, None)

        original_urlopen = task_module.urllib2.urlopen
        task_module.urllib2.urlopen = fake_urlopen
        try:
            result = self.task.simulation_request_start(u"手机", u"电子产品")
        finally:
            task_module.urllib2.urlopen = original_urlopen
        self.assertFalse(result)
        self.assertEqual(len(self.warnings), 1)
        self.assertIn(
            "PRODUCTION_SIMULATION_START_409_CONTINUE", self.warnings[0])

    def test_simulation_request_start_retries_then_succeeds(self):
        self.task.simulation_host = "192.168.1.5"
        self.task.simulation_port = 11313
        self.task.simulation_start_timeout = 1.0
        self.task.simulation_start_retries = 3
        self.task.require_safe = lambda: None
        attempts = []
        sleeps = []
        original_sleep = task_module.time.sleep
        task_module.time.sleep = lambda seconds: sleeps.append(seconds)

        def fake_urlopen(request, timeout=None):
            attempts.append(1)
            if len(attempts) < 3:
                raise task_module.urllib2.URLError("connection refused")

            class Response(object):
                def read(self):
                    return '{"accepted": true}'

                def close(self):
                    pass

            return Response()

        original_urlopen = task_module.urllib2.urlopen
        task_module.urllib2.urlopen = fake_urlopen
        try:
            self.task.simulation_request_start(u"手机", u"电子产品")
        finally:
            task_module.urllib2.urlopen = original_urlopen
            task_module.time.sleep = original_sleep
        self.assertEqual(len(attempts), 3)
        self.assertEqual(sleeps, [2.0, 2.0])

    def test_simulation_request_start_continues_after_all_retries(self):
        self.task.simulation_host = "192.168.1.5"
        self.task.simulation_port = 11313
        self.task.simulation_start_timeout = 1.0
        self.task.simulation_start_retries = 3
        self.task.require_safe = lambda: None
        attempts = []
        original_sleep = task_module.time.sleep
        task_module.time.sleep = lambda _seconds: None

        def fake_urlopen(request, timeout=None):
            attempts.append(1)
            raise task_module.urllib2.URLError("connection refused")

        original_urlopen = task_module.urllib2.urlopen
        task_module.urllib2.urlopen = fake_urlopen
        try:
            result = self.task.simulation_request_start(u"手机", u"电子产品")
        finally:
            task_module.urllib2.urlopen = original_urlopen
            task_module.time.sleep = original_sleep
        self.assertFalse(result)
        self.assertEqual(len(attempts), 3)
        self.assertTrue(any(
            "PRODUCTION_SIMULATION_START_FAILED_CONTINUE" in warning
            for warning in self.warnings))

    def test_simulation_wait_done_returns_after_done(self):
        self.task.simulation_host = "192.168.1.5"
        self.task.simulation_port = 11313
        self.task.simulation_done_timeout = 10.0
        self.task.simulation_poll_period = 0.01
        self.task.require_safe = lambda: None
        states = iter(['{"state": "running"}', '{"state": "done"}'])
        original_is_shutdown = task_module.rospy.is_shutdown
        task_module.rospy.is_shutdown = lambda: False

        def fake_urlopen(url, timeout=None):
            self.assertEqual(timeout, 10.0)

            class Response(object):
                def read(self):
                    return next(states)

                def close(self):
                    pass

            return Response()

        original_urlopen = task_module.urllib2.urlopen
        task_module.urllib2.urlopen = fake_urlopen
        try:
            self.task.simulation_wait_done()
        finally:
            task_module.urllib2.urlopen = original_urlopen
            task_module.rospy.is_shutdown = original_is_shutdown

    def test_simulation_wait_done_continues_after_failed_state_timeout(self):
        self.task.simulation_host = "192.168.1.5"
        self.task.simulation_port = 11313
        self.task.simulation_done_timeout = 0.03
        self.task.simulation_poll_period = 0.01
        self.task.require_safe = lambda: None
        original_is_shutdown = task_module.rospy.is_shutdown
        task_module.rospy.is_shutdown = lambda: False

        def fake_urlopen(url, timeout=None):
            class Response(object):
                def read(self):
                    return '{"state": "failed", "detail": "sim crashed"}'

                def close(self):
                    pass

            return Response()

        original_urlopen = task_module.urllib2.urlopen
        task_module.urllib2.urlopen = fake_urlopen
        try:
            completed = self.task.simulation_wait_done()
        finally:
            task_module.urllib2.urlopen = original_urlopen
            task_module.rospy.is_shutdown = original_is_shutdown
        self.assertFalse(completed)

    def test_simulation_wait_done_continues_on_timeout(self):
        self.task.simulation_host = "192.168.1.5"
        self.task.simulation_port = 11313
        self.task.simulation_done_timeout = 0.05
        self.task.simulation_poll_period = 0.01
        self.task.require_safe = lambda: None
        original_is_shutdown = task_module.rospy.is_shutdown
        task_module.rospy.is_shutdown = lambda: False

        def fake_urlopen(url, timeout=None):
            class Response(object):
                def read(self):
                    return '{"state": "running"}'

                def close(self):
                    pass

            return Response()

        original_urlopen = task_module.urllib2.urlopen
        task_module.urllib2.urlopen = fake_urlopen
        try:
            completed = self.task.simulation_wait_done()
        finally:
            task_module.urllib2.urlopen = original_urlopen
            task_module.rospy.is_shutdown = original_is_shutdown
        self.assertFalse(completed)

    def test_simulation_wait_done_reconnects_after_bad_status_line(self):
        self.task.simulation_host = "192.168.1.5"
        self.task.simulation_port = 11313
        self.task.simulation_done_timeout = 1.0
        self.task.simulation_poll_period = 0.0
        self.task.require_safe = lambda: None
        attempts = []
        original_is_shutdown = task_module.rospy.is_shutdown
        task_module.rospy.is_shutdown = lambda: False

        def fake_urlopen(url, timeout=None):
            attempts.append((url, timeout))
            if len(attempts) == 1:
                raise task_module.httplib.BadStatusLine(
                    "No status line received")

            class Response(object):
                def read(self):
                    return '{"state": "done"}'

                def close(self):
                    pass

            return Response()

        original_urlopen = task_module.urllib2.urlopen
        task_module.urllib2.urlopen = fake_urlopen
        try:
            self.assertTrue(self.task.simulation_wait_done())
        finally:
            task_module.urllib2.urlopen = original_urlopen
            task_module.rospy.is_shutdown = original_is_shutdown
        self.assertEqual(len(attempts), 2)

    def test_warehouse_name_for_category_matches_workshop_signs(self):
        self.assertEqual(
            self.task.warehouse_name_for_category(u"食品"),
            u"食品加工车间")
        self.assertEqual(
            self.task.warehouse_name_for_category(u"日用品"),
            u"日用品加工车间")
        # The real-field sign reads 生产车间, not 加工车间.
        self.assertEqual(
            self.task.warehouse_name_for_category(u"电子产品"),
            u"电子产品生产车间")

    def test_warehouse_name_for_category_falls_back_with_warning(self):
        name = self.task.warehouse_name_for_category(u"未知类别")
        self.assertEqual(name, u"未知类别加工车间")
        self.assertTrue(any(
            "PRODUCTION_TTS_WAREHOUSE_UNKNOWN" in warning
            for warning in self.warnings))

    def test_handoff_to_lane_disabled_does_not_spawn(self):
        self.task.lane_handoff_enabled = False
        self.task.handoff_to_lane()

    def test_handoff_to_lane_activates_resident_node_then_switches_owner(self):
        self.task.lane_handoff_enabled = True
        self.task.lane_activate_service = "/lane_proto/set_active"
        self.task.lane_owner_service = "/cmd_vel_owner/set_lane_mode"
        self.task.lane_handoff_timeout = 1.0
        self.task.lane_state = "STOPPED"
        self.task.lane_state_event = threading.Event()
        self.task.lock = threading.RLock()
        self.task.set_ros_camera_streaming = lambda _on, required=True: None
        self.task.publish_state = lambda _state: None
        calls = []
        original_wait = task_module.rospy.wait_for_service
        original_proxy = task_module.rospy.ServiceProxy

        class Response(object):
            success = True
            message = "ok"

        task_module.rospy.wait_for_service = (
            lambda service, timeout: calls.append(("wait", service)))
        task_module.rospy.ServiceProxy = (
            lambda service, _kind: lambda enabled: (
                calls.append(("call", service, enabled)) or Response()))
        try:
            self.task.handoff_to_lane()
        finally:
            task_module.rospy.wait_for_service = original_wait
            task_module.rospy.ServiceProxy = original_proxy
        self.assertEqual(calls, [
            ("wait", "/lane_proto/set_active"),
            ("wait", "/cmd_vel_owner/set_lane_mode"),
            ("call", "/lane_proto/set_active", True),
            ("call", "/cmd_vel_owner/set_lane_mode", True),
        ])

    def test_handoff_to_lane_service_failure_aborts(self):
        self.task.lane_handoff_enabled = True
        self.task.lane_activate_service = "/lane_proto/set_active"
        self.task.lane_owner_service = "/cmd_vel_owner/set_lane_mode"
        self.task.set_ros_camera_streaming = lambda _on, required=True: None
        original_wait = task_module.rospy.wait_for_service
        task_module.rospy.wait_for_service = (
            lambda *_args, **_kwargs: (_ for _ in ()).throw(
                task_module.rospy.ROSException("not ready")))
        try:
            with self.assertRaises(task_module.MissionAbort):
                self.task.handoff_to_lane()
        finally:
            task_module.rospy.wait_for_service = original_wait


if __name__ == "__main__":
    unittest.main()
