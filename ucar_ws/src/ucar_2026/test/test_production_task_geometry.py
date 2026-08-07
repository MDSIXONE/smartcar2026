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


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))
SCRIPT_ROOT = os.path.join(PACKAGE_ROOT, "scripts")
if SCRIPT_ROOT not in sys.path:
    sys.path.insert(0, SCRIPT_ROOT)

from production_task_geometry import (  # noqa: E402
    DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG,
    DEFAULT_PRODUCTION_ROUTE,
    TaskDefinitionError,
    bearing,
    build_straight_segments,
    load_middle_target_guard_points,
    load_middle_zone_geometry,
    load_numbered_points,
    load_wall_reference_points,
    needs_recenter,
    normalize_angle,
    position_error,
    positive_turn_increment,
    require_points,
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
            52: (-1.75, 2.25),
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
        self.assertAngleAlmostEqual(
            bearing(staging, self.points[262]), math.pi)
        self.assertAngleAlmostEqual(
            bearing(staging, self.points[232]), math.pi / 2.0)
        self.assertAngleAlmostEqual(
            bearing(staging, self.points[295]), -math.pi / 2.0)

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
            [12, 22, 13, 23, 14, 24, 15, 25, 16, 26, 17, 27,
             18, 28, 19, 29])
        self.assertEqual(
            DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG,
            [-45, 45] * 8)
        # One heading per navigation leg (staging leg + route legs).
        self.assertEqual(
            len(DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG),
            len(DEFAULT_PRODUCTION_ROUTE))

    def test_production_route_collapses_to_exact_straight_segments(self):
        self.assertEqual(
            build_straight_segments(
                DEFAULT_PRODUCTION_ROUTE, self.points,
                math.radians(1.0)),
            [(12, 22), (22, 13), (13, 23), (23, 14), (14, 24),
             (24, 15), (15, 25), (25, 16), (16, 26), (26, 17),
             (17, 27), (27, 18), (18, 28), (28, 19), (19, 29)])

    def test_middle_target_guard_mapping_and_filtered_scan_match(self):
        guards = load_middle_target_guard_points(
            self.grid_path, DEFAULT_PRODUCTION_ROUTE)
        self.assertEqual(
            dict((number, sorted(points))
                 for number, points in guards.items()),
            {
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
                29: [435, 436, 444, 445],
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

    def test_stop_point_for_wall_point_is_25cm_inside_the_field(self):
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
            self.assertEqual(
                stop_point_for_wall_point(wall_point, 0.5, bounds),
                expected_stop)

    def test_stop_point_for_wall_point_rejects_off_boundary_point(self):
        bounds = (-2.5, 2.5, -0.5, 1.5)
        with self.assertRaises(TaskDefinitionError):
            stop_point_for_wall_point((0.0, 0.0), 0.5, bounds)
        with self.assertRaises(TaskDefinitionError):
            stop_point_for_wall_point((0.75, 0.75), 0.5, bounds)
        with self.assertRaises(TaskDefinitionError):
            stop_point_for_wall_point((-2.5, 2.5), 0.5, bounds)


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
        self.task.expected_item_text = u""
        self.task.expected_production_category = None
        self.task._ocr_turn_stop_flag = False
        self.task.processing_dwell_seconds = 0.0
        self.task.middle_zone_square_side = 0.5
        self.task.middle_zone_bounds = (-2.5, 2.5, -0.5, 1.5)
        self.task.ocr_alignment_min_speed = 0.12
        self.task.spark_classify_enabled = False
        self.task.tts_enabled = False

    def tearDown(self):
        task_module.rospy.logwarn = self.original_logwarn
        task_module.rospy.loginfo = self.original_loginfo

    def capture_warning(self, message, *args):
        self.warnings.append(message % args)

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
        self.task.wait_for_item_input = (
            lambda: (events.append("item_input"), u"测试物品")[1])
        self.task.wait_for_safe_start = lambda: events.append("safe_start")
        self.task.switch_to_point_mode = lambda: events.append("point_mode")
        self.task.resume_production_only = False
        self.task.staging_point_number = 52
        self.task.qr_observation_numbers = [262]
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}

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
        self.task.wait_for_item_input = (
            lambda: (events.append("item_input"), u"测试物品")[1])
        self.task.wait_for_safe_start = lambda: events.append("safe_start")
        self.task.switch_to_point_mode = lambda: events.append("point_mode")
        self.task.resume_production_only = True
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
        self.task.wait_for_item_input = (
            lambda: (events.append("item_input"), u"测试物品")[1])
        self.task.wait_for_safe_start = lambda: events.append("safe_start")
        self.task.switch_to_point_mode = lambda: events.append("point_mode")
        self.task.switch_to_body_projection = lambda: self.fail(
            "QR completion must not select body_projection")
        self.task.resume_production_only = False
        self.task.staging_point_number = 52
        self.task.qr_observation_numbers = [262]
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None
        self.task.wait_for_qr_scanner = lambda: None
        self.task.start_ros_camera_and_wait = lambda _context: None
        self.task.qr_enable_pub = QrPublisher()
        self.task.stop_qr_classifier = lambda: None
        self.task.post_qr_waypoint_number = 0

        def scan_observation_point(observation_number):
            events.append("qr_scan")
            self.task.qr_classifications.append({
                "observation": observation_number,
                "qr_text": u"测试",
                "category": u"日用品",
                "source": "stub",
                "attempts": 0,
                "model": "test",
                "error": "",
            })

        self.task.scan_observation_point = scan_observation_point
        self.task.stop_ros_camera_streaming = lambda required=True: None
        self.task.prepare_result_directory = lambda: None
        self.task.use_ros_camera_for_ocr = True
        self.task.camera_image_topic = "/usb_cam/image_raw"
        self.task.start_native_ocr = lambda: events.append("ocr_start")
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
             "qr_scan", "ocr_start", "production_navigation"])

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
        self.task.qr_sequence = 0
        self.task.latest_qr_text = ""
        self.task.used_qr_codes = set()
        self.task.require_distinct_qr_codes = True
        self.task.qr_event = threading.Event()
        self.task.require_safe = lambda: None

        class QrMessage(object):
            data = "next-code"

        def publish_qr():
            time.sleep(0.01)
            self.task.qr_result_cb(QrMessage())

        worker = threading.Thread(target=publish_qr)
        worker.start()
        started = time.time()
        detected = self.task.wait_for_fresh_qr(0, 1.0)
        elapsed = time.time() - started
        worker.join()

        self.assertEqual(detected, "next-code")
        self.assertLess(elapsed, 0.20)

    def test_qr_seen_while_facing_is_accepted_without_search_wait(self):
        self.task.lock = threading.RLock()
        self.task.points = {52: (-1.75, 2.25), 262: (-2.50, 2.25)}
        self.task.staging_point_number = 52
        self.task.qr_sequence = 0
        self.task.latest_qr_text = ""
        self.task.used_qr_codes = set()
        self.task.qr_observation_numbers = [262]
        self.task.require_distinct_qr_codes = True
        self.task.qr_event = threading.Event()
        self.task.publish_state = lambda _state: None

        class QrMessage(object):
            data = "turning-code"

        def navigate(*_args, **_kwargs):
            self.task.qr_result_cb(QrMessage())

        self.task.navigate_coordinates = navigate
        self.task.wait_for_fresh_qr = (
            lambda *_args: self.fail("must not wait after a turn-time QR"))

        self.task.scan_observation_point(262)

        self.assertEqual(self.task.used_qr_codes, set(["turning-code"]))

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

    def test_arrival_scan_restores_capture_yaw_before_observing(self):
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
        def turn_one_circle(_label, candidate_handler=None):
            self.assertIsNotNone(candidate_handler)
            self.assertTrue(candidate_handler(response, 1.2))
            return None, 2.0 * math.pi

        self.task.rotate_full_revolution_for_ocr = turn_one_circle
        self.task.restore_ocr_capture_yaw = (
            lambda _response, _label: calls.append("restore"))
        self.task.stop_motion = lambda: None
        self.task.wait_for_chassis_stop = lambda _context: None

        def observe(point_number, _label):
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

        self.assertEqual(calls[0], "restore")
        self.assertEqual(calls[1], ("observe", 12))
        self.assertEqual(calls[2], "save")
        self.assertEqual(calls[3], "save")
        self.assertEqual(
            self.task.observations[0]["turn_detection_pose_map"],
            [1.0, 2.0, 0.3])
        self.assertEqual(self.task.target_scan_events[0]["wall_point_number"], 297)
        self.assertEqual(
            self.task.target_scan_events[0]["outcome"],
            "processing_category_recorded")

    def test_restore_capture_yaw_precedes_alignment_recapture(self):
        calls = []
        self.task.current_map_pose = lambda _context: (1.0, 2.0, 0.9)
        self.task.navigate_coordinates = (
            lambda *args, **kwargs: calls.append((args, kwargs)))
        self.task.wait_for_chassis_stop = (
            lambda context: calls.append(("stop", context)))
        self.task.restore_ocr_capture_yaw(
            {"capture_requested_pose_map": [1.0, 2.0, 0.3]}, "test")

        self.assertEqual(calls[0][0][0:3], (1.0, 2.0, 0.3))
        self.assertFalse(calls[0][1]["require_plan"])
        self.assertTrue(calls[0][1]["require_action_success"])
        self.assertEqual(calls[1], ("stop", "test restore capture yaw"))

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


if __name__ == "__main__":
    unittest.main()
