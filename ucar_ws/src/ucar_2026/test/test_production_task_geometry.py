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
    load_numbered_points,
    load_wall_reference_points,
    needs_recenter,
    normalize_angle,
    position_error,
    positive_turn_increment,
    require_points,
)

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
            [3, 2, 1, 11, 21, 31, 32, 33, 34, 35, 4, 5, 6, 7, 8,
             9, 10, 20, 30, 40, 39, 38, 37])
        self.assertEqual(
            DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG,
            [180, 180, -90, -90, -90, 0, 0, 0, 0, 108.434949,
             0, 0, 0, 0, 0, 0, -90, -90, -90, 180, 180, 180,
             180])
        for index in range(len(DEFAULT_PRODUCTION_ROUTE) - 1):
            source = self.points[DEFAULT_PRODUCTION_ROUTE[index]]
            target = self.points[DEFAULT_PRODUCTION_ROUTE[index + 1]]
            self.assertAngleAlmostEqual(
                math.radians(
                    DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG[index]),
                bearing(source, target))
        self.assertEqual(
            DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG[-1],
            DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG[-2])

    def test_production_route_collapses_to_exact_straight_segments(self):
        self.assertEqual(
            build_straight_segments(
                DEFAULT_PRODUCTION_ROUTE, self.points,
                math.radians(1.0)),
            [(3, 1), (1, 31), (31, 35), (35, 4), (4, 10),
             (10, 40), (40, 37)])

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

    def test_alignment_uses_move_base_then_stop_gate(self):
        calls = []
        self.task.ocr_alignment_max_speed = 0.22
        self.task.ocr_alignment_kp = 0.0025
        self.task.ocr_alignment_step_seconds = 0.30
        self.task.current_map_pose = lambda _context: (1.0, 2.0, 0.5)
        def capture_navigation(
                x, y, yaw, label, require_plan,
                require_action_success=False):
            calls.append((
                "navigate", x, y, yaw, label, require_plan,
                require_action_success))

        self.task.navigate_coordinates = capture_navigation
        self.task.wait_for_chassis_stop = (
            lambda context: calls.append(("stop_gate", context)))
        self.task.rotate_for_pixel_error(40.0, "test")
        self.assertEqual(calls[0][0], "navigate")
        self.assertEqual(calls[0][1:3], (1.0, 2.0))
        self.assertFalse(calls[0][5])
        self.assertTrue(calls[0][6])
        self.assertEqual(calls[1], ("stop_gate", "test protected alignment"))


if __name__ == "__main__":
    unittest.main()
