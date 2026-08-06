#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Fail-safe ROS state machine for the requested 2026 production mission.

Flow:
  1. Navigate to centre 52.
  2. From 52, face QR observation points 262, 232, and 295 in order.
     If a fresh QR is not decoded while facing a point, turn slowly for at
     most one complete revolution while scanning.
  3. Disable QR decoding and start the Python 3 OCR helper.  In the default
     configuration the helper consumes frames saved from the ROS usb_cam topic.
  4. Lock CymPlanner in front-lookahead point mode for the whole mission.
  5. Navigate each configured production target.  After arrival, turn at most
     one full revolution while querying OCR.  A candidate stops the turn,
     passes a fresh-odometry stop gate, then aligns the box and reads the front
     lidar distance.  A full turn without a candidate proceeds immediately to
     the next target.
  6. Save every attempt plus the three strongest distinct wall observations.

The node never drives to the QR edge points: they lie on the field boundary.
They are gaze targets while the chassis remains at centre 52.
"""

from __future__ import print_function

import json
import math
import os
import select
import subprocess
import threading
import time

import actionlib
import cv2
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int8, String
from std_srvs.srv import Empty

from production_task_geometry import (
    DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG,
    DEFAULT_PRODUCTION_ROUTE,
    TaskDefinitionError,
    bearing,
    is_finite,
    load_numbered_points,
    load_middle_target_guard_points,
    load_wall_reference_points,
    needs_recenter,
    normalize_angle,
    position_error,
    positive_turn_increment,
    require_points,
)
from production_task_perception import (
    alignment_angular_speed,
    front_scan_distance,
    forward_ray_wall_intersection,
    horizontal_pixel_error,
    is_navigation_ocr_candidate,
    nearest_numbered_point,
    normalize_production_category,
    odom_velocity_is_stopped,
    select_three_processing_observations,
    target_guard_scan_matches,
)


class MissionAbort(RuntimeError):
    pass


class ProductionTask2026(object):
    def __init__(self):
        self.grid_path = rospy.get_param("~grid_path")
        self.staging_point_number = int(
            rospy.get_param("~staging_point_number", 52))
        self.qr_observation_numbers = [
            int(value) for value in
            rospy.get_param("~qr_observation_numbers", [262, 232, 295])
        ]
        self.production_route_numbers = [
            int(value) for value in
            rospy.get_param(
                "~production_route_numbers",
                DEFAULT_PRODUCTION_ROUTE)
        ]
        self.production_observation_headings = [
            math.radians(float(value)) for value in
            rospy.get_param(
                "~production_observation_headings_deg",
                DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG)
        ]

        self.start_delay = float(rospy.get_param("~start_delay", 2.0))
        self.resume_production_only = bool(
            rospy.get_param("~resume_production_only", False))
        self.move_base_ready_timeout = float(
            rospy.get_param("~move_base_ready_timeout", 90.0))
        self.safe_start_timeout = float(
            rospy.get_param("~safe_start_timeout", 45.0))
        self.plan_timeout = float(rospy.get_param("~plan_timeout", 15.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.goal_cancel_timeout = float(
            rospy.get_param("~goal_cancel_timeout", 3.0))
        self.arrival_tolerance = float(
            rospy.get_param("~arrival_tolerance", 0.10))
        self.post_turn_recenter_trigger = float(
            rospy.get_param("~post_turn_recenter_trigger", 0.06))
        self.post_turn_recenter_attempts = max(
            1, int(rospy.get_param("~post_turn_recenter_attempts", 2)))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.35))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.50))
        self.minimum_finite_odom_samples = max(
            1, int(rospy.get_param("~minimum_finite_odom_samples", 10)))

        # This is an upper bound for an unsuccessful observation, not a
        # dwell time.  A QR event wakes the scanner immediately.
        self.qr_search_timeout = float(rospy.get_param(
            "~qr_search_timeout",
            rospy.get_param("~qr_hold_seconds", 4.0)))
        self.qr_rotation_speed = abs(float(
            rospy.get_param("~qr_rotation_speed", 0.18)))
        self.rotation_control_rate = max(
            5.0, float(rospy.get_param("~rotation_control_rate", 20.0)))
        self.rotation_timeout_scale = max(
            1.1, float(rospy.get_param("~rotation_timeout_scale", 1.8)))
        self.rotation_completion_tolerance = max(
            0.01, float(rospy.get_param(
                "~rotation_completion_tolerance", 0.05)))
        self.require_distinct_qr_codes = bool(
            rospy.get_param("~require_distinct_qr_codes", True))
        self.navigation_mode_connect_timeout = float(rospy.get_param(
            "~navigation_mode_connect_timeout",
            rospy.get_param("~body_mode_connect_timeout", 5.0)))
        self.camera_node_name = str(
            rospy.get_param("~camera_node_name", "/usb_cam"))
        self.camera_start_service = str(rospy.get_param(
            "~camera_start_service",
            self.camera_node_name.rstrip("/") + "/start_capture"))
        self.camera_stop_service = str(rospy.get_param(
            "~camera_stop_service",
            self.camera_node_name.rstrip("/") + "/stop_capture"))
        self.camera_service_timeout = float(
            rospy.get_param("~camera_service_timeout", 5.0))
        self.camera_starts_suspended = bool(
            rospy.get_param("~camera_starts_suspended", False))
        self.use_ros_camera_for_ocr = bool(
            rospy.get_param("~use_ros_camera_for_ocr", True))
        self.camera_image_topic = str(
            rospy.get_param(
                "~camera_image_topic", "/usb_cam/image_raw"))
        self.camera_frame_timeout = float(
            rospy.get_param("~camera_frame_timeout", 1.0))
        self.video_device = str(
            rospy.get_param("~video_device", "/dev/ucar_video"))
        self.camera_width = int(rospy.get_param("~camera_width", 640))
        self.camera_height = int(rospy.get_param("~camera_height", 480))
        self.camera_warmup_frames = max(
            1, int(rospy.get_param("~camera_warmup_frames", 8)))
        self.camera_open_timeout = float(
            rospy.get_param("~camera_open_timeout", 8.0))
        self.ocr_python = str(
            rospy.get_param("~ocr_python", "/usr/bin/python3"))
        self.ocr_helper_path = str(rospy.get_param(
            "~ocr_helper_path",
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "production_camera_ocr.py")))
        self.live_ppocr_path = str(rospy.get_param(
            "~live_ppocr_path",
            "/home/ucar/ocr3/ppocr_trt/python/live_ppocr.py"))
        self.ppocr_root = str(rospy.get_param(
            "~ppocr_root", "/home/ucar/ocr3/ppocr_trt"))
        self.ocr_side = int(rospy.get_param("~ocr_side", 640))
        self.camera_mirror = bool(
            rospy.get_param("~camera_mirror", True))
        self.ocr_helper_ready_timeout = float(
            rospy.get_param("~ocr_helper_ready_timeout", 45.0))
        self.ocr_capture_timeout = float(
            rospy.get_param("~ocr_capture_timeout", 12.0))
        self.ocr_scan_rotation_speed = abs(float(rospy.get_param(
            "~ocr_scan_rotation_speed", 0.18)))
        self.ocr_scan_poll_period = float(rospy.get_param(
            "~ocr_scan_poll_period",
            rospy.get_param("~navigation_ocr_poll_period", 0.20)))
        self.ocr_scan_candidate_confidence = float(rospy.get_param(
            "~ocr_scan_candidate_confidence",
            rospy.get_param("~navigation_ocr_candidate_confidence", 60.0)))
        self.ocr_min_confidence = float(
            rospy.get_param("~ocr_min_confidence", 0.30))
        self.ocr_alignment_tolerance_px = float(
            rospy.get_param("~ocr_alignment_tolerance_px", 18.0))
        self.ocr_alignment_kp = float(
            rospy.get_param("~ocr_alignment_kp", 0.0025))
        self.ocr_alignment_kd = float(
            rospy.get_param("~ocr_alignment_kd", 0.00035))
        self.ocr_alignment_max_speed = abs(float(
            rospy.get_param("~ocr_alignment_max_speed", 0.22)))
        self.ocr_alignment_attempts = max(
            1, int(rospy.get_param("~ocr_alignment_attempts", 6)))
        self.ocr_alignment_step_seconds = float(
            rospy.get_param("~ocr_alignment_step_seconds", 0.30))
        self.ocr_alignment_yaw_tolerance = float(
            rospy.get_param("~ocr_alignment_yaw_tolerance_rad", 0.01))
        self.ocr_alignment_turn_timeout = float(
            rospy.get_param("~ocr_alignment_turn_timeout", 2.5))
        self.ocr_alignment_min_speed = abs(float(
            rospy.get_param("~ocr_alignment_min_speed", 0.12)))
        self.front_scan_half_angle = math.radians(float(
            rospy.get_param("~front_scan_half_angle_deg", 3.0)))
        self.front_scan_timeout = float(
            rospy.get_param("~front_scan_timeout", 1.0))
        self.lidar_forward_offset = float(
            rospy.get_param("~lidar_forward_offset_m", 0.0))
        self.wall_match_max_error = float(
            rospy.get_param("~wall_match_max_error_m", 0.30))
        self.ray_range_agreement = float(
            rospy.get_param("~ray_range_agreement_m", 0.30))
        # This is the same dynamically filtered source consumed by the
        # global obstacle layer.  It deliberately excludes static-map walls,
        # so a fixed field vertex cannot falsely skip a production target.
        self.target_guard_scan_topic = str(rospy.get_param(
            "~target_guard_scan_topic", "/scan_global_obstacles"))
        self.target_guard_match_radius = float(rospy.get_param(
            "~target_guard_match_radius_m", 0.12))
        self.target_guard_confirmation_scans = int(rospy.get_param(
            "~target_guard_confirmation_scans", 2))
        self.target_guard_precheck_timeout = float(rospy.get_param(
            "~target_guard_precheck_timeout", 0.50))
        self.target_guard_scan_max_age = float(rospy.get_param(
            "~target_guard_scan_max_age", 0.50))
        self.straight_segment_angle_tolerance = math.radians(float(
            rospy.get_param(
                "~straight_segment_angle_tolerance_deg", 1.0)))
        self.stop_confirmation_timeout = float(
            rospy.get_param("~stop_confirmation_timeout", 2.0))
        self.stopped_odom_speed_epsilon = float(
            rospy.get_param("~stopped_odom_speed_epsilon", 0.02))
        self.stopped_odom_samples = max(
            1, int(rospy.get_param("~stopped_odom_samples", 3)))
        self.result_directory = os.path.expanduser(str(
            rospy.get_param(
                "~result_directory", "~/.ros/ucar_2026_observations")))
        # Spark Xunfei QR-text classification.  A helper subprocess owns the
        # network call so the mission thread is never blocked; classification
        # failure never aborts the task (local keyword map is the fallback).
        self.spark_classify_enabled = bool(
            rospy.get_param("~spark_classify_enabled", False))
        self.spark_python = str(
            rospy.get_param("~spark_python", "/usr/bin/python3"))
        self.spark_helper_path = str(rospy.get_param(
            "~spark_helper_path",
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "production_qr_classifier.py")))
        self.spark_model = str(
            rospy.get_param("~spark_model", "spark-x"))
        self.spark_api_base_url = str(rospy.get_param(
            "~spark_api_base_url",
            "https://spark-api-open.xf-yun.com/x2/chat/completions"))
        self.spark_thinking = str(rospy.get_param(
            "~spark_thinking", "disabled"))
        # Path to a file whose first line is the Xunfei APIPassword.  The file
        # itself must live on the vehicle and is never committed to the repo.
        self.spark_password_file = str(
            rospy.get_param("~spark_password_file", ""))
        self.spark_retries = max(
            0, int(rospy.get_param("~spark_retries", 2)))
        self.spark_timeout = max(
            0.5, float(rospy.get_param("~spark_timeout", 8.0)))
        self.spark_helper_ready_timeout = float(
            rospy.get_param("~spark_helper_ready_timeout", 30.0))

        if self.qr_rotation_speed <= 0.0:
            raise TaskDefinitionError("qr_rotation_speed must be positive")
        if (
                len(self.production_route_numbers) !=
                len(self.production_observation_headings)):
            raise TaskDefinitionError(
                "production route and observation headings have different "
                "lengths")
        if self.ocr_alignment_max_speed <= 0.0:
            raise TaskDefinitionError(
                "ocr_alignment_max_speed must be positive")
        if (not is_finite(self.ocr_alignment_kp) or
                self.ocr_alignment_kp <= 0.0):
            raise TaskDefinitionError("ocr_alignment_kp must be positive")
        if (not is_finite(self.ocr_alignment_kd) or
                self.ocr_alignment_kd < 0.0):
            raise TaskDefinitionError("ocr_alignment_kd must be non-negative")
        if self.ocr_alignment_step_seconds <= 0.0:
            raise TaskDefinitionError(
                "ocr_alignment_step_seconds must be positive")
        if self.ocr_alignment_yaw_tolerance <= 0.0:
            raise TaskDefinitionError(
                "ocr_alignment_yaw_tolerance must be positive")
        if self.ocr_alignment_turn_timeout <= 0.0:
            raise TaskDefinitionError(
                "ocr_alignment_turn_timeout must be positive")
        ocr_scan_values = (
            self.ocr_scan_rotation_speed,
            self.ocr_scan_poll_period,
            self.ocr_scan_candidate_confidence,
            self.goal_cancel_timeout,
            self.stop_confirmation_timeout,
            self.stopped_odom_speed_epsilon,
        )
        if (
                not all(is_finite(value)
                        for value in ocr_scan_values) or
                self.ocr_scan_rotation_speed <= 0.0 or
                self.ocr_scan_poll_period <= 0.0 or
                self.ocr_scan_candidate_confidence < 0.0 or
                self.goal_cancel_timeout <= 0.0 or
                self.stop_confirmation_timeout <= 0.0 or
                self.stopped_odom_speed_epsilon < 0.0):
            raise TaskDefinitionError(
                "OCR turn-scan timing and stop-gate parameters are invalid")
        if not (
                0.0 < self.post_turn_recenter_trigger <
                self.arrival_tolerance):
            raise TaskDefinitionError(
                "post_turn_recenter_trigger must be positive and smaller "
                "than arrival_tolerance")
        target_guard_values = (
            self.target_guard_match_radius,
            self.target_guard_precheck_timeout,
            self.target_guard_scan_max_age,
        )
        if (
                not all(is_finite(value) for value in target_guard_values) or
                not self.target_guard_scan_topic or
                self.target_guard_match_radius <= 0.0 or
                self.target_guard_confirmation_scans <= 0 or
                self.target_guard_precheck_timeout <= 0.0 or
                self.target_guard_scan_max_age <= 0.0):
            raise TaskDefinitionError(
                "target guard scan parameters must be finite and positive")

        all_required_numbers = (
            [self.staging_point_number] +
            self.qr_observation_numbers +
            self.production_route_numbers)
        self.points = load_numbered_points(self.grid_path)
        require_points(self.points, all_required_numbers)
        self.production_navigation_legs = [
            (self.staging_point_number, self.production_route_numbers[0])
        ] + list(zip(
            self.production_route_numbers[:-1],
            self.production_route_numbers[1:]))
        self.target_guard_points = load_middle_target_guard_points(
            self.grid_path, self.production_route_numbers)
        self.wall_reference_points = load_wall_reference_points(
            self.grid_path)

        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.make_plan = rospy.ServiceProxy("move_base/make_plan", GetPlan)
        self.tf_listener = tf.TransformListener()
        self.cv_bridge = CvBridge()

        self.cmd_vel_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10)
        self.qr_enable_pub = rospy.Publisher(
            "/qrcode_start_flag", Int8, queue_size=1, latch=True)
        self.navigation_mode_pub = rospy.Publisher(
            "/ucar/navigation_mode", String, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher(
            "/ucar_2026/task_state", String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher(
            "/ucar_2026/task_result", String, queue_size=1, latch=True)

        self.lock = threading.RLock()
        self.latest_odom_receipt = None
        self.latest_odom_finite = False
        self.latest_odom_velocity = None
        self.consecutive_finite_odom = 0
        self.latest_scan = None
        self.latest_scan_receipt = None
        self.latest_target_guard_scan = None
        self.latest_target_guard_scan_receipt = None
        self.target_guard_scan_sequence = 0
        self.latest_camera_image = None
        self.latest_camera_receipt = None
        self.camera_sequence = 0
        self.camera_streaming = not self.camera_starts_suspended
        self.critical_error = ""
        self.qr_sequence = 0
        self.latest_qr_text = ""
        self.used_qr_codes = set()
        self.qr_event = threading.Event()
        self.mission_started = False
        self.mission_finished = False
        self.ocr_process = None
        self.ocr_log_handle = None
        self.spark_process = None
        self.spark_log_handle = None
        self.qr_classifications = []
        self.observations = []
        self.target_scan_events = []
        self.target_guard_events = []
        self.run_directory = None
        self.capture_sequence = 0

        rospy.Subscriber(
            "/odom_raw", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber(
            "/scan", LaserScan, self.scan_cb, queue_size=5)
        rospy.Subscriber(
            self.target_guard_scan_topic, LaserScan,
            self.target_guard_scan_cb, queue_size=5)
        rospy.Subscriber(
            self.camera_image_topic, Image, self.camera_image_cb,
            queue_size=1)
        rospy.Subscriber(
            "/qr_result", String, self.qr_result_cb, queue_size=20)
        rospy.Subscriber(
            "/rosout_agg", Log, self.rosout_cb, queue_size=100)
        rospy.on_shutdown(self.shutdown)

        self.publish_state("WAITING_START")
        rospy.Timer(
            rospy.Duration(self.start_delay), self.start_cb, oneshot=True)

    def publish_state(self, state):
        self.state_pub.publish(String(data=state))
        rospy.loginfo("PRODUCTION_TASK_STATE %s", state)

    def publish_result(self, success, reason):
        payload = {
            "success": bool(success),
            "reason": str(reason),
            "qr_codes": sorted(self.used_qr_codes),
            "qr_classifications": self.qr_classifications,
            "recognized_categories": select_three_processing_observations(
                self.observations),
            "result_file": (
                os.path.join(self.run_directory, "observations.json")
                if self.run_directory else ""),
        }
        encoded = json.dumps(payload, sort_keys=True)
        self.result_pub.publish(String(data=encoded))
        rospy.loginfo("PRODUCTION_TASK_RESULT %s", encoded)

    def odom_cb(self, message):
        values = [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        ]
        finite = all(is_finite(value) for value in values)
        velocity = (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        )
        with self.lock:
            self.latest_odom_receipt = rospy.Time.now()
            self.latest_odom_finite = finite
            self.latest_odom_velocity = velocity if finite else None
            if finite:
                self.consecutive_finite_odom += 1
                if self.consecutive_finite_odom >= 10:
                    self.clear_sensor_alive_error()
            else:
                self.consecutive_finite_odom = 0
                self.critical_error = "non-finite /odom_raw"

    def clear_sensor_alive_error(self):
        """A recovered odom stream clears sensor-alive errors only.

        The safety gate checks current odom health (>=10 finite frames and
        fresh TFs), so a transient startup gap reported by base_driver must
        not lock the mission forever.  Structural errors (crc16, head_len,
        tf_nan_input, non-finite) stay sticky and require a full restart.
        """
        if not self.critical_error:
            return
        lowered = self.critical_error.lower()
        if ("odom sensor not active" in lowered or
                "imu sensor not active" in lowered):
            self.critical_error = ""

    def scan_cb(self, message):
        with self.lock:
            self.latest_scan = message
            self.latest_scan_receipt = rospy.Time.now()

    def target_guard_scan_cb(self, message):
        """Keep dynamic-only guard evidence; cancellation stays in main loop."""
        with self.lock:
            self.latest_target_guard_scan = message
            self.latest_target_guard_scan_receipt = rospy.Time.now()
            self.target_guard_scan_sequence += 1

    def camera_image_cb(self, message):
        with self.lock:
            self.latest_camera_image = message
            self.latest_camera_receipt = rospy.Time.now()
            self.camera_sequence += 1

    def qr_result_cb(self, message):
        text = message.data.strip()
        if not text:
            return
        with self.lock:
            self.qr_sequence += 1
            self.latest_qr_text = text
        self.qr_event.set()
        rospy.loginfo("PRODUCTION_QR_EVENT sequence=%d value=%s",
                      self.qr_sequence, self.log_safe_text(text))

    def rosout_cb(self, message):
        text = message.msg.lower()
        critical_markers = (
            "crc16",
            "head_len",
            "tf_nan_input",
            "odom sensor not active",
            "imu sensor not active",
        )
        if not any(marker in text for marker in critical_markers):
            return
        with self.lock:
            if not self.critical_error:
                self.critical_error = message.msg

    def start_cb(self, _event):
        with self.lock:
            if self.mission_started:
                return
            self.mission_started = True
        worker = threading.Thread(target=self.run_mission_guarded)
        worker.daemon = True
        worker.start()

    def run_mission_guarded(self):
        try:
            self.run_mission()
        except MissionAbort as exc:
            reason = str(exc)
            rospy.logerr("PRODUCTION_TASK_ABORTED %s", reason)
            self.stop_everything()
            try:
                self.save_observation_summary()
            except Exception as save_error:
                rospy.logerr(
                    "PRODUCTION_RESULT_SAVE_FAILED %s", save_error)
            self.publish_state("ABORTED")
            self.publish_result(False, reason)
        except Exception as exc:
            reason = "unexpected task error: %s" % exc
            rospy.logerr("PRODUCTION_TASK_ABORTED %s", reason)
            self.stop_everything()
            try:
                self.save_observation_summary()
            except Exception as save_error:
                rospy.logerr(
                    "PRODUCTION_RESULT_SAVE_FAILED %s", save_error)
            self.publish_state("ABORTED")
            self.publish_result(False, reason)
        finally:
            try:
                self.ensure_ros_camera_released()
            except Exception as cleanup_error:
                rospy.logerr(
                    "PRODUCTION_CAMERA_FINAL_CLEANUP_FAILED %s",
                    cleanup_error)
            with self.lock:
                self.mission_finished = True

    def run_mission(self):
        self.publish_state("WAITING_SAFE_START")
        if not self.move_base.wait_for_server(
                rospy.Duration(self.move_base_ready_timeout)):
            raise MissionAbort(
                "move_base unavailable after %.1f s" %
                self.move_base_ready_timeout)
        self.wait_for_safe_start()
        self.switch_to_point_mode()

        if self.resume_production_only:
            self.qr_enable_pub.publish(Int8(data=0))
            rospy.logwarn(
                "PRODUCTION_TASK_RESUME production-only route=%s",
                self.production_route_numbers)
        else:
            staging = self.points[self.staging_point_number]
            first_observation = self.points[self.qr_observation_numbers[0]]
            staging_yaw = bearing(staging, first_observation)
            self.navigate_to(
                self.staging_point_number, staging_yaw, "STAGING_52")

            self.publish_state("QR_SEQUENCE")
            self.move_base.cancel_all_goals()
            self.stop_motion()
            self.wait_for_chassis_stop("camera start before QR sequence")
            self.wait_for_qr_scanner()
            self.start_ros_camera_and_wait("QR sequence")
            self.qr_enable_pub.publish(Int8(data=1))
            for observation_number in self.qr_observation_numbers:
                self.scan_observation_point(observation_number)
            self.qr_enable_pub.publish(Int8(data=0))
            self.stop_qr_classifier()
            self.stop_ros_camera_streaming(required=True)

        self.prepare_result_directory()
        if self.use_ros_camera_for_ocr:
            self.publish_state("OPEN_ROS_IMAGE_OCR")
            rospy.loginfo(
                "PRODUCTION_CAMERA_MODE ros_image topic=%s",
                self.camera_image_topic)
        else:
            self.release_ros_camera()
        self.start_native_ocr()

        rospy.loginfo(
            "PRODUCTION_TARGET_LEGS %s arrival_ocr_turn=360deg",
            self.production_navigation_legs)
        for segment_index, (start_number, end_number) in enumerate(
                self.production_navigation_legs, 1):
            self.navigate_target_and_scan(
                segment_index, start_number, end_number,
                target_yaw=self.production_observation_headings[
                    segment_index - 1])

        self.stop_native_ocr()
        self.ensure_ros_camera_released()
        selected = select_three_processing_observations(self.observations)
        self.save_observation_summary()
        if len(selected) < 3:
            raise MissionAbort(
                "only %d/3 distinct processing categories were recognized" %
                len(selected))
        self.stop_motion()
        self.publish_state("SUCCEEDED")
        self.publish_result(
            True,
            "saved three processing categories; route completed through point %d" %
            self.production_route_numbers[-1])

    def scan_observation_point(self, observation_number):
        staging = self.points[self.staging_point_number]
        observation = self.points[observation_number]
        observation_yaw = bearing(staging, observation)
        self.publish_state("QR_FACE_%d" % observation_number)
        # Capture the sequence before turning so a code acquired while the
        # chassis is settling at this new face is accepted immediately.
        with self.lock:
            baseline = self.qr_sequence
        self.navigate_coordinates(
            staging[0], staging[1], observation_yaw,
            "QR face point %d" % observation_number,
            require_plan=False)

        detected = self.fresh_qr_after(baseline)
        if detected is None:
            detected = self.wait_for_fresh_qr(
                baseline, self.qr_search_timeout)
        if detected is None:
            self.publish_state("QR_SEARCH_TURN_%d" % observation_number)
            detected = self.rotate_full_revolution(
                "QR observation point %d" % observation_number,
                self.qr_rotation_speed,
                stop_for_qr=True,
                qr_baseline=baseline)
        if detected is None:
            raise MissionAbort(
                "no fresh QR detected for observation point %d after one "
                "complete search turn" % observation_number)
        self.used_qr_codes.add(detected)
        rospy.loginfo(
            "PRODUCTION_QR_ACCEPTED observation=%d value=%s count=%d/%d",
            observation_number, self.log_safe_text(detected),
            len(self.used_qr_codes), len(self.qr_observation_numbers))
        self.classify_qr_text(observation_number, detected)

    def wait_for_qr_scanner(self):
        deadline = (
            rospy.Time.now() + rospy.Duration(
                self.navigation_mode_connect_timeout))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            if self.qr_enable_pub.get_num_connections() > 0:
                return
            rospy.sleep(0.1)
        raise MissionAbort("QR scanner did not connect to /qrcode_start_flag")

    def wait_for_fresh_qr(self, baseline, timeout):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            detected = self.fresh_qr_after(baseline)
            if detected is not None:
                return detected
            # Wait on the subscription callback instead of imposing a fixed
            # observation delay.  The bounded wait still lets safety state
            # and ROS shutdown be checked when no QR is visible.
            self.qr_event.wait(0.05)
            self.qr_event.clear()
        return None

    def fresh_qr_after(self, baseline):
        with self.lock:
            sequence = self.qr_sequence
            text = self.latest_qr_text
        if sequence <= baseline or not text:
            return None
        if self.require_distinct_qr_codes and text in self.used_qr_codes:
            return None
        return text

    def prepare_result_directory(self):
        if self.run_directory is not None:
            return
        run_name = time.strftime("run_%Y%m%d_%H%M%S")
        self.run_directory = os.path.join(self.result_directory, run_name)
        try:
            os.makedirs(self.run_directory)
        except OSError as exc:
            if not os.path.isdir(self.run_directory):
                raise MissionAbort(
                    "cannot create result directory %s: %s" %
                    (self.run_directory, exc))
        rospy.loginfo(
            "PRODUCTION_RESULT_DIRECTORY %s", self.run_directory)

    def run_subprocess(self, command, timeout, context):
        try:
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as exc:
            raise MissionAbort("%s cannot start: %s" % (context, exc))
        deadline = time.time() + float(timeout)
        try:
            while process.poll() is None and time.time() < deadline:
                self.require_safe()
                rospy.sleep(0.05)
        except Exception:
            if process.poll() is None:
                process.terminate()
                rospy.sleep(0.2)
            if process.poll() is None:
                process.kill()
            process.communicate()
            raise
        if process.poll() is None:
            process.terminate()
            rospy.sleep(0.2)
            if process.poll() is None:
                process.kill()
            process.communicate()
            raise MissionAbort("%s timed out after %.1f s" % (
                context, timeout))
        stdout, stderr = process.communicate()
        return process.returncode, stdout, stderr

    def call_camera_service(self, service_name, context, required=True):
        try:
            rospy.wait_for_service(
                service_name, timeout=self.camera_service_timeout)
            rospy.ServiceProxy(service_name, Empty)()
            return True
        except Exception as exc:
            message = "%s via %s failed: %s" % (
                context, service_name, exc)
            if required:
                raise MissionAbort(message)
            rospy.logwarn("PRODUCTION_CAMERA_SERVICE_WARNING %s", message)
            return False

    def set_ros_camera_streaming(self, enabled, required=True):
        enabled = bool(enabled)
        if self.camera_streaming == enabled:
            return True
        service_name = (
            self.camera_start_service if enabled
            else self.camera_stop_service)
        context = (
            "starting ROS camera stream" if enabled
            else "stopping ROS camera stream")
        if not self.call_camera_service(
                service_name, context, required=required):
            return False
        with self.lock:
            self.camera_streaming = enabled
            if not enabled:
                self.latest_camera_image = None
                self.latest_camera_receipt = None
        rospy.loginfo(
            "PRODUCTION_CAMERA_STREAM %s service=%s",
            "started" if enabled else "stopped", service_name)
        return True

    def stop_ros_camera_streaming(self, required=True):
        """Stop capture with one bounded retry while keeping the node alive."""
        if not self.camera_streaming:
            return True
        for attempt in range(1, 3):
            if self.set_ros_camera_streaming(False, required=False):
                return True
            rospy.logwarn(
                "PRODUCTION_CAMERA_STOP_RETRY attempt=%d/2", attempt)
            if attempt < 2:
                rospy.sleep(0.10)
        if required:
            raise MissionAbort(
                "ROS camera stream did not stop after two attempts")
        return False

    def ensure_ros_camera_released(self):
        """Best-effort final shutdown, with an exact node kill as fallback."""
        if self.stop_ros_camera_streaming(required=False):
            return True
        try:
            return_code, _stdout, stderr = self.run_subprocess(
                ["rosnode", "kill", self.camera_node_name],
                self.camera_open_timeout,
                "final ROS camera shutdown")
        except Exception as exc:
            rospy.logerr(
                "PRODUCTION_CAMERA_FINAL_STOP_FAILED %s", exc)
            return False
        if return_code != 0:
            rospy.logerr(
                "PRODUCTION_CAMERA_FINAL_STOP_FAILED node=%s error=%s",
                self.camera_node_name,
                stderr.decode("utf-8", "replace").strip()
                if not isinstance(stderr, str) else stderr.strip())
            return False
        with self.lock:
            self.camera_streaming = False
            self.latest_camera_image = None
            self.latest_camera_receipt = None
        rospy.logwarn(
            "PRODUCTION_CAMERA_STOP_FALLBACK killed node=%s",
            self.camera_node_name)
        return True

    def start_ros_camera_and_wait(self, context):
        with self.lock:
            baseline_sequence = self.camera_sequence
        self.set_ros_camera_streaming(True, required=True)
        required_sequence = baseline_sequence + self.camera_warmup_frames
        deadline = (
            rospy.Time.now() +
            rospy.Duration(self.camera_service_timeout))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            with self.lock:
                sequence = self.camera_sequence
                receipt = self.latest_camera_receipt
            if sequence >= required_sequence and receipt is not None:
                rospy.loginfo(
                    "PRODUCTION_CAMERA_FRESH context=%s frames=%d",
                    context, sequence - baseline_sequence)
                return
            rospy.sleep(0.02)
        raise MissionAbort(
            "%s did not receive %d fresh camera frames after start" %
            (context, self.camera_warmup_frames))

    def release_ros_camera(self):
        """Stop only the launch-owned camera node before direct V4L2 access."""
        self.move_base.cancel_all_goals()
        self.stop_motion()
        self.qr_enable_pub.publish(Int8(data=0))
        self.publish_state("RELEASE_ROS_CAMERA")
        return_code, _stdout, stderr = self.run_subprocess(
            ["rosnode", "kill", self.camera_node_name],
            self.camera_open_timeout, "stopping ROS camera")
        if return_code != 0:
            raise MissionAbort(
                "cannot stop ROS camera %s: %s" %
                (self.camera_node_name,
                 stderr.decode("utf-8", "replace").strip()
                 if not isinstance(stderr, str) else stderr.strip()))
        rospy.sleep(0.5)
        with self.lock:
            self.camera_streaming = False
            self.latest_camera_image = None
            self.latest_camera_receipt = None
        rospy.loginfo(
            "PRODUCTION_CAMERA_RELEASED node=%s device=%s",
            self.camera_node_name, self.video_device)

    def start_native_ocr(self):
        """Start the vehicle's Python 3 live_ppocr engine."""
        self.publish_state(
            "OPEN_ROS_IMAGE_OCR" if self.use_ros_camera_for_ocr
            else "OPEN_NATIVE_CAMERA_OCR")
        command = [
            self.ocr_python,
            self.ocr_helper_path,
            "--ocr-module", self.live_ppocr_path,
            "--device", self.video_device,
            "--det", os.path.join(self.ppocr_root, "out", "det.plan"),
            "--rec", os.path.join(self.ppocr_root, "out", "rec.plan"),
            "--keys", os.path.join(self.ppocr_root, "out", "keys.txt"),
            "--width", str(self.camera_width),
            "--height", str(self.camera_height),
            "--side", str(self.ocr_side),
            "--warmup-frames", str(self.camera_warmup_frames),
            "--open-timeout", str(self.camera_open_timeout),
        ]
        if self.use_ros_camera_for_ocr:
            command.append("--ros-image-input")
        if self.camera_mirror:
            command.append("--mirror")
        log_path = os.path.join(self.run_directory, "live_ppocr.log")
        self.ocr_log_handle = open(log_path, "ab")
        try:
            self.ocr_process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=self.ocr_log_handle,
                cwd=self.ppocr_root,
                bufsize=1)
        except OSError as exc:
            self.ocr_log_handle.close()
            self.ocr_log_handle = None
            raise MissionAbort("cannot start live_ppocr helper: %s" % exc)
        message = self.read_ocr_message(
            self.ocr_helper_ready_timeout, "live_ppocr startup")
        if not message.get("ready"):
            raise MissionAbort(
                "live_ppocr helper did not report ready: %s" % message)
        rospy.loginfo(
            "PRODUCTION_NATIVE_OCR_READY mode=%s device=%s cv2=%s "
            "candidates=%s",
            message.get("mode"), self.video_device,
            message.get("cv2_version"),
            message.get("candidates"))

    def read_ocr_message(self, timeout, context):
        deadline = time.time() + float(timeout)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.require_safe()
            if self.ocr_process is None:
                raise MissionAbort("%s: OCR helper is not running" % context)
            if self.ocr_process.poll() is not None:
                raise MissionAbort(
                    "%s: OCR helper exited with code %d; see live_ppocr.log" %
                    (context, self.ocr_process.returncode))
            readable, _writable, _errors = select.select(
                [self.ocr_process.stdout], [], [], 0.1)
            if not readable:
                continue
            raw_line = self.ocr_process.stdout.readline()
            if not raw_line:
                continue
            try:
                return json.loads(raw_line)
            except ValueError:
                rospy.logwarn(
                    "PRODUCTION_OCR_NON_JSON %s",
                    raw_line.decode("utf-8", "replace").strip()
                    if not isinstance(raw_line, str) else raw_line.strip())
        raise MissionAbort("%s timed out after %.1f s" % (context, timeout))

    def capture_ocr(self, capture_label, attempt):
        if self.ocr_process is None or self.ocr_process.poll() is not None:
            raise MissionAbort("live_ppocr helper is not running")
        self.capture_sequence += 1
        safe_label = "".join(
            character if character.isalnum() else "_"
            for character in str(capture_label))
        image_path = os.path.join(
            self.run_directory,
            "capture_%05d_%s_attempt_%02d.png" % (
                self.capture_sequence, safe_label, attempt))
        payload = {
            "command": "capture",
            "output": image_path,
            "minimum_confidence": self.ocr_min_confidence,
        }
        if self.use_ros_camera_for_ocr:
            self.save_latest_ros_camera_frame(image_path)
            payload["input"] = image_path
        try:
            self.ocr_process.stdin.write(
                (json.dumps(payload) + "\n").encode("utf-8"))
            self.ocr_process.stdin.flush()
        except (IOError, OSError) as exc:
            raise MissionAbort("cannot command live_ppocr helper: %s" % exc)
        response = self.read_ocr_message(
            self.ocr_capture_timeout,
            "OCR capture %s" % capture_label)
        if not response.get("ok"):
            raise MissionAbort(
                "live_ppocr capture failed for %s: %s" %
                (capture_label, response.get("error", response)))
        return response

    def save_latest_ros_camera_frame(self, image_path):
        deadline = (
            rospy.Time.now() +
            rospy.Duration(self.camera_frame_timeout))
        message = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            with self.lock:
                receipt = self.latest_camera_receipt
                candidate = self.latest_camera_image
            if (
                    candidate is not None and receipt is not None and
                    (rospy.Time.now() - receipt).to_sec() <=
                    self.camera_frame_timeout):
                message = candidate
                break
            rospy.sleep(0.02)
        if message is None:
            raise MissionAbort(
                "no fresh ROS camera frame on %s" %
                self.camera_image_topic)
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            raise MissionAbort("cannot convert ROS camera frame: %s" % exc)
        if not cv2.imwrite(image_path, frame):
            raise MissionAbort(
                "cannot save ROS camera frame %s" % image_path)

    def start_async_motion_ocr(self, capture_label):
        """Start exactly one OCR request without blocking action supervision."""
        task = {
            "done": threading.Event(),
            "response": None,
            "error": None,
            "capture_requested_at": time.strftime(
                "%Y-%m-%dT%H:%M:%S%z"),
            "capture_requested_pose_map": list(
                self.current_map_pose(capture_label + " exposure request")),
        }

        def worker():
            try:
                task["response"] = self.capture_ocr(capture_label, 1)
            except Exception as exc:
                task["error"] = exc
            finally:
                task["done"].set()

        task["thread"] = threading.Thread(target=worker)
        task["thread"].daemon = True
        task["thread"].start()
        return task

    def finish_async_motion_ocr(self, task, keep_stopped=False):
        """Join one OCR request and return its response or original failure."""
        deadline = time.time() + self.ocr_capture_timeout + 2.0
        zero = Twist()
        while not task["done"].wait(0.02):
            if keep_stopped:
                self.cmd_vel_pub.publish(zero)
            if time.time() >= deadline:
                raise MissionAbort(
                    "asynchronous moving OCR worker did not finish")
        task["thread"].join()
        if task["error"] is not None:
            raise task["error"]
        response = task["response"]
        if isinstance(response, dict):
            # The Python 3 helper owns recognition fields, while the Python 2
            # task owns the map pose at exposure time.  Both are required to
            # restore the candidate yaw safely after asynchronous inference.
            response = dict(response)
            response["capture_requested_at"] = task["capture_requested_at"]
            response["capture_requested_pose_map"] = list(
                task["capture_requested_pose_map"])
        return response

    def cleanup_async_motion_ocr(self, task):
        if task is None:
            return
        try:
            response = self.finish_async_motion_ocr(
                task, keep_stopped=True)
            if response is not None:
                self.discard_unmatched_motion_frame(response)
        except Exception as exc:
            rospy.logerr(
                "PRODUCTION_ASYNC_OCR_CLEANUP_FAILED %s", exc)

    def stop_native_ocr(self):
        process = self.ocr_process
        self.ocr_process = None
        if process is not None and process.poll() is None:
            try:
                process.stdin.write(b'{"command":"close"}\n')
                process.stdin.flush()
            except (IOError, OSError):
                pass
            deadline = time.time() + 2.0
            while process.poll() is None and time.time() < deadline:
                rospy.sleep(0.05)
            if process.poll() is None:
                process.terminate()
                rospy.sleep(0.2)
            if process.poll() is None:
                process.kill()
            process.wait()
        if self.ocr_log_handle is not None:
            self.ocr_log_handle.close()
            self.ocr_log_handle = None
        rospy.loginfo("PRODUCTION_NATIVE_OCR_CLOSED")

    def start_qr_classifier(self):
        """Lazily start the Spark classifier helper; idempotent."""
        if not self.spark_classify_enabled:
            return False
        if self.spark_process is not None and self.spark_process.poll() is None:
            return True
        if self.spark_process is not None:
            self.stop_qr_classifier()
        command = [
            self.spark_python,
            self.spark_helper_path,
            "--api-base-url", self.spark_api_base_url,
            "--model", self.spark_model,
            "--retries", str(self.spark_retries),
            "--timeout", str(self.spark_timeout),
            "--thinking", self.spark_thinking,
        ]
        if self.spark_password_file:
            command += ["--password-file", self.spark_password_file]
        log_path = os.path.join(
            self.result_directory, "spark_classifier.log")
        try:
            self.spark_log_handle = open(log_path, "ab")
            self.spark_process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=self.spark_log_handle,
                bufsize=1)
        except (IOError, OSError) as exc:
            self.spark_process = None
            if self.spark_log_handle is not None:
                try:
                    self.spark_log_handle.close()
                except Exception:
                    pass
                self.spark_log_handle = None
            rospy.logerr(
                "PRODUCTION_SPARK_CLASSIFIER_START_FAILED %s", exc)
            return False
        message = self.read_spark_message(
            self.spark_helper_ready_timeout, "spark classifier startup")
        if not message or not message.get("ready"):
            rospy.logerr(
                "PRODUCTION_SPARK_CLASSIFIER_READY_FAILED %s",
                message if message else "no ready reply")
            self.stop_qr_classifier()
            return False
        rospy.loginfo(
            "PRODUCTION_SPARK_CLASSIFIER_READY model=%s remote=%s "
            "local_keywords=%s",
            message.get("model"), message.get("remote_configured"),
            message.get("local_keywords"))
        return True

    def read_spark_message(self, timeout, context):
        """Read one JSON line from the helper; never raises for the task."""
        deadline = time.time() + float(timeout)
        while not rospy.is_shutdown() and time.time() < deadline:
            if (self.spark_process is None or
                    self.spark_process.poll() is not None):
                rospy.logerr(
                    "PRODUCTION_SPARK_CLASSIFIER_GONE %s", context)
                return None
            readable, _writable, _errors = select.select(
                [self.spark_process.stdout], [], [], 0.1)
            if not readable:
                continue
            raw_line = self.spark_process.stdout.readline()
            if not raw_line:
                continue
            try:
                return json.loads(raw_line)
            except ValueError:
                rospy.logwarn(
                    "PRODUCTION_SPARK_NON_JSON %s",
                    raw_line.decode("utf-8", "replace").strip()
                    if not isinstance(raw_line, str) else raw_line.strip())
        return None

    def stop_qr_classifier(self):
        process = self.spark_process
        self.spark_process = None
        if process is not None and process.poll() is None:
            try:
                process.stdin.write(
                    (json.dumps({"command": "close"}) + "\n").encode("utf-8"))
                process.stdin.flush()
            except (IOError, OSError):
                pass
            deadline = time.time() + 3.0
            while process.poll() is None and time.time() < deadline:
                rospy.sleep(0.05)
            if process.poll() is None:
                process.terminate()
                rospy.sleep(0.2)
            if process.poll() is None:
                process.kill()
            process.wait()
        if self.spark_log_handle is not None:
            try:
                self.spark_log_handle.close()
            except Exception:
                pass
            self.spark_log_handle = None
        rospy.loginfo("PRODUCTION_SPARK_CLASSIFIER_CLOSED")

    def log_safe_text(self, value):
        """ASCII-only form of a text value for Python 2 log lines.

        Any Unicode is JSON-escaped (ensure_ascii), so rospy log messages can
        never hit the Python 2 implicit-ascii decode path (which crashed the
        task with UnicodeDecodeError once a Chinese category was logged).
        """
        if value is None:
            return ""
        if isinstance(value, bytes):
            try:
                value = value.decode("utf-8")
            except UnicodeDecodeError:
                value = value.decode("utf-8", "replace")
        if isinstance(value, str):
            return value
        return json.dumps(value, ensure_ascii=True)

    def classify_qr_text(self, observation_number, qr_text):
        """Classify one QR result; records the outcome, never aborts."""
        if not self.spark_classify_enabled:
            return
        # /qr_result arrives as UTF-8 bytes on Python 2 (and str on Python 3).
        # Normalize to unicode so json.dumps(payload) never hits the Python 2
        # implicit-ascii decode path for non-ASCII bytes.
        if isinstance(qr_text, bytes):
            try:
                qr_text = qr_text.decode("utf-8")
            except UnicodeDecodeError:
                qr_text = qr_text.decode("utf-8", "replace")
        entry = {
            "observation": int(observation_number),
            "qr_text": qr_text,
            "category": None,
            "source": "none",
            "attempts": 0,
            "model": self.spark_model,
            "error": "",
        }
        if not qr_text:
            rospy.logwarn(
                "PRODUCTION_SPARK_CLASSIFY observation=%d empty qr_text",
                observation_number)
            self.qr_classifications.append(entry)
            return
        if not self.start_qr_classifier():
            entry["error"] = "classifier helper unavailable"
            rospy.logerr(
                "PRODUCTION_SPARK_CLASSIFY observation=%d qr=%s "
                "category=null source=none error=%s (cannot reach spark model)",
                observation_number, self.log_safe_text(qr_text),
                self.log_safe_text(entry["error"]))
            self.qr_classifications.append(entry)
            return
        try:
            payload = {"command": "classify", "qr_text": qr_text}
            self.spark_process.stdin.write(
                (json.dumps(payload) + "\n").encode("utf-8"))
            self.spark_process.stdin.flush()
        except (IOError, OSError) as exc:
            entry["error"] = "cannot command classifier: %s" % exc
            rospy.logerr(
                "PRODUCTION_SPARK_CLASSIFY observation=%d qr=%s "
                "category=null source=none error=%s (cannot reach spark model)",
                observation_number, self.log_safe_text(qr_text),
                self.log_safe_text(entry["error"]))
            self.qr_classifications.append(entry)
            return
        response = self.read_spark_message(
            self.spark_timeout, "classify observation %d" %
            observation_number)
        if not response:
            entry["error"] = "no reply from classifier helper"
            rospy.logerr(
                "PRODUCTION_SPARK_CLASSIFY observation=%d qr=%s "
                "category=null source=none error=%s (cannot reach spark model)",
                observation_number, self.log_safe_text(qr_text),
                self.log_safe_text(entry["error"]))
            self.qr_classifications.append(entry)
            return
        entry["category"] = response.get("category")
        entry["source"] = response.get("source", "none")
        entry["attempts"] = response.get("attempts", 0)
        entry["model"] = response.get("model", self.spark_model)
        entry["error"] = response.get("error", "")
        self.qr_classifications.append(entry)
        if entry["source"] == "none":
            rospy.logerr(
                "PRODUCTION_SPARK_CLASSIFY observation=%d qr=%s "
                "category=null source=none attempts=%d error=%s "
                "(cannot reach spark model)",
                observation_number, self.log_safe_text(qr_text),
                entry["attempts"],
                self.log_safe_text(entry["error"]))
        else:
            rospy.loginfo(
                "PRODUCTION_SPARK_CLASSIFY observation=%d qr=%s "
                "category=%s source=%s attempts=%d",
                observation_number, self.log_safe_text(qr_text),
                self.log_safe_text(entry["category"]),
                entry["source"], entry["attempts"])

    def rotate_for_pixel_error(
            self, error_pixels, context, previous_error_pixels=None,
            sample_seconds=None):
        speed = alignment_angular_speed(
            error_pixels, self.ocr_alignment_kp, self.ocr_alignment_kd,
            self.ocr_alignment_max_speed, self.camera_mirror,
            previous_error_pixels, sample_seconds)
        target_delta = abs(speed) * self.ocr_alignment_step_seconds
        self.rotate_in_place_for_yaw(
            speed, target_delta, context + " protected alignment")

    def rotate_in_place_for_yaw(self, signed_speed, target_delta, context):
        """Apply a measured small yaw correction without a move_base goal.

        Point-mode move_base can accept a same-position goal without reaching
        its requested orientation.  OCR alignment therefore owns this short
        rotation and only succeeds after odometry proves the signed yaw change.
        """
        speed = float(signed_speed)
        target = abs(float(target_delta))
        if target <= self.ocr_alignment_yaw_tolerance:
            return
        direction = 1.0 if speed > 0.0 else -1.0
        # The chassis MCU has a rotation dead zone; a PD-computed speed below
        # it stalls the yaw entirely (measured 0.004 rad in 2.5 s at 0.073
        # rad/s).  Lift the command above the dead zone while keeping the
        # requested direction; progress accumulation and stop confirmation
        # bound any overshoot.
        if abs(speed) < self.ocr_alignment_min_speed:
            speed = self.ocr_alignment_min_speed * direction
        self.move_base.cancel_all_goals()
        self.stop_motion()
        self.wait_for_chassis_stop(context + " start")
        previous_yaw = self.current_odom_yaw(context + " start")
        progress = 0.0
        required_progress = max(
            0.0, target - self.ocr_alignment_yaw_tolerance)
        deadline = (
            rospy.Time.now() +
            rospy.Duration(self.ocr_alignment_turn_timeout))
        rate = rospy.Rate(self.rotation_control_rate)
        command = Twist()
        command.angular.z = speed
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                self.require_safe()
                current_yaw = self.current_odom_yaw(context)
                progress += positive_turn_increment(
                    previous_yaw, current_yaw, direction)
                previous_yaw = current_yaw
                if progress >= required_progress:
                    self.stop_motion()
                    self.wait_for_chassis_stop(context + " complete")
                    rospy.loginfo(
                        "PRODUCTION_OCR_ALIGNMENT_TURN context=%s "
                        "command_speed=%.3f requested=%.3f actual=%.3f",
                        context, speed, target, progress)
                    return
                self.cmd_vel_pub.publish(command)
                rate.sleep()
        finally:
            self.stop_motion()
        # The final command can cross the measured-yaw threshold between the
        # last control sample and deadline.  Do not keep rotating after the
        # deadline: prove the chassis has stopped, then account for that last
        # finite odom sample before declaring the correction unsuccessful.
        self.wait_for_chassis_stop(context + " timeout settle")
        final_yaw = self.current_odom_yaw(context + " timeout settle")
        progress += positive_turn_increment(
            previous_yaw, final_yaw, direction)
        if progress >= required_progress:
            rospy.loginfo(
                "PRODUCTION_OCR_ALIGNMENT_TURN context=%s "
                "command_speed=%.3f requested=%.3f actual=%.3f "
                "settled=true",
                context, speed, target, progress)
            return
        raise MissionAbort(
            "%s did not reach %.3f rad of measured yaw within %.1f s "
            "(actual=%.3f required=%.3f)" %
            (context, target, self.ocr_alignment_turn_timeout,
             progress, required_progress))

    def wait_for_chassis_stop(self, context):
        """Require fresh low-velocity odometry before any alignment rotation."""
        deadline = (
            rospy.Time.now() +
            rospy.Duration(self.stop_confirmation_timeout))
        with self.lock:
            previous_receipt = self.latest_odom_receipt
        stopped_samples = 0
        zero = Twist()
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            self.cmd_vel_pub.publish(zero)
            with self.lock:
                receipt = self.latest_odom_receipt
                velocity = self.latest_odom_velocity
            if receipt is not None and receipt != previous_receipt:
                previous_receipt = receipt
                if odom_velocity_is_stopped(
                        velocity, self.stopped_odom_speed_epsilon):
                    stopped_samples += 1
                    if stopped_samples >= self.stopped_odom_samples:
                        rospy.loginfo(
                            "PRODUCTION_STOP_CONFIRMED context=%s samples=%d",
                            context, stopped_samples)
                        return
                else:
                    stopped_samples = 0
            rospy.sleep(0.02)
        raise MissionAbort(
            "%s did not confirm %d stopped odom samples within %.1f s" %
            (context, self.stopped_odom_samples,
             self.stop_confirmation_timeout))

    def cancel_navigation_for_observation(self, context):
        """Cancel only this task's active goal and prove the chassis stopped."""
        status_before_cancel = self.move_base.get_state()
        rospy.loginfo(
            "PRODUCTION_CANCEL_FOR_OCR context=%s status_before=%d",
            context, status_before_cancel)
        self.move_base.cancel_goal()
        self.stop_motion()
        deadline = (
            rospy.Time.now() + rospy.Duration(self.goal_cancel_timeout))
        acceptable = (
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.RECALLED,
        )
        terminal_failure = (
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.LOST,
        )
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            status = self.move_base.get_state()
            if status in acceptable:
                self.wait_for_chassis_stop(context)
                return status
            if status in terminal_failure:
                raise MissionAbort(
                    "%s navigation ended with unsafe status %d while "
                    "stopping for OCR" % (context, status))
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.02)
        raise MissionAbort(
            "%s goal cancellation was not acknowledged within %.1f s" %
            (context, self.goal_cancel_timeout))

    def discard_unmatched_motion_frame(self, response):
        image_path = response.get("image_path", "")
        if not image_path:
            return
        try:
            os.remove(image_path)
        except OSError as exc:
            rospy.logwarn(
                "PRODUCTION_OCR_FRAME_CLEANUP_FAILED path=%s error=%s",
                image_path, exc)

    def verify_segment_arrival(
            self, end_number, status, label, target_coordinate):
        pose = self.current_map_pose(label + " arrival")
        error = position_error(pose, target_coordinate)
        if status != GoalStatus.SUCCEEDED and error > self.arrival_tolerance:
            raise MissionAbort(
                "%s failed with move_base status %d at %.3f m from point %d" %
                (label, status, error, end_number))
        if error > self.arrival_tolerance:
            raise MissionAbort(
                "%s stopped %.3f m from point %d (limit %.3f m)" %
                (label, error, end_number, self.arrival_tolerance))
        self.stop_motion()
        rospy.loginfo(
            "PRODUCTION_SEGMENT_REACHED label=%s point=%d error=%.3f "
            "move_base_status=%d",
            label, end_number, error, status)

    def navigate_target_and_scan(
            self, leg_index, start_number, end_number, target_yaw):
        """Guard one target leg, then scan only if that target was reached."""
        target = self.points[end_number]
        label = "PRODUCTION_TARGET_%03d" % end_number
        self.publish_state(label)
        rospy.loginfo(
            "PRODUCTION_TARGET_GOAL index=%d/%d start=%d end=%d "
            "target=(%.3f, %.3f) yaw=%.3f",
            leg_index, len(self.production_navigation_legs),
            start_number, end_number, target[0], target[1], target_yaw)
        monitor = self.new_target_guard_monitor(end_number)
        guard_number = self.wait_for_target_guard_precheck(monitor)
        if guard_number is not None:
            self.stop_motion()
            self.wait_for_chassis_stop(label + " target guard before goal")
            self.record_target_guard_skip(
                leg_index, start_number, end_number, guard_number,
                "before_goal", monitor)
            return "target_guard_skipped"

        navigation_guard = {"number": None, "scan_unavailable": False}

        def guard_callback():
            detected_number = self.poll_target_guard(monitor)
            if detected_number is not None:
                navigation_guard["number"] = detected_number
                return True
            if self.target_guard_scan_expired(monitor):
                navigation_guard["scan_unavailable"] = True
                return True
            return False

        reached = self.navigate_coordinates(
            target[0], target[1], target_yaw, label, require_plan=True,
            guard_callback=guard_callback)
        if navigation_guard["number"] is not None:
            self.record_target_guard_skip(
                leg_index, start_number, end_number,
                navigation_guard["number"], "during_navigation", monitor)
            return "target_guard_skipped"
        if navigation_guard["scan_unavailable"]:
            raise MissionAbort(
                "%s target guard scan became unavailable during navigation" %
                label)
        if not reached:
            raise MissionAbort("%s did not reach target" % label)
        self.scan_production_point(leg_index, start_number, end_number, label)

    def new_target_guard_monitor(self, target_number):
        """Start a new guard epoch; old scans may not affect a new target."""
        target_number = int(target_number)
        if target_number not in self.target_guard_points:
            raise MissionAbort("target %d has no guard point mapping" %
                               target_number)
        with self.lock:
            sequence = self.target_guard_scan_sequence
        return {
            "target_number": target_number,
            "last_sequence": sequence,
            "usable_scan_seen": False,
            "clean_scan_seen": False,
            "hit_counts": {},
            "last_errors": {},
            "last_scan_stamp": None,
            "last_source_stamp": None,
            "last_usable_receipt": None,
        }

    @staticmethod
    def clear_target_guard_hits(monitor):
        """Break a consecutive-hit chain after invalid or clear evidence."""
        monitor["hit_counts"] = {}
        monitor["last_errors"] = {}

    def poll_target_guard(self, monitor):
        """Return a confirmed guard number from one distinct dynamic scan."""
        with self.lock:
            scan = self.latest_target_guard_scan
            receipt = self.latest_target_guard_scan_receipt
            sequence = self.target_guard_scan_sequence
        if scan is None or receipt is None or sequence <= monitor["last_sequence"]:
            return None
        monitor["last_sequence"] = sequence
        now = rospy.Time.now()
        receipt_age = (now - receipt).to_sec()
        source_stamp = scan.header.stamp
        source_age = (now - source_stamp).to_sec()
        previous_source_stamp = monitor["last_source_stamp"]
        source_gap = (
            (source_stamp - previous_source_stamp).to_sec()
            if previous_source_stamp is not None else None)
        if (
                source_stamp.is_zero() or
                previous_source_stamp is not None and
                source_stamp <= previous_source_stamp or
                receipt_age > self.target_guard_scan_max_age or
                source_age > self.target_guard_scan_max_age or
                source_age < -self.target_guard_scan_max_age):
            self.clear_target_guard_hits(monitor)
            monitor["clean_scan_seen"] = False
            rospy.logwarn_throttle(
                2.0,
                "PRODUCTION_TARGET_GUARD_INVALID_SCAN target=%d "
                "receipt_age=%.3f source_age=%.3f duplicate=%s zero=%s",
                monitor["target_number"], receipt_age, source_age,
                str(previous_source_stamp is not None and
                    source_stamp <= previous_source_stamp),
                str(source_stamp.is_zero()))
            return None
        if source_gap is not None and source_gap > self.target_guard_scan_max_age:
            self.clear_target_guard_hits(monitor)
            monitor["clean_scan_seen"] = False
            rospy.logwarn_throttle(
                2.0, "PRODUCTION_TARGET_GUARD_GAP_RESET target=%d gap=%.3f",
                monitor["target_number"], source_gap)
        monitor["last_source_stamp"] = source_stamp
        try:
            laser_pose = self.laser_map_pose(scan)
        except MissionAbort as exc:
            self.clear_target_guard_hits(monitor)
            monitor["clean_scan_seen"] = False
            rospy.logwarn_throttle(
                2.0, "PRODUCTION_TARGET_GUARD_TF_SKIP target=%d error=%s",
                monitor["target_number"], exc)
            return None
        monitor["usable_scan_seen"] = True
        monitor["last_usable_receipt"] = receipt
        matches = target_guard_scan_matches(
            scan, laser_pose,
            self.target_guard_points[monitor["target_number"]],
            self.target_guard_match_radius)
        hit_counts = monitor["hit_counts"]
        for number in list(hit_counts):
            if number not in matches:
                del hit_counts[number]
        for number, error in matches.items():
            hit_counts[number] = hit_counts.get(number, 0) + 1
            monitor["last_errors"][number] = float(error)
        monitor["clean_scan_seen"] = not bool(matches)
        monitor["last_scan_stamp"] = scan.header.stamp
        confirmed = [
            number for number, count in hit_counts.items()
            if count >= self.target_guard_confirmation_scans]
        if not confirmed:
            return None
        return min(
            confirmed,
            key=lambda number: (monitor["last_errors"].get(number, float("inf")),
                                int(number)))

    def target_guard_scan_expired(self, monitor):
        """True if navigation lost the last TF-projected guard scan."""
        last_usable_receipt = monitor.get("last_usable_receipt")
        if last_usable_receipt is None:
            return True
        return (
            (rospy.Time.now() - last_usable_receipt).to_sec() >
            self.target_guard_scan_max_age)

    def wait_for_target_guard_precheck(self, monitor):
        """Use fresh pre-goal evidence without delaying a clean target leg."""
        deadline = (
            rospy.Time.now() +
            rospy.Duration(self.target_guard_precheck_timeout))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            guard_number = self.poll_target_guard(monitor)
            if guard_number is not None:
                return guard_number
            # A TF-projected clean frame is sufficient to start the goal;
            # a one-frame hit stays in this loop for confirmation.  A raw
            # scan without a valid map projection must never mean "clear".
            if monitor["clean_scan_seen"]:
                return None
            rospy.sleep(0.02)
        if not monitor["usable_scan_seen"]:
            raise MissionAbort(
                "target guard scan unavailable for target %d within %.2f s" %
                (monitor["target_number"], self.target_guard_precheck_timeout))
        if monitor["hit_counts"]:
            rospy.logwarn(
                "PRODUCTION_TARGET_GUARD_UNCONFIRMED target=%d hits=%s",
                monitor["target_number"], monitor["hit_counts"])
        return None

    def record_target_guard_skip(
            self, leg_index, start_number, end_number, guard_number, phase,
            monitor):
        """Audit a guard decision before the outer route advances one leg."""
        next_target = (
            self.production_navigation_legs[leg_index][1]
            if leg_index < len(self.production_navigation_legs) else None)
        stamp = monitor.get("last_scan_stamp")
        event = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "target_point_number": int(end_number),
            "guard_point_number": int(guard_number),
            "guard_point_numbers": sorted(self.target_guard_points[end_number]),
            "segment_index": int(leg_index),
            "segment_start_point_number": int(start_number),
            "phase": str(phase),
            "scan_match_error_m": float(
                monitor["last_errors"].get(guard_number, float("inf"))),
        }
        if stamp is not None and not stamp.is_zero():
            event["scan_stamp"] = float(stamp.to_sec())
        self.target_guard_events.append(event)
        self.target_scan_events.append({
            "target_point_number": int(end_number),
            "outcome": "target_guard_skipped",
            "guard_point_number": int(guard_number),
            "phase": str(phase),
        })
        self.publish_state(
            "TARGET_GUARD_SKIP_%03d_%03d" % (end_number, guard_number))
        rospy.logwarn(
            "PRODUCTION_TARGET_GUARD_SKIP target=%d guard_point=%d "
            "phase=%s next_target=%s",
            end_number, guard_number, phase,
            str(next_target) if next_target is not None else "none")
        self.save_observation_summary()

    def scan_production_point(
            self, leg_index, start_number, point_number, target_label):
        """Complete one stationary 360-degree scan and record new classes."""
        scan_label = "PRODUCTION_OCR_TURN_%03d" % point_number
        self.publish_state(scan_label)
        if self.use_ros_camera_for_ocr:
            self.start_ros_camera_and_wait(scan_label)
        try:
            recorded_categories = set(
                item["processing_category"]
                for item in select_three_processing_observations(
                    self.observations))

            def handle_candidate(response, turn_progress):
                detection = response["detection"]
                category = normalize_production_category(detection.get("text"))
                if category is None or category in recorded_categories:
                    return False
                self.stop_motion()
                self.wait_for_chassis_stop(scan_label + " candidate")
                self.restore_ocr_capture_yaw(response, scan_label)
                observation_label = "%s_%s" % (
                    scan_label, category.encode("utf-8"))
                self.publish_state(observation_label)
                observation = self.observe_wall(point_number, observation_label)
                observation.update({
                    "processing_category": category,
                    "segment_index": int(leg_index),
                    "segment_start_point_number": int(start_number),
                    "segment_end_point_number": int(point_number),
                    "turn_progress_radians": float(turn_progress),
                    "turn_detection_image_path": response["image_path"],
                    "turn_detection": detection,
                    "turn_detection_capture_requested_at":
                        response["capture_requested_at"],
                    "turn_detection_pose_map":
                        list(response["capture_requested_pose_map"]),
                })
                event = {
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                    "target_point_number": int(point_number),
                    "segment_index": int(leg_index),
                    "segment_start_point_number": int(start_number),
                    "turn_progress_radians": float(turn_progress),
                    "processing_category": category,
                    "text": detection["text"],
                    "confidence": float(detection["confidence"]),
                    "observation_aligned": bool(observation["aligned"]),
                    "wall_point_number": observation.get("wall_point_number"),
                }
                if observation["aligned"] and observation.get("wall_point_number") is not None:
                    recorded_categories.add(category)
                    self.observations.append(observation)
                    event["outcome"] = "processing_category_recorded"
                    rospy.loginfo(
                        "PRODUCTION_CATEGORY_RECORDED category=%s route_point=%d "
                        "wall_point=%d coordinate=(%.3f,%.3f) text=%s",
                        category.encode("utf-8"), point_number,
                        observation["wall_point_number"],
                        observation["wall_point_coordinate"][0],
                        observation["wall_point_coordinate"][1],
                        json.dumps(observation["text"], ensure_ascii=True))
                else:
                    event["outcome"] = "processing_category_rejected"
                    rospy.logwarn(
                        "PRODUCTION_CATEGORY_REJECTED category=%s route_point=%d "
                        "aligned=%s range_residual=%s",
                        category.encode("utf-8"), point_number,
                        observation["aligned"],
                        str(observation.get("range_residual_m")))
                self.target_scan_events.append(event)
                self.save_observation_summary()
                # Alignment changes yaw.  Return to the actual exposure yaw
                # before resuming so the remaining scan covers one real circle.
                self.restore_ocr_capture_yaw(response, scan_label + " resume")
                return True

            _response, turn_progress = self.rotate_full_revolution_for_ocr(
                scan_label, candidate_handler=handle_candidate)
            event = {
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "target_point_number": int(point_number),
                "segment_index": int(leg_index),
                "segment_start_point_number": int(start_number),
                "turn_progress_radians": float(turn_progress),
            }
            event["outcome"] = "ocr_full_turn_complete"
            self.target_scan_events.append(event)
            self.save_observation_summary()
            rospy.loginfo(
                "PRODUCTION_OCR_TURN_COMPLETE target=%d progress=%.3f "
                "categories=%d",
                point_number, turn_progress, len(recorded_categories))
            return None
        finally:
            if self.use_ros_camera_for_ocr:
                self.stop_ros_camera_streaming(required=not rospy.is_shutdown())

    def restore_ocr_capture_yaw(self, response, context):
        """Return to the candidate frame yaw before the alignment re-capture."""
        capture_pose = response.get("capture_requested_pose_map")
        if not isinstance(capture_pose, (list, tuple)) or len(capture_pose) != 3:
            raise MissionAbort(
                "%s OCR candidate has no capture pose" % context)
        if not all(is_finite(float(value)) for value in capture_pose):
            raise MissionAbort(
                "%s OCR candidate capture pose is not finite" % context)
        current = self.current_map_pose(context + " restore yaw start")
        self.navigate_coordinates(
            current[0], current[1], float(capture_pose[2]),
            context + " restore capture yaw",
            require_plan=False, require_action_success=True)
        self.wait_for_chassis_stop(context + " restore capture yaw")

    def rotate_full_revolution_for_ocr(self, label, candidate_handler=None):
        """Turn one circle; a handler may stop/process multiple candidates.

        Without a handler this preserves the legacy first-candidate contract.
        With one, only commanded turning contributes to progress: any stopped
        OCR alignment/range pause extends the deadline and resets the yaw
        baseline before the remaining arc resumes.
        """
        self.move_base.cancel_all_goals()
        self.stop_motion()
        self.wait_for_chassis_stop(label + " start")
        self.require_safe()
        direction = 1.0
        speed = self.ocr_scan_rotation_speed
        previous_yaw = self.current_odom_yaw(label)
        progress = 0.0
        target_progress = 2.0 * math.pi
        timeout = target_progress / speed * self.rotation_timeout_scale + 2.0
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        next_capture = rospy.Time.now()
        capture_index = 0
        capture_task = None
        rate = rospy.Rate(self.rotation_control_rate)
        rospy.loginfo(
            "PRODUCTION_OCR_TURN_START label=%s speed=%.3f timeout=%.1f",
            label, speed, timeout)
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                self.require_safe()
                current_yaw = self.current_odom_yaw(label)
                progress += positive_turn_increment(
                    previous_yaw, current_yaw, direction)
                previous_yaw = current_yaw

                if capture_task is not None and capture_task["done"].is_set():
                    completed_task = capture_task
                    capture_task = None
                    response = self.finish_async_motion_ocr(completed_task)
                    if is_navigation_ocr_candidate(
                            response, self.ocr_scan_candidate_confidence):
                        rospy.loginfo(
                            "PRODUCTION_OCR_TURN_CANDIDATE label=%s "
                            "progress=%.3f text=%s confidence=%.1f",
                            label, progress,
                            json.dumps(response["detection"]["text"],
                                       ensure_ascii=True),
                            response["detection"]["confidence"])
                        if candidate_handler is None:
                            self.stop_motion()
                            self.wait_for_chassis_stop(label + " candidate")
                            return response, progress
                        paused_at = rospy.Time.now()
                        handled = candidate_handler(response, progress)
                        deadline += rospy.Time.now() - paused_at
                        previous_yaw = self.current_odom_yaw(
                            label + " candidate resume")
                        if not handled:
                            self.discard_unmatched_motion_frame(response)
                    else:
                        self.discard_unmatched_motion_frame(response)
                    next_capture = (
                        rospy.Time.now() +
                        rospy.Duration(self.ocr_scan_poll_period))

                if progress >= (
                        target_progress - self.rotation_completion_tolerance):
                    self.stop_motion()
                    self.wait_for_chassis_stop(label + " complete")
                    if capture_task is not None:
                        completed_task = capture_task
                        capture_task = None
                        response = self.finish_async_motion_ocr(
                            completed_task, keep_stopped=True)
                        if is_navigation_ocr_candidate(
                                response,
                                self.ocr_scan_candidate_confidence):
                            rospy.loginfo(
                                "PRODUCTION_OCR_TURN_FINAL_CANDIDATE "
                                "label=%s progress=%.3f", label, progress)
                            if candidate_handler is None:
                                return response, progress
                            paused_at = rospy.Time.now()
                            handled = candidate_handler(response, progress)
                            deadline += rospy.Time.now() - paused_at
                            if not handled:
                                self.discard_unmatched_motion_frame(response)
                        else:
                            self.discard_unmatched_motion_frame(response)
                    rospy.loginfo(
                        "PRODUCTION_OCR_TURN_COMPLETE label=%s progress=%.3f",
                        label, progress)
                    return None, progress

                if (
                        capture_task is None and
                        rospy.Time.now() >= next_capture):
                    capture_index += 1
                    capture_task = self.start_async_motion_ocr(
                        "%s_capture_%04d" % (label, capture_index))

                command = Twist()
                command.angular.z = direction * speed
                self.cmd_vel_pub.publish(command)
                rate.sleep()
        finally:
            self.stop_motion()
            if capture_task is not None:
                self.cleanup_async_motion_ocr(capture_task)
        raise MissionAbort(
            "%s did not complete a 360-degree OCR turn within %.1f s" %
            (label, timeout))

    def navigate_segment_with_continuous_ocr(
            self, segment_index, start_number, end_number,
            target_yaw=None):
        """Retired safety gate: production OCR may not run while moving."""
        raise MissionAbort(
            "continuous moving OCR is retired; use navigate_target_and_scan")
        start_coordinate = self.points[start_number]
        end_coordinate = self.points[end_number]
        if target_yaw is None:
            target_yaw = bearing(start_coordinate, end_coordinate)
        label = "PRODUCTION_SEGMENT_%03d_%03d" % (
            start_number, end_number)
        observations_this_segment = 0
        scan_index = 0

        while not rospy.is_shutdown():
            self.require_safe()
            current = self.current_map_pose(label + " resume")
            if position_error(current, end_coordinate) <= self.arrival_tolerance:
                rospy.loginfo(
                    "PRODUCTION_SEGMENT_ALREADY_REACHED label=%s",
                    label)
                return
            self.wait_for_plan(
                end_coordinate[0], end_coordinate[1], target_yaw, label)
            goal = MoveBaseGoal()
            goal.target_pose = self.map_pose(
                end_coordinate[0], end_coordinate[1], target_yaw)
            self.publish_state(label)
            rospy.loginfo(
                "PRODUCTION_SEGMENT_GOAL index=%d/%d start=%d end=%d "
                "target=(%.3f, %.3f) yaw=%.3f",
                segment_index, len(self.production_segments),
                start_number, end_number, end_coordinate[0],
                end_coordinate[1], target_yaw)
            self.move_base.send_goal(goal)
            deadline = (
                rospy.Time.now() + rospy.Duration(self.goal_timeout))
            next_capture = rospy.Time.now()
            capture_task = None
            try:
                while not rospy.is_shutdown():
                    self.require_safe()
                    status = self.move_base.get_state()
                    if status in (
                            GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED,
                            GoalStatus.ABORTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.LOST):
                        self.stop_motion()
                        if capture_task is not None:
                            completed_task = capture_task
                            capture_task = None
                            response = self.finish_async_motion_ocr(
                                completed_task, keep_stopped=True)
                            self.discard_unmatched_motion_frame(response)
                        self.verify_segment_arrival(
                            end_number, status, label, end_coordinate)
                        return
                    if rospy.Time.now() >= deadline:
                        self.move_base.cancel_goal()
                        self.stop_motion()
                        raise MissionAbort(
                            "%s timed out after %.1f s" %
                            (label, self.goal_timeout))

                    if capture_task is not None:
                        if not capture_task["done"].is_set():
                            rospy.sleep(0.02)
                            continue
                        completed_task = capture_task
                        capture_task = None
                        response = self.finish_async_motion_ocr(
                            completed_task)
                        next_capture = (
                            rospy.Time.now() +
                            rospy.Duration(
                                self.navigation_ocr_poll_period))

                        # The goal may have ended while TensorRT processed the
                        # frame.  Never rotate or range on a stale moving
                        # candidate; the next loop handles the terminal state.
                        status = self.move_base.get_state()
                        if status not in (
                                GoalStatus.PENDING, GoalStatus.ACTIVE,
                                GoalStatus.PREEMPTING,
                                GoalStatus.RECALLING):
                            self.discard_unmatched_motion_frame(response)
                            continue
                        if not is_navigation_ocr_candidate(
                                response,
                                self.navigation_ocr_candidate_confidence):
                            self.discard_unmatched_motion_frame(response)
                            continue

                        detection = response["detection"]
                        trigger_pose = tuple(
                            completed_task[
                                "capture_requested_pose_map"])
                        self.publish_state(
                            "PRODUCTION_OCR_CANDIDATE_%03d_%03d" %
                            (start_number, end_number))
                        rospy.loginfo(
                            "PRODUCTION_MOVING_OCR_CANDIDATE "
                            "segment=%d->%d text=%s confidence=%.1f "
                            "exposure_pose=(%.3f, %.3f, %.3f)",
                            start_number, end_number,
                            json.dumps(
                                detection["text"], ensure_ascii=True),
                            detection["confidence"], trigger_pose[0],
                            trigger_pose[1], trigger_pose[2])
                        cancel_status = (
                            self.cancel_navigation_for_observation(label))
                        if cancel_status == GoalStatus.SUCCEEDED:
                            rospy.loginfo(
                                "PRODUCTION_OCR_CANDIDATE_AT_SEGMENT_END "
                                "segment=%d->%d; discarding moving "
                                "candidate without observation",
                                start_number, end_number)
                            self.discard_unmatched_motion_frame(response)
                            self.verify_segment_arrival(
                                end_number, cancel_status, label,
                                end_coordinate)
                            return
                        stopped_pose = self.current_map_pose(
                            label + " stopped OCR pose")
                        observation_label = (
                            "%s_observation_%02d" %
                            (label, observations_this_segment + 1))
                        self.publish_state(observation_label)
                        observation = self.observe_wall(
                            end_number, observation_label,
                            candidate_wall_points=
                            self.wall_candidates_for_target(end_number))
                        observation.update({
                            "segment_index": int(segment_index),
                            "segment_start_point_number": int(start_number),
                            "segment_end_point_number": int(end_number),
                            "moving_detection_image_path":
                                response["image_path"],
                            "moving_detection": detection,
                            "moving_detection_capture_requested_at":
                                completed_task["capture_requested_at"],
                            "moving_detection_pose_map":
                                list(trigger_pose),
                            "stopped_pose_map": list(stopped_pose),
                        })
                        if self.target_guard_triggered(
                                end_number, observation):
                            guard_number = observation["wall_point_number"]
                            event = {
                                "timestamp": time.strftime(
                                    "%Y-%m-%dT%H:%M:%S%z"),
                                "target_point_number": int(end_number),
                                "guard_point_number": int(guard_number),
                                "guard_point_numbers": sorted(
                                    self.target_guard_points[end_number]),
                                "segment_index": int(segment_index),
                                "segment_start_point_number": int(
                                    start_number),
                            }
                            observation["target_guard_triggered"] = True
                            self.target_guard_events.append(event)
                            self.observations.append(observation)
                            self.last_observation_pose = self.current_map_pose(
                                label + " target guard complete")
                            self.save_observation_summary()
                            rospy.logwarn(
                                "PRODUCTION_TARGET_GUARD_SKIP target=%d "
                                "guard_point=%d next_target=%s",
                                end_number, guard_number,
                                (str(self.production_navigation_legs[
                                    segment_index][1])
                                 if segment_index < len(
                                     self.production_navigation_legs)
                                 else "none"))
                            return "target_guard_skipped"
                        self.observations.append(observation)
                        self.last_observation_pose = self.current_map_pose(
                            label + " observation complete")
                        observations_this_segment += 1
                        self.save_observation_summary()
                        rospy.loginfo(
                            "PRODUCTION_SEGMENT_RESUME start=%d end=%d "
                            "observations=%d",
                            start_number, end_number,
                            observations_this_segment)
                        break

                    can_scan = (
                        (
                            observations_this_segment <
                            self.navigation_ocr_max_observations_per_segment or
                            bool(self.target_guard_points.get(end_number))) and
                        rospy.Time.now() >= next_capture and
                        self.continuous_ocr_is_armed(
                            continue_for_target_guard=bool(
                                self.target_guard_points.get(end_number))))
                    if can_scan:
                        scan_index += 1
                        capture_label = "%s_motion_%04d" % (
                            label, scan_index)
                        capture_task = self.start_async_motion_ocr(
                            capture_label)
                    rospy.sleep(0.02)
            finally:
                if capture_task is not None:
                    self.move_base.cancel_goal()
                    self.stop_motion()
                    self.cleanup_async_motion_ocr(capture_task)

    def target_guard_triggered(self, target_number, observation):
        """Return whether an accepted lidar match protects this target."""
        guard_points = self.target_guard_points.get(int(target_number), {})
        wall_number = observation.get("wall_point_number")
        return wall_number is not None and int(wall_number) in guard_points

    def wall_candidates_for_target(self, target_number):
        """Allow a target guard without discarding normal wall references."""
        candidates = dict(self.wall_reference_points)
        candidates.update(self.target_guard_points.get(int(target_number), {}))
        return candidates

    def observe_wall(
            self, route_point_number, observation_label=None,
            candidate_wall_points=None):
        """Capture, OCR-align, range, and match after a proven full stop."""
        if observation_label is None:
            observation_label = "point_%03d" % route_point_number
        self.stop_motion()
        previous_error = None
        previous_capture_time = None
        divergence_count = 0
        best_detection = None
        best_path = ""
        aligned = False
        image_width = self.camera_width
        attempt_image_paths = []

        for attempt in range(1, self.ocr_alignment_attempts + 1):
            self.require_safe()
            response = self.capture_ocr(observation_label, attempt)
            # Python 2.7 on the vehicle has no time.monotonic().  The PD
            # derivative only needs elapsed time between adjacent parked
            # captures; ROS wall time is sufficient and Python 2-compatible.
            capture_time = time.time()
            image_path = response["image_path"]
            attempt_image_paths.append(image_path)
            image_width = int(response["width"])
            detection = response.get("detection")
            if detection is None:
                rospy.logwarn(
                    "PRODUCTION_OCR_EMPTY point=%d attempt=%d/%d",
                    route_point_number, attempt,
                    self.ocr_alignment_attempts)
                continue
            best_detection = detection
            best_path = image_path
            error = horizontal_pixel_error(detection, image_width)
            rospy.loginfo(
                "PRODUCTION_OCR_BOX point=%d attempt=%d text=%s "
                "confidence=%.1f horizontal_error_px=%.1f",
                route_point_number, attempt,
                json.dumps(detection["text"], ensure_ascii=True),
                detection["confidence"], error)
            if abs(error) <= self.ocr_alignment_tolerance_px:
                aligned = True
                break
            if (previous_error is not None and
                    abs(error) > abs(previous_error) * 1.35):
                divergence_count += 1
            else:
                divergence_count = 0
            if divergence_count >= 2:
                raise MissionAbort(
                    "OCR PD alignment diverged twice at point %d; "
                    "check camera yaw sign and frame freshness" %
                    route_point_number)
            elapsed = (
                capture_time - previous_capture_time
                if previous_capture_time is not None else None)
            self.rotate_for_pixel_error(
                error, observation_label, previous_error, elapsed)
            previous_error = error
            previous_capture_time = capture_time

        observation = {
            "route_point_number": int(route_point_number),
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "image_path": best_path,
            "attempt_image_paths": attempt_image_paths,
            "aligned": bool(aligned),
            "text": best_detection["text"] if best_detection else "",
            "confidence": (
                float(best_detection["confidence"])
                if best_detection else -1.0),
            "bbox": (
                list(best_detection["bbox"]) if best_detection else []),
        }
        if not aligned:
            rospy.logwarn(
                "PRODUCTION_OCR_NOT_ALIGNED point=%d text=%s",
                route_point_number,
                json.dumps(observation["text"], ensure_ascii=True))
            return observation

        self.wait_for_chassis_stop(
            observation_label + " before lidar")
        scan, distance = self.wait_for_fresh_front_distance()
        laser_pose = self.laser_map_pose(scan)
        if candidate_wall_points is None:
            candidate_wall_points = self.wall_reference_points
        ray_intersection = forward_ray_wall_intersection(
            laser_pose, candidate_wall_points)
        if ray_intersection is None:
            raise MissionAbort("forward lidar ray does not meet a wall boundary")
        ray_distance, hit = ray_intersection
        measured_distance = float(distance) + self.lidar_forward_offset
        range_residual = abs(measured_distance - ray_distance)
        match = nearest_numbered_point(hit, candidate_wall_points)
        if match is None:
            raise MissionAbort("grid has no wall reference candidates")
        wall_number, wall_coordinate, match_error = match
        observation.update({
            "front_distance_m": float(distance),
            "laser_pose_map": list(laser_pose),
            "forward_ray_wall_intersection_map": list(hit),
            "forward_ray_wall_distance_m": float(ray_distance),
            "range_residual_m": float(range_residual),
            "wall_point_number": int(wall_number),
            "wall_point_coordinate": list(wall_coordinate),
            "wall_match_error_m": float(match_error),
        })
        if range_residual > self.ray_range_agreement:
            rospy.logwarn(
                "PRODUCTION_WALL_RAY_REJECTED route_point=%d candidate=%d "
                "range_residual=%.3f limit=%.3f",
                route_point_number, wall_number, range_residual,
                self.ray_range_agreement)
            observation.pop("wall_point_number", None)
        else:
            rospy.loginfo(
                "PRODUCTION_WALL_RAY route_point=%d wall_point=%d "
                "text=%s distance=%.3f ray_distance=%.3f residual=%.3f",
                route_point_number, wall_number,
                json.dumps(observation["text"], ensure_ascii=True),
                distance, ray_distance, range_residual)
        return observation

    def wait_for_fresh_front_distance(self):
        baseline = rospy.Time.now()
        deadline = baseline + rospy.Duration(self.front_scan_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            with self.lock:
                scan = self.latest_scan
                receipt = self.latest_scan_receipt
            if scan is not None and receipt is not None and receipt > baseline:
                distance = front_scan_distance(
                    scan, self.front_scan_half_angle)
                if distance is not None:
                    return scan, distance
            rospy.sleep(0.02)
        raise MissionAbort(
            "no finite fresh front lidar range within %.1f s" %
            self.front_scan_timeout)

    def laser_map_pose(self, scan):
        frame = scan.header.frame_id
        if not frame:
            raise MissionAbort("front scan has no frame_id")
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                "map", frame, scan.header.stamp)
            yaw = tf.transformations.euler_from_quaternion(rotation)[2]
            return translation[0], translation[1], yaw
        except tf.Exception as exc:
            raise MissionAbort(
                "map pose for lidar frame %s at scan stamp unavailable: %s" %
                (frame, exc))

    def save_observation_summary(self):
        if self.run_directory is None:
            return
        payload = {
            "route": self.production_route_numbers,
            "target_legs": self.production_navigation_legs,
            "target_guard_points": dict(
                (str(number), sorted(points))
                for number, points in self.target_guard_points.items()),
            "target_guard_events": self.target_guard_events,
            "target_scan_events": self.target_scan_events,
            "observations": self.observations,
            "qr_classifications": self.qr_classifications,
            "recognized_categories": select_three_processing_observations(
                self.observations),
        }
        target = os.path.join(self.run_directory, "observations.json")
        temporary = target + ".tmp"
        with open(temporary, "w") as handle:
            json.dump(payload, handle, sort_keys=True, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.rename(temporary, target)

    def switch_to_point_mode(self):
        """Lock all task navigation legs to CymPlanner's front point mode."""
        self.publish_state("SET_POINT_NAVIGATION_MODE")
        deadline = (
            rospy.Time.now() + rospy.Duration(
                self.navigation_mode_connect_timeout))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            if self.navigation_mode_pub.get_num_connections() > 0:
                break
            rospy.sleep(0.1)
        if self.navigation_mode_pub.get_num_connections() <= 0:
            raise MissionAbort(
                "CymPlanner is not connected to /ucar/navigation_mode")
        # Repeat the latched command so the delivery is observable and robust
        # to a connection that completed at the edge of the wait loop.
        for _index in range(3):
            self.navigation_mode_pub.publish(
                String(data="point"))
            rospy.sleep(0.1)
        rospy.loginfo(
            "PRODUCTION_TASK navigation mode locked to point for all legs")

    def wait_for_safe_start(self):
        deadline = rospy.Time.now() + rospy.Duration(self.safe_start_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            reason = self.safety_failure_reason()
            if reason is None:
                rospy.loginfo(
                    "PRODUCTION_TASK safety gate passed with %d finite odom "
                    "samples and both TF chains available.",
                    self.consecutive_finite_odom)
                return
            rospy.logwarn_throttle(
                2.0, "PRODUCTION_TASK waiting for safety gate: %s" % reason)
            rospy.sleep(0.1)
        raise MissionAbort(
            "safety gate did not pass within %.1f s: %s" %
            (self.safe_start_timeout, self.safety_failure_reason()))

    def safety_failure_reason(self):
        with self.lock:
            critical_error = self.critical_error
            odom_receipt = self.latest_odom_receipt
            odom_finite = self.latest_odom_finite
            finite_count = self.consecutive_finite_odom
        if critical_error:
            return critical_error
        if odom_receipt is None:
            return "no /odom_raw received"
        if not odom_finite:
            return "/odom_raw is non-finite"
        odom_age = (rospy.Time.now() - odom_receipt).to_sec()
        if odom_age > self.odom_timeout:
            return "/odom_raw is stale by %.3f s" % odom_age
        if finite_count < self.minimum_finite_odom_samples:
            return "only %d/%d consecutive finite odom samples" % (
                finite_count, self.minimum_finite_odom_samples)
        for target, source in (
                ("odom", "base_link"), ("map", "base_link")):
            try:
                latest = self.tf_listener.getLatestCommonTime(target, source)
                if latest.is_zero():
                    return "%s -> %s TF has zero timestamp" % (target, source)
                age = (rospy.Time.now() - latest).to_sec()
                if age > self.tf_timeout:
                    return "%s -> %s TF is stale by %.3f s" % (
                        target, source, age)
                self.tf_listener.lookupTransform(
                    target, source, rospy.Time(0))
            except tf.Exception as exc:
                return "%s -> %s TF unavailable: %s" % (
                    target, source, exc)
        return None

    def require_safe(self):
        reason = self.safety_failure_reason()
        if reason is not None:
            raise MissionAbort(reason)

    @staticmethod
    def map_pose(x_value, y_value, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(x_value)
        pose.pose.position.y = float(y_value)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def current_map_pose(self, context):
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
            return (
                translation[0],
                translation[1],
                tf.transformations.euler_from_quaternion(rotation)[2],
            )
        except tf.Exception as exc:
            raise MissionAbort(
                "%s: map pose unavailable: %s" % (context, exc))

    def current_odom_yaw(self, context):
        try:
            _translation, rotation = self.tf_listener.lookupTransform(
                "odom", "base_link", rospy.Time(0))
            return tf.transformations.euler_from_quaternion(rotation)[2]
        except tf.Exception as exc:
            raise MissionAbort(
                "%s: odom yaw unavailable: %s" % (context, exc))

    def navigate_to(self, point_number, yaw, state):
        point = self.points[point_number]
        self.publish_state(state)
        self.navigate_coordinates(
            point[0], point[1], yaw, "point %d" % point_number,
            require_plan=True)

    def verify_position(self, point_number, context, warn_only=False):
        target = self.points[point_number]
        pose = self.current_map_pose(context)
        error = position_error(pose, target)
        if error > self.arrival_tolerance:
            if warn_only:
                rospy.logwarn(
                    "PRODUCTION_TASK_POSITION_WARNING context=%s point=%d "
                    "error=%.3f m limit=%.3f m; continuing mission",
                    context, point_number, error, self.arrival_tolerance)
                return False
            raise MissionAbort(
                "%s drifted %.3f m from point %d (limit %.3f m)" %
                (context, error, point_number, self.arrival_tolerance))
        rospy.loginfo(
            "PRODUCTION_TASK_POSITION_VERIFIED context=%s point=%d "
            "error=%.3f m", context, point_number, error)
        return True

    def recenter_after_turn(self, point_number, yaw):
        target = self.points[point_number]
        for attempt in range(1, self.post_turn_recenter_attempts + 1):
            pose = self.current_map_pose(
                "post-turn recenter check point %d" % point_number)
            error = position_error(pose, target)
            if not needs_recenter(
                    error, self.post_turn_recenter_trigger):
                rospy.loginfo(
                    "PRODUCTION_TASK_RECENTER_NOT_NEEDED point=%d "
                    "error=%.3f m trigger=%.3f m",
                    point_number, error, self.post_turn_recenter_trigger)
                return

            self.publish_state(
                "PRODUCTION_RECENTER_%d_%d" % (point_number, attempt))
            rospy.logwarn(
                "PRODUCTION_TASK_RECENTER_START point=%d attempt=%d/%d "
                "error=%.3f m trigger=%.3f m",
                point_number, attempt, self.post_turn_recenter_attempts,
                error, self.post_turn_recenter_trigger)
            corrected = self.navigate_coordinates(
                target[0], target[1], yaw,
                "post-turn recenter point %d attempt %d" %
                (point_number, attempt),
                require_plan=True,
                abort_on_navigation_failure=False)
            if not corrected:
                rospy.logwarn(
                    "PRODUCTION_TASK_RECENTER_WARNING point=%d "
                    "attempt=%d/%d failed; continuing recovery attempts",
                    point_number, attempt,
                    self.post_turn_recenter_attempts)

        pose = self.current_map_pose(
            "post-turn recenter final check point %d" % point_number)
        error = position_error(pose, target)
        if needs_recenter(error, self.post_turn_recenter_trigger):
            rospy.logwarn(
                "PRODUCTION_TASK_RECENTER_LIMIT point=%d error=%.3f m "
                "trigger=%.3f m; applying arrival limit %.3f m",
                point_number, error, self.post_turn_recenter_trigger,
                self.arrival_tolerance)

    def navigate_coordinates(
            self, x_value, y_value, yaw, label, require_plan,
            abort_on_navigation_failure=True,
            require_action_success=False, guard_callback=None):
        self.require_safe()
        if require_plan:
            plan_available = self.wait_for_plan(
                x_value, y_value, yaw, label,
                abort_on_failure=abort_on_navigation_failure)
            if not plan_available:
                return False

        goal = MoveBaseGoal()
        goal.target_pose = self.map_pose(x_value, y_value, yaw)
        rospy.loginfo(
            "PRODUCTION_TASK_GOAL label=%s target=(%.3f, %.3f) yaw=%.3f",
            label, x_value, y_value, yaw)
        self.move_base.send_goal(goal)
        deadline = rospy.Time.now() + rospy.Duration(self.goal_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            if guard_callback is not None and guard_callback():
                # Do not act from a subscriber callback: cancel, zero speed,
                # action acknowledgement and stopped-odom confirmation must
                # remain serialised in the navigation supervisor.
                self.cancel_navigation_for_observation(
                    label + " target guard")
                return False
            if self.move_base.wait_for_result(rospy.Duration(0.1)):
                break
        else:
            self.move_base.cancel_goal()
            self.stop_motion()
            if not abort_on_navigation_failure:
                rospy.logwarn(
                    "PRODUCTION_TASK_NAVIGATION_WARNING label=%s "
                    "timed out after %.1f s; continuing mission",
                    label, self.goal_timeout)
                return False
            raise MissionAbort(
                "%s timed out after %.1f s" % (label, self.goal_timeout))

        status = self.move_base.get_state()
        self.stop_motion()
        if status != GoalStatus.SUCCEEDED:
            if require_action_success:
                raise MissionAbort(
                    "%s requires move_base success but ended with status %d" %
                    (label, status))
            pose = self.current_map_pose(label + " aborted arrival")
            arrival_error = position_error(pose, (x_value, y_value))
            if arrival_error <= self.arrival_tolerance:
                rospy.logwarn(
                    "PRODUCTION_TASK_GOAL_ACCEPTED label=%s "
                    "move_base_status=%d arrival_error=%.3f m "
                    "limit=%.3f m",
                    label, status, arrival_error, self.arrival_tolerance)
                return True
            if not abort_on_navigation_failure:
                rospy.logwarn(
                    "PRODUCTION_TASK_NAVIGATION_WARNING label=%s "
                    "move_base status=%d; continuing mission",
                    label, status)
                return False
            raise MissionAbort(
                "%s failed with move_base status %d" % (label, status))
        pose = self.current_map_pose(label + " arrival")
        arrival_error = position_error(pose, (x_value, y_value))
        if arrival_error > self.arrival_tolerance:
            if not abort_on_navigation_failure:
                rospy.logwarn(
                    "PRODUCTION_TASK_NAVIGATION_WARNING label=%s "
                    "arrival_error=%.3f m limit=%.3f m; continuing mission",
                    label, arrival_error, self.arrival_tolerance)
                return False
            raise MissionAbort(
                "%s stopped %.3f m from target (limit %.3f m)" %
                (label, arrival_error, self.arrival_tolerance))
        rospy.loginfo(
            "PRODUCTION_TASK_GOAL_REACHED label=%s error=%.3f m",
            label, arrival_error)
        return True

    def wait_for_plan(
            self, x_value, y_value, yaw, label, abort_on_failure=True):
        deadline = rospy.Time.now() + rospy.Duration(self.plan_timeout)
        target = self.map_pose(x_value, y_value, yaw)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            try:
                rospy.wait_for_service(
                    "move_base/make_plan", timeout=1.0)
                current = self.current_map_pose(label + " plan")
                start = self.map_pose(current[0], current[1], current[2])
                response = self.make_plan(start, target, 0.0)
                if len(response.plan.poses) > 1:
                    rospy.loginfo(
                        "PRODUCTION_TASK_PLAN label=%s poses=%d",
                        label, len(response.plan.poses))
                    return True
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn_throttle(
                    2.0, "PRODUCTION_TASK plan wait for %s: %s" %
                    (label, exc))
            rospy.sleep(0.5)
        if not abort_on_failure:
            rospy.logwarn(
                "PRODUCTION_TASK_NAVIGATION_WARNING no global plan to %s "
                "within %.1f s; continuing mission",
                label, self.plan_timeout)
            return False
        raise MissionAbort(
            "no global plan to %s within %.1f s" %
            (label, self.plan_timeout))

    def rotate_full_revolution(
            self, label, speed, stop_for_qr, qr_baseline):
        self.move_base.cancel_all_goals()
        self.stop_motion()
        self.require_safe()
        direction = 1.0
        previous_yaw = self.current_odom_yaw(label)
        progress = 0.0
        target_progress = 2.0 * math.pi
        timeout = (
            target_progress / speed * self.rotation_timeout_scale + 2.0)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(self.rotation_control_rate)
        rospy.loginfo(
            "PRODUCTION_TASK_TURN_START label=%s speed=%.3f timeout=%.1f",
            label, speed, timeout)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            if stop_for_qr:
                detected = self.fresh_qr_after(qr_baseline)
                if detected is not None:
                    self.stop_motion()
                    rospy.loginfo(
                        "PRODUCTION_TASK_TURN_QR label=%s progress=%.3f "
                        "value=%s", label, progress, detected)
                    return detected

            current_yaw = self.current_odom_yaw(label)
            progress += positive_turn_increment(
                previous_yaw, current_yaw, direction)
            previous_yaw = current_yaw
            if progress >= (
                    target_progress - self.rotation_completion_tolerance):
                self.stop_motion()
                rospy.loginfo(
                    "PRODUCTION_TASK_TURN_COMPLETE label=%s progress=%.3f",
                    label, progress)
                return None

            command = Twist()
            command.angular.z = direction * speed
            self.cmd_vel_pub.publish(command)
            rate.sleep()

        self.stop_motion()
        raise MissionAbort(
            "%s did not complete a 360-degree turn within %.1f s" %
            (label, timeout))

    def stop_motion(self):
        command = Twist()
        for _index in range(6):
            self.cmd_vel_pub.publish(command)
            rospy.sleep(0.03)

    def stop_everything(self):
        try:
            self.move_base.cancel_all_goals()
        except Exception:
            pass
        try:
            self.qr_enable_pub.publish(Int8(data=0))
        except Exception:
            pass
        try:
            self.stop_motion()
        except Exception:
            pass
        try:
            self.stop_native_ocr()
        except Exception:
            pass
        try:
            self.stop_qr_classifier()
        except Exception:
            pass
        try:
            self.ensure_ros_camera_released()
        except Exception:
            pass

    def shutdown(self):
        self.stop_everything()


if __name__ == "__main__":
    rospy.init_node("production_task_2026")
    try:
        ProductionTask2026()
    except TaskDefinitionError as exc:
        rospy.logfatal("Invalid production task configuration: %s", exc)
        raise
    rospy.loginfo("2026 production task node started.")
    rospy.spin()
