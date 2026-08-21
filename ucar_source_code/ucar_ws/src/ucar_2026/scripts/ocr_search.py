#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""独立 OCR 搜索节点。

该节点只负责：

1. 把当前车辆位置按省赛约定视为 3 号点，并原地转向 13 号点；
2. 按独立调试路线逐点导航：3 → 428 → 429...436 → 445 → 444 → 437 → 419 → 427；
3. 到达每个点后按固定方位数（默认 8 个）逐一停靠扫描，每个方位停留约
   1 秒并抓帧调用 live_ppocr；
4. 点位被动态障碍占用时跳过；全路线没有识别结果时导航到 441；
5. 对 OCR 候选执行省赛同源的连续原地对齐，单点最多等待 15 秒；对齐失败
   继续扫描剩余方位；
6. 记录 OCR 候选结果。

它不处理二维码、激光测距、墙边停车、搬运、仿真联动或巡线交接。
原省赛主入口仍由 ``2026.launch`` 启动，本节点由 ``ocr_search.launch`` 单独启动。
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
from std_msgs.msg import String

from production_task_geometry import (
    bearing,
    is_finite,
    load_numbered_points,
    position_error,
    positive_turn_increment,
    require_points,
    shortest_yaw_delta,
)
from production_task_perception import (
    alignment_angular_speed,
    horizontal_pixel_error,
    is_navigation_ocr_candidate,
    normalize_production_category,
    ocr_detection_bbox_area,
    target_guard_scan_matches,
)


class OcrSearchError(RuntimeError):
    """A visible, task-level OCR search failure."""


class OcrAlignmentTimeout(OcrSearchError):
    """The current point's continuous OCR alignment budget expired."""


class OcrSearch(object):
    """Run only the navigation and full-turn OCR search portion."""

    DEFAULT_ROUTE = [
        428, 429, 430, 431, 432, 433, 434, 435, 436,
        445, 444, 437, 419, 427,
    ]
    DEFAULT_HEADINGS_DEG = [
        0, 0, 0, 0, 0, 0, 0, 0,
        -90, 180, 180, 90, 0, -153.4,
    ]

    def __init__(self):
        self.grid_path = str(rospy.get_param(
            "~grid_path",
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..", "config", "production_full_grid_all_numbered.json")))
        self.start_point_number = int(rospy.get_param(
            "~start_point_number", 3))
        self.start_heading_point_number = int(rospy.get_param(
            "~start_heading_point_number", 13))
        self.route_numbers = [int(value) for value in rospy.get_param(
            "~route_numbers", self.DEFAULT_ROUTE)]
        self.route_headings = [math.radians(float(value)) for value in
                               rospy.get_param(
                                   "~route_headings_deg",
                                   self.DEFAULT_HEADINGS_DEG)]
        if not self.route_numbers:
            raise OcrSearchError("route_numbers must not be empty")
        if len(self.route_numbers) != len(self.route_headings):
            raise OcrSearchError(
                "route_numbers and route_headings_deg have different lengths")

        self.arrival_tolerance = float(rospy.get_param(
            "~arrival_tolerance", 0.12))
        self.plan_timeout = float(rospy.get_param("~plan_timeout", 30.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 180.0))
        self.navigation_arrival_retry_attempts = int(rospy.get_param(
            "~navigation_arrival_retry_attempts", 3))
        self.move_base_ready_timeout = float(rospy.get_param(
            "~move_base_ready_timeout", 180.0))

        self.cmd_vel_topic = str(rospy.get_param(
            "~cmd_vel_topic", "/cmd_vel/navigation"))
        self.odom_timeout = float(rospy.get_param(
            "~odom_timeout", 3.0))
        self.stop_confirmation_timeout = float(rospy.get_param(
            "~stop_confirmation_timeout", 4.0))
        self.stopped_odom_speed_epsilon = float(rospy.get_param(
            "~stopped_odom_speed_epsilon", 0.02))
        self.stopped_odom_samples = int(rospy.get_param(
            "~stopped_odom_samples", 3))
        self.obstacle_scan_topic = str(rospy.get_param(
            "~obstacle_scan_topic", "/scan_global_obstacles"))
        self.obstacle_match_radius = float(rospy.get_param(
            "~obstacle_match_radius_m", 0.12))
        self.obstacle_confirmation_scans = int(rospy.get_param(
            "~obstacle_confirmation_scans", 2))
        self.obstacle_precheck_timeout = float(rospy.get_param(
            "~obstacle_precheck_timeout", 1.0))
        self.obstacle_scan_max_age = float(rospy.get_param(
            "~obstacle_scan_max_age", 1.0))
        self.tf_lookup_retry_seconds = float(rospy.get_param(
            "~tf_lookup_retry_seconds", 1.0))

        self.camera_image_topic = str(rospy.get_param(
            "~camera_image_topic", "/usb_cam/image_raw"))
        self.camera_width = int(rospy.get_param("~camera_width", 640))
        self.camera_height = int(rospy.get_param("~camera_height", 480))
        self.camera_warmup_frames = int(rospy.get_param(
            "~camera_warmup_frames", 8))
        self.camera_frame_timeout = float(rospy.get_param(
            "~camera_frame_timeout", 1.0))
        self.camera_device = str(rospy.get_param(
            "~camera_device", "/dev/ucar_camera"))
        self.camera_mirror = bool(rospy.get_param(
            "~camera_mirror", True))

        self.ocr_python = str(rospy.get_param(
            "~ocr_python", "/usr/bin/python3"))
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
        self.ocr_helper_ready_timeout = float(rospy.get_param(
            "~ocr_helper_ready_timeout", 90.0))
        self.ocr_capture_timeout = float(rospy.get_param(
            "~ocr_capture_timeout", 24.0))
        self.ocr_min_confidence = float(rospy.get_param(
            "~ocr_min_confidence", 0.30))
        self.ocr_scan_rotation_speed = abs(float(rospy.get_param(
            "~ocr_scan_rotation_speed", 0.70)))
        self.rotation_timeout_scale = float(rospy.get_param(
            "~rotation_timeout_scale", 3.5))
        self.ocr_scan_positions = int(rospy.get_param(
            "~ocr_scan_positions", 8))
        self.ocr_scan_dwell_seconds = float(rospy.get_param(
            "~ocr_scan_dwell_seconds", 1.0))
        self.ocr_scan_poll_period = float(rospy.get_param(
            "~ocr_scan_poll_period", 0.20))
        self.ocr_scan_candidate_confidence = float(rospy.get_param(
            "~ocr_scan_candidate_confidence", 60.0))
        self.ocr_candidate_min_bbox_area_px = float(rospy.get_param(
            "~ocr_candidate_min_bbox_area_px", 1000.0))
        self.ocr_alignment_tolerance_px = float(rospy.get_param(
            "~ocr_alignment_tolerance_px", 30.0))
        self.ocr_alignment_timeout = float(rospy.get_param(
            "~ocr_alignment_timeout", 15.0))
        self.ocr_alignment_kp = float(rospy.get_param(
            "~ocr_alignment_kp", 0.0025))
        self.ocr_alignment_kd = float(rospy.get_param(
            "~ocr_alignment_kd", 0.00035))
        self.ocr_alignment_max_speed = abs(float(rospy.get_param(
            "~ocr_alignment_max_speed", 0.22)))
        self.ocr_alignment_min_speed = abs(float(rospy.get_param(
            "~ocr_alignment_min_speed", 0.12)))
        self.rotation_control_rate = float(rospy.get_param(
            "~rotation_control_rate", 20.0))
        self.rotation_completion_tolerance = float(rospy.get_param(
            "~rotation_completion_tolerance_rad", 0.03))
        self.fixed_heading_rotation_speed = abs(float(rospy.get_param(
            "~fixed_heading_rotation_speed", 0.70)))
        self.fixed_heading_min_speed = abs(float(rospy.get_param(
            "~fixed_heading_min_speed", 0.12)))
        self.fixed_heading_timeout = float(rospy.get_param(
            "~fixed_heading_timeout", 8.0))
        self.fixed_heading_yaw_tolerance = float(rospy.get_param(
            "~fixed_heading_yaw_tolerance_rad", 0.01))
        self.destination_heading_point_number = int(rospy.get_param(
            "~destination_heading_point_number", 170))

        self.result_directory = os.path.expanduser(str(rospy.get_param(
            "~result_directory", "~/.ros/ucar_2026_ocr_search")))

        for name, value in (
                ("arrival_tolerance", self.arrival_tolerance),
                ("plan_timeout", self.plan_timeout),
                ("goal_timeout", self.goal_timeout),
                ("move_base_ready_timeout", self.move_base_ready_timeout),
                ("odom_timeout", self.odom_timeout),
                ("stop_confirmation_timeout", self.stop_confirmation_timeout),
                ("obstacle_match_radius", self.obstacle_match_radius),
                ("obstacle_precheck_timeout", self.obstacle_precheck_timeout),
                ("obstacle_scan_max_age", self.obstacle_scan_max_age),
                ("tf_lookup_retry_seconds", self.tf_lookup_retry_seconds),
                ("camera_frame_timeout", self.camera_frame_timeout),
                ("ocr_helper_ready_timeout", self.ocr_helper_ready_timeout),
                ("ocr_capture_timeout", self.ocr_capture_timeout),
                ("ocr_scan_rotation_speed", self.ocr_scan_rotation_speed),
                ("ocr_scan_dwell_seconds", self.ocr_scan_dwell_seconds),
                ("ocr_alignment_tolerance_px",
                 self.ocr_alignment_tolerance_px),
                ("ocr_alignment_timeout", self.ocr_alignment_timeout),
                ("ocr_alignment_kp", self.ocr_alignment_kp),
                ("ocr_alignment_max_speed", self.ocr_alignment_max_speed),
                ("ocr_alignment_min_speed", self.ocr_alignment_min_speed),
                ("rotation_timeout_scale", self.rotation_timeout_scale),
                ("ocr_scan_poll_period", self.ocr_scan_poll_period),
                ("rotation_control_rate", self.rotation_control_rate),
                ("fixed_heading_rotation_speed",
                 self.fixed_heading_rotation_speed),
                ("fixed_heading_timeout", self.fixed_heading_timeout)):
            if not is_finite(value) or value <= 0.0:
                raise OcrSearchError(
                    "%s must be finite and positive" % name)
        if self.navigation_arrival_retry_attempts < 0:
            raise OcrSearchError(
                "navigation_arrival_retry_attempts must be non-negative")
        if self.camera_warmup_frames <= 0:
            raise OcrSearchError("camera_warmup_frames must be positive")
        if self.ocr_scan_positions <= 0:
            raise OcrSearchError("ocr_scan_positions must be positive")
        if self.stopped_odom_samples <= 0:
            raise OcrSearchError("stopped_odom_samples must be positive")
        if self.obstacle_confirmation_scans <= 0:
            raise OcrSearchError("obstacle_confirmation_scans must be positive")
        if self.stopped_odom_speed_epsilon < 0.0:
            raise OcrSearchError(
                "stopped_odom_speed_epsilon must be non-negative")
        if self.ocr_scan_candidate_confidence < 0.0:
            raise OcrSearchError(
                "ocr_scan_candidate_confidence must be non-negative")
        if self.ocr_candidate_min_bbox_area_px <= 0.0:
            raise OcrSearchError(
                "ocr_candidate_min_bbox_area_px must be positive")
        if self.ocr_alignment_tolerance_px <= 0.0:
            raise OcrSearchError(
                "ocr_alignment_tolerance_px must be positive")
        if self.ocr_alignment_max_speed <= 0.0:
            raise OcrSearchError(
                "ocr_alignment_max_speed must be positive")
        if self.ocr_alignment_min_speed <= 0.0:
            raise OcrSearchError(
                "ocr_alignment_min_speed must be positive")
        if self.ocr_alignment_kp <= 0.0:
            raise OcrSearchError("ocr_alignment_kp must be positive")
        if (not is_finite(self.ocr_alignment_kd) or
                self.ocr_alignment_kd < 0.0):
            raise OcrSearchError("ocr_alignment_kd must be non-negative")
        if self.fixed_heading_min_speed <= 0.0:
            raise OcrSearchError("fixed_heading_min_speed must be positive")
        if self.fixed_heading_yaw_tolerance <= 0.0:
            raise OcrSearchError(
                "fixed_heading_yaw_tolerance_rad must be positive")

        all_required_points = (
            [self.start_point_number, self.start_heading_point_number,
             self.destination_heading_point_number, 441] +
            self.route_numbers)
        self.points = load_numbered_points(self.grid_path)
        require_points(self.points, all_required_points)

        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.make_plan = rospy.ServiceProxy("move_base/make_plan", GetPlan)
        self.tf_listener = tf.TransformListener()
        self.cv_bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=10)
        self.state_pub = rospy.Publisher(
            "/ucar_2026/ocr_search/state", String,
            queue_size=1, latch=True)
        self.result_pub = rospy.Publisher(
            "/ucar_2026/ocr_search/result", String,
            queue_size=1, latch=True)

        self.lock = threading.RLock()
        self.latest_odom_receipt = None
        self.latest_odom_velocity = None
        self.latest_odom_finite = False
        self.latest_camera_image = None
        self.latest_camera_receipt = None
        self.camera_sequence = 0
        self.latest_obstacle_scan = None
        self.latest_obstacle_scan_receipt = None
        self.obstacle_scan_sequence = 0
        self.critical_error = ""
        self.ocr_process = None
        self.ocr_log_handle = None
        self.capture_sequence = 0
        self.run_directory = None
        self.results = []
        self.skipped_points = []
        self.ocr_failed_points = []
        self.started = False

        rospy.Subscriber(
            "/odom_raw", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber(
            self.camera_image_topic, Image, self.camera_image_cb,
            queue_size=1)
        rospy.Subscriber(
            self.obstacle_scan_topic, LaserScan, self.obstacle_scan_cb,
            queue_size=5)
        rospy.Subscriber(
            "/rosout_agg", Log, self.rosout_cb, queue_size=100)
        rospy.on_shutdown(self.shutdown)
        self.publish_state("WAITING_START")

    def publish_state(self, state):
        self.state_pub.publish(String(data=str(state)))
        rospy.loginfo("OCR_SEARCH_STATE %s", state)

    def log_safe_text(self, value):
        return json.dumps(value, ensure_ascii=True)

    def odom_cb(self, message):
        values = [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        ]
        finite = all(is_finite(value) for value in values)
        with self.lock:
            self.latest_odom_receipt = rospy.Time.now()
            self.latest_odom_finite = finite
            self.latest_odom_velocity = (
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.angular.z,
            ) if finite else None
            if not finite:
                self.critical_error = "non-finite /odom_raw"

    def camera_image_cb(self, message):
        with self.lock:
            self.latest_camera_image = message
            self.latest_camera_receipt = rospy.Time.now()
            self.camera_sequence += 1

    def obstacle_scan_cb(self, message):
        with self.lock:
            self.latest_obstacle_scan = message
            self.latest_obstacle_scan_receipt = rospy.Time.now()
            self.obstacle_scan_sequence += 1

    def rosout_cb(self, message):
        text = message.msg.lower()
        if "crc16" in text and ("imu" in text or "ahrs" in text):
            rospy.logwarn_throttle(
                5.0, "OCR_SEARCH_IMU_CRC_WARNING %s", message.msg)
            return
        critical_markers = (
            "crc16",
            "head_len",
            "tf_nan_input",
            "odom sensor not active",
            "imu sensor not active",
        )
        if any(marker in text for marker in critical_markers):
            with self.lock:
                if not self.critical_error:
                    self.critical_error = message.msg

    def require_safe(self):
        with self.lock:
            error = self.critical_error
            receipt = self.latest_odom_receipt
            finite = self.latest_odom_finite
        if error:
            raise OcrSearchError(error)
        if receipt is None or not finite:
            raise OcrSearchError("/odom_raw is not ready with finite values")
        if (rospy.Time.now() - receipt).to_sec() > self.odom_timeout:
            raise OcrSearchError(
                "/odom_raw is stale by more than %.1f s" %
                self.odom_timeout)

    def current_map_pose(self, context):
        self.require_safe()
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
        except tf.Exception as exc:
            raise OcrSearchError(
                "%s: map pose unavailable: %s" % (context, exc))
        return (
            translation[0],
            translation[1],
            tf.transformations.euler_from_quaternion(rotation)[2],
        )

    def current_odom_yaw(self, context):
        self.require_safe()
        try:
            _translation, rotation = self.tf_listener.lookupTransform(
                "odom", "base_link", rospy.Time(0))
        except tf.Exception as exc:
            raise OcrSearchError(
                "%s: odom yaw unavailable: %s" % (context, exc))
        return tf.transformations.euler_from_quaternion(rotation)[2]

    def laser_map_pose(self, scan):
        frame = scan.header.frame_id
        if not frame:
            raise OcrSearchError("obstacle scan has no frame_id")
        deadline = time.time() + self.tf_lookup_retry_seconds
        while True:
            try:
                translation, rotation = self.tf_listener.lookupTransform(
                    "map", frame, scan.header.stamp)
                return (
                    translation[0],
                    translation[1],
                    tf.transformations.euler_from_quaternion(rotation)[2],
                )
            except tf.ExtrapolationException as exc:
                if time.time() >= deadline:
                    raise OcrSearchError(
                        "map pose for obstacle scan frame %s unavailable "
                        "after %.1f s: %s" %
                        (frame, self.tf_lookup_retry_seconds, exc))
                time.sleep(0.01)
            except tf.Exception as exc:
                raise OcrSearchError(
                    "map pose for obstacle scan frame %s unavailable: %s" %
                    (frame, exc))

    def map_pose(self, x_value, y_value, yaw):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x_value)
        pose.pose.position.y = float(y_value)
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def wait_for_chassis_stop(self, context):
        deadline = rospy.Time.now() + rospy.Duration(
            self.stop_confirmation_timeout)
        previous_receipt = None
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
                if velocity is not None and max(
                        abs(float(value)) for value in velocity) <= \
                        self.stopped_odom_speed_epsilon:
                    stopped_samples += 1
                    if stopped_samples >= self.stopped_odom_samples:
                        rospy.loginfo(
                            "OCR_SEARCH_STOP_CONFIRMED context=%s samples=%d",
                            context, stopped_samples)
                        return
                else:
                    stopped_samples = 0
            rospy.sleep(0.02)
        raise OcrSearchError(
            "%s did not confirm stopped odom within %.1f s" %
            (context, self.stop_confirmation_timeout))

    def stop_motion(self):
        zero = Twist()
        for _index in range(6):
            self.cmd_vel_pub.publish(zero)
            rospy.sleep(0.03)

    def wait_for_plan(self, x_value, y_value, yaw, label):
        deadline = rospy.Time.now() + rospy.Duration(self.plan_timeout)
        target = self.map_pose(x_value, y_value, yaw)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            rospy.wait_for_service("move_base/make_plan", timeout=1.0)
            current = self.current_map_pose(label + " plan")
            response = self.make_plan(
                self.map_pose(current[0], current[1], current[2]),
                target, 0.0)
            if len(response.plan.poses) > 1:
                rospy.loginfo(
                    "OCR_SEARCH_PLAN label=%s poses=%d",
                    label, len(response.plan.poses))
                return
            rospy.sleep(0.5)
        raise OcrSearchError(
            "no global plan to %s within %.1f s" %
            (label, self.plan_timeout))

    def point_is_blocked(self, point_number):
        """Require fresh dynamic-only lidar evidence before a point goal."""
        deadline = time.time() + self.obstacle_precheck_timeout
        baseline_sequence = self.obstacle_scan_sequence
        last_source_stamp = None
        hit_samples = 0
        clean_samples = 0
        usable_samples = 0
        target = {int(point_number): self.points[int(point_number)]}
        while not rospy.is_shutdown() and time.time() < deadline:
            self.require_safe()
            with self.lock:
                scan = self.latest_obstacle_scan
                receipt = self.latest_obstacle_scan_receipt
                sequence = self.obstacle_scan_sequence
            if (scan is None or receipt is None or
                    sequence <= baseline_sequence):
                rospy.sleep(0.02)
                continue

            now = rospy.Time.now()
            source_stamp = scan.header.stamp
            if source_stamp.is_zero():
                rospy.sleep(0.02)
                continue
            if (now - receipt).to_sec() > self.obstacle_scan_max_age:
                rospy.sleep(0.02)
                continue
            source_age = (now - source_stamp).to_sec()
            if (source_age > self.obstacle_scan_max_age or
                    source_age < -self.obstacle_scan_max_age):
                rospy.sleep(0.02)
                continue
            if (last_source_stamp is not None and
                    source_stamp <= last_source_stamp):
                rospy.sleep(0.02)
                continue
            last_source_stamp = source_stamp

            laser_pose = self.laser_map_pose(scan)
            matches = target_guard_scan_matches(
                scan, laser_pose, target, self.obstacle_match_radius)
            usable_samples += 1
            if int(point_number) in matches:
                hit_samples += 1
                clean_samples = 0
                rospy.loginfo(
                    "OCR_SEARCH_POINT_OBSTACLE point=%d error=%.3f "
                    "sample=%d/%d",
                    point_number, matches[int(point_number)], hit_samples,
                    self.obstacle_confirmation_scans)
                if hit_samples >= self.obstacle_confirmation_scans:
                    return True
            else:
                clean_samples += 1
                hit_samples = 0
                if clean_samples >= self.obstacle_confirmation_scans:
                    return False
            rospy.sleep(0.02)

        raise OcrSearchError(
            "no usable fresh obstacle scan for point %d within %.1f s "
            "(usable_samples=%d)" %
            (point_number, self.obstacle_precheck_timeout, usable_samples))

    def navigate_to_point(self, point_number, yaw, label):
        point = self.points[int(point_number)]
        self.wait_for_plan(point[0], point[1], yaw, label)
        for attempt in range(self.navigation_arrival_retry_attempts + 1):
            self.require_safe()
            goal = MoveBaseGoal()
            goal.target_pose = self.map_pose(point[0], point[1], yaw)
            rospy.loginfo(
                "OCR_SEARCH_GOAL label=%s point=%d target=(%.3f,%.3f) "
                "yaw=%.3f attempt=%d/%d",
                label, point_number, point[0], point[1], yaw,
                attempt + 1, self.navigation_arrival_retry_attempts + 1)
            self.move_base.send_goal(goal)
            deadline = rospy.Time.now() + rospy.Duration(self.goal_timeout)
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                self.require_safe()
                if self.move_base.wait_for_result(rospy.Duration(0.1)):
                    break
            else:
                self.move_base.cancel_goal()
                self.stop_motion()
                raise OcrSearchError(
                    "%s timed out after %.1f s" %
                    (label, self.goal_timeout))

            status = self.move_base.get_state()
            if status != GoalStatus.SUCCEEDED:
                self.stop_motion()
                raise OcrSearchError(
                    "%s failed with move_base status %d" %
                    (label, status))
            pose = self.current_map_pose(label + " arrival")
            error = position_error(pose, point)
            if error <= self.arrival_tolerance:
                rospy.loginfo(
                    "OCR_SEARCH_GOAL_REACHED label=%s point=%d error=%.3f",
                    label, point_number, error)
                return
            if attempt < self.navigation_arrival_retry_attempts:
                rospy.logwarn(
                    "OCR_SEARCH_ARRIVAL_RETRY label=%s point=%d error=%.3f "
                    "limit=%.3f",
                    label, point_number, error, self.arrival_tolerance)
                continue
            raise OcrSearchError(
                "%s stopped %.3f m from point %d (limit %.3f m)" %
                (label, error, point_number, self.arrival_tolerance))
        raise OcrSearchError("%s navigation loop ended unexpectedly" % label)

    def navigate_search_point(self, point_number, yaw, label):
        if self.point_is_blocked(point_number):
            self.skipped_points.append(int(point_number))
            self.publish_state("SKIP_BLOCKED_%03d" % point_number)
            rospy.logwarn(
                "OCR_SEARCH_POINT_SKIPPED point=%d reason=obstacle_before_goal",
                point_number)
            return False
        try:
            self.navigate_to_point(point_number, yaw, label)
        except OcrSearchError as exc:
            if self.point_is_blocked(point_number):
                self.skipped_points.append(int(point_number))
                self.publish_state("SKIP_BLOCKED_%03d" % point_number)
                rospy.logwarn(
                    "OCR_SEARCH_POINT_SKIPPED point=%d "
                    "reason=obstacle_after_navigation_failure error=%s",
                    point_number, exc)
                return False
            raise
        return True

    def prepare_start_pose(self):
        start = self.points[self.start_point_number]
        heading = self.points[self.start_heading_point_number]
        target_yaw = bearing(start, heading)
        pose = self.current_map_pose("OCR search assumed point 3")
        error = position_error(pose, start)
        if error > self.arrival_tolerance:
            raise OcrSearchError(
                "vehicle is %.3f m from assumed start point %d; "
                "OCR search requires the vehicle to start there" %
                (error, self.start_point_number))
        rospy.loginfo(
            "OCR_SEARCH_START_ASSUMED point=%d pose=(%.3f,%.3f) "
            "heading_point=%d target_yaw=%.3f",
            self.start_point_number, start[0], start[1],
            self.start_heading_point_number, target_yaw)
        self.rotate_in_place_to_yaw(
            target_yaw, "OCR search start heading point %d" %
            self.start_heading_point_number)

    def rotate_in_place_to_yaw(self, target_yaw, context):
        self.move_base.cancel_all_goals()
        self.stop_motion()
        self.wait_for_chassis_stop(context + " start")
        previous_yaw = self.current_odom_yaw(context + " start")
        delta = shortest_yaw_delta(previous_yaw, target_yaw)
        direction = 1.0 if delta >= 0.0 else -1.0
        required = max(
            0.0, abs(delta) - self.fixed_heading_yaw_tolerance)
        if required <= 0.0:
            return
        speed = self.fixed_heading_rotation_speed
        if speed < self.fixed_heading_min_speed:
            speed = self.fixed_heading_min_speed
        progress = 0.0
        deadline = rospy.Time.now() + rospy.Duration(
            self.fixed_heading_timeout)
        command = Twist()
        command.angular.z = direction * speed
        rate = rospy.Rate(self.rotation_control_rate)
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                self.require_safe()
                current_yaw = self.current_odom_yaw(context)
                progress += positive_turn_increment(
                    previous_yaw, current_yaw, direction)
                previous_yaw = current_yaw
                if progress >= required:
                    self.stop_motion()
                    self.wait_for_chassis_stop(context + " complete")
                    rospy.loginfo(
                        "OCR_SEARCH_START_HEADING_REACHED context=%s "
                        "requested=%.3f actual=%.3f",
                        context, abs(delta), progress)
                    return
                self.cmd_vel_pub.publish(command)
                rate.sleep()
        finally:
            self.stop_motion()
        raise OcrSearchError(
            "%s did not reach %.3f rad within %.1f s (actual=%.3f)" %
            (context, abs(delta), self.fixed_heading_timeout, progress))

    def prepare_result_directory(self):
        run_name = time.strftime("run_%Y%m%d_%H%M%S")
        self.run_directory = os.path.join(self.result_directory, run_name)
        os.makedirs(self.run_directory)
        rospy.loginfo("OCR_SEARCH_RESULT_DIRECTORY %s", self.run_directory)

    def read_ocr_message(self, timeout, context):
        deadline = time.time() + float(timeout)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.require_safe()
            if self.ocr_process is None:
                raise OcrSearchError("%s: OCR helper is not running" % context)
            if self.ocr_process.poll() is not None:
                raise OcrSearchError(
                    "%s: OCR helper exited with code %d" %
                    (context, self.ocr_process.returncode))
            readable, _writable, _errors = select.select(
                [self.ocr_process.stdout], [], [], 0.1)
            if not readable:
                continue
            raw_line = self.ocr_process.stdout.readline()
            if not raw_line:
                continue
            try:
                if not isinstance(raw_line, str):
                    raw_line = raw_line.decode("utf-8")
                return json.loads(raw_line)
            except (ValueError, UnicodeDecodeError):
                rospy.logwarn(
                    "OCR_SEARCH_NON_JSON %s", self.log_safe_text(raw_line))
        raise OcrSearchError(
            "%s timed out after %.1f s" % (context, timeout))

    def start_ocr(self):
        log_path = os.path.join(self.run_directory, "live_ppocr.log")
        command = [
            self.ocr_python,
            self.ocr_helper_path,
            "--ocr-module", self.live_ppocr_path,
            "--device", self.camera_device,
            "--det", os.path.join(self.ppocr_root, "out", "det.plan"),
            "--rec", os.path.join(self.ppocr_root, "out", "rec.plan"),
            "--keys", os.path.join(self.ppocr_root, "out", "keys.txt"),
            "--width", str(self.camera_width),
            "--height", str(self.camera_height),
            "--side", str(self.ocr_side),
            "--warmup-frames", str(self.camera_warmup_frames),
            "--open-timeout", "8.0",
            "--ros-image-input",
        ]
        if self.camera_mirror:
            command.append("--mirror")
        self.ocr_log_handle = open(log_path, "ab")
        self.ocr_process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self.ocr_log_handle,
            cwd=self.ppocr_root,
            bufsize=1)
        message = self.read_ocr_message(
            self.ocr_helper_ready_timeout, "live_ppocr startup")
        if not message.get("ready"):
            raise OcrSearchError(
                "live_ppocr helper did not report ready: %s" % message)
        rospy.loginfo(
            "OCR_SEARCH_OCR_READY mode=%s cv2=%s candidates=%s",
            message.get("mode"), message.get("cv2_version"),
            self.log_safe_text(message.get("candidates")))

    def wait_for_fresh_camera_frame(self, baseline_sequence, context):
        deadline = time.time() + self.camera_frame_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            self.require_safe()
            with self.lock:
                sequence = self.camera_sequence
                receipt = self.latest_camera_receipt
            if (sequence > baseline_sequence and receipt is not None and
                    (rospy.Time.now() - receipt).to_sec() <=
                    self.camera_frame_timeout):
                return sequence
            rospy.sleep(0.02)
        raise OcrSearchError(
            "%s did not receive a fresh camera frame" % context)

    def capture_ocr(self, capture_label, attempt):
        if self.ocr_process is None or self.ocr_process.poll() is not None:
            raise OcrSearchError("live_ppocr helper is not running")
        with self.lock:
            baseline_sequence = self.camera_sequence
        self.wait_for_fresh_camera_frame(baseline_sequence, capture_label)
        with self.lock:
            message = self.latest_camera_image
        if message is None:
            raise OcrSearchError("camera frame disappeared at %s" % capture_label)
        self.capture_sequence += 1
        image_path = os.path.join(
            self.run_directory,
            "capture_%05d_%s_attempt_%02d.png" % (
                self.capture_sequence,
                "".join(character if character.isalnum() else "_"
                        for character in str(capture_label)),
                attempt))
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            raise OcrSearchError(
                "cannot convert ROS camera frame: %s" % exc)
        if not cv2.imwrite(image_path, frame):
            raise OcrSearchError("cannot save camera frame %s" % image_path)
        payload = {
            "command": "capture",
            "input": image_path,
            "output": image_path,
            "minimum_confidence": self.ocr_min_confidence,
        }
        try:
            self.ocr_process.stdin.write(
                (json.dumps(payload) + "\n").encode("utf-8"))
            self.ocr_process.stdin.flush()
        except (IOError, OSError) as exc:
            raise OcrSearchError(
                "cannot command live_ppocr helper: %s" % exc)
        response = self.read_ocr_message(
            self.ocr_capture_timeout, "OCR capture %s" % capture_label)
        if not response.get("ok"):
            raise OcrSearchError(
                "live_ppocr capture failed for %s: %s" %
                (capture_label, response.get("error", response)))
        return response

    def start_async_capture(self, capture_label, attempt=1):
        task = {
            "done": threading.Event(),
            "response": None,
            "error": None,
        }

        def worker():
            try:
                task["response"] = self.capture_ocr(capture_label, attempt)
            except Exception as exc:
                task["error"] = exc
            finally:
                task["done"].set()

        task["thread"] = threading.Thread(target=worker)
        task["thread"].daemon = True
        task["thread"].start()
        return task

    def finish_async_capture(self, task, keep_stopped=False):
        deadline = time.time() + self.ocr_capture_timeout + 2.0
        zero = Twist()
        while not task["done"].wait(0.02):
            self.require_safe()
            if keep_stopped:
                self.cmd_vel_pub.publish(zero)
            if time.time() >= deadline:
                raise OcrSearchError(
                    "asynchronous OCR capture did not finish")
        task["thread"].join()
        if task["error"] is not None:
            raise task["error"]
        return task["response"]

    def capture_ocr_while_turning(
            self, signed_speed, capture_label, attempt, deadline=None):
        """Capture one fresh OCR frame while continuously turning in place."""
        speed = float(signed_speed)
        if speed == 0.0:
            raise OcrSearchError(
                "continuous OCR alignment has zero turn speed")
        direction = 1.0 if speed > 0.0 else -1.0
        if abs(speed) < self.ocr_alignment_min_speed:
            speed = self.ocr_alignment_min_speed * direction
        command = Twist()
        command.angular.z = speed
        task = None
        completed = False
        try:
            self.require_safe()
            self.cmd_vel_pub.publish(command)
            task = self.start_async_capture(
                "%s_moving" % capture_label, attempt)
            capture_deadline = time.time() + self.ocr_capture_timeout + 2.0
            if deadline is not None:
                capture_deadline = min(capture_deadline, deadline)
            rate = rospy.Rate(self.rotation_control_rate)
            while not task["done"].is_set():
                self.require_safe()
                if time.time() >= capture_deadline:
                    raise OcrAlignmentTimeout(
                        "continuous OCR alignment budget expired at %s" %
                        capture_label)
                self.cmd_vel_pub.publish(command)
                rate.sleep()
            response = self.finish_async_capture(task)
            completed = True
            return response
        finally:
            if not completed:
                self.stop_motion()
                self.cleanup_async_capture(task)

    def cleanup_async_capture(self, task):
        if task is None:
            return
        task["done"].wait(self.ocr_capture_timeout + 2.0)
        task["thread"].join()

    def log_small_candidate(self, response, label):
        detection = response.get("detection") if isinstance(response, dict) else None
        if not isinstance(detection, dict):
            return
        area = ocr_detection_bbox_area(detection)
        text = detection.get("text", "")
        confidence = float(detection.get("confidence", -1.0))
        if text and confidence >= self.ocr_scan_candidate_confidence and \
                area < self.ocr_candidate_min_bbox_area_px:
            rospy.loginfo(
                "OCR_SEARCH_CANDIDATE_IGNORED_SMALL label=%s text=%s "
                "confidence=%.1f area_px=%.1f threshold_px=%.1f",
                label, self.log_safe_text(text), confidence, area,
                self.ocr_candidate_min_bbox_area_px)

    def record_candidate(self, point_number, progress, response):
        detection = response["detection"]
        text = detection.get("text", "")
        category = normalize_production_category(text)
        result = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "point_number": int(point_number),
            "turn_progress_radians": float(progress),
            "text": text,
            "category": category,
            "confidence": float(detection["confidence"]),
            "bbox": detection.get("bbox"),
            "image_path": response.get("image_path"),
            "detection": detection,
        }
        self.results.append(result)
        self.save_results()
        rospy.loginfo(
            "OCR_SEARCH_CANDIDATE point=%d progress=%.3f text=%s "
            "category=%s confidence=%.1f image=%s",
            point_number, progress, self.log_safe_text(text),
            self.log_safe_text(category), detection["confidence"],
            response.get("image_path"))

    def align_ocr_candidate(
            self, point_number, observation_label, initial_response):
        """Align one point's OCR candidate, then return its final frame.

        The candidate frame is the first alignment observation.  Every later
        frame is captured while the chassis keeps turning at the bounded PD
        angular speed.  The point has a wall-clock budget, not a frame-count
        budget, and the 30px tolerance stays fixed.
        """
        self.stop_motion()
        self.wait_for_chassis_stop(observation_label + " alignment start")
        previous_error = None
        previous_capture_time = None
        alignment_speed = None
        divergence_count = 0
        response = initial_response
        attempt = 0
        deadline = time.time() + self.ocr_alignment_timeout

        while not rospy.is_shutdown() and time.time() < deadline:
            self.require_safe()
            attempt += 1
            capture_time = time.time()
            detection = response.get("detection")
            if detection is None:
                rospy.logwarn(
                    "OCR_SEARCH_OCR_EMPTY point=%d elapsed=%.1f/%.1f",
                    point_number, capture_time - (deadline -
                                                  self.ocr_alignment_timeout),
                    self.ocr_alignment_timeout)
                try:
                    response = self.capture_ocr_while_turning(
                        alignment_speed, observation_label, attempt + 1,
                        deadline=deadline)
                except OcrAlignmentTimeout:
                    break
                continue

            image_width = int(response["width"])
            error = horizontal_pixel_error(detection, image_width)
            rospy.loginfo(
                "OCR_SEARCH_OCR_BOX point=%d attempt=%d text=%s "
                "confidence=%.1f horizontal_error_px=%.1f "
                "tolerance_px=%.1f",
                point_number, attempt,
                self.log_safe_text(detection["text"]),
                detection["confidence"], error,
                self.ocr_alignment_tolerance_px)
            if abs(error) <= self.ocr_alignment_tolerance_px:
                self.stop_motion()
                self.wait_for_chassis_stop(
                    observation_label + " OCR aligned")
                rospy.loginfo(
                    "OCR_SEARCH_OCR_ALIGNED point=%d elapsed=%.1f/%.1f",
                    point_number, time.time() -
                    (deadline - self.ocr_alignment_timeout),
                    self.ocr_alignment_timeout)
                return response

            if (previous_error is not None and
                    abs(error) > abs(previous_error) * 1.35):
                divergence_count += 1
            else:
                divergence_count = 0
            if divergence_count >= 2:
                rospy.logwarn(
                    "OCR_SEARCH_OCR_ALIGNMENT_DIVERGED point=%d "
                    "attempt=%d; reset PD derivative",
                    point_number, attempt)
                previous_error = None
                previous_capture_time = None
                divergence_count = 0

            elapsed = (
                capture_time - previous_capture_time
                if previous_capture_time is not None else None)
            alignment_speed = alignment_angular_speed(
                error, self.ocr_alignment_kp, self.ocr_alignment_kd,
                self.ocr_alignment_max_speed, self.camera_mirror,
                previous_error, elapsed)
            if abs(alignment_speed) < self.ocr_alignment_min_speed:
                alignment_speed = self.ocr_alignment_min_speed * (
                    1.0 if alignment_speed >= 0.0 else -1.0)
            rospy.loginfo(
                "OCR_SEARCH_OCR_ALIGNMENT_CONTINUOUS point=%d "
                "attempt=%d speed=%.3f error_px=%.1f elapsed=%.1f/%.1f",
                point_number, attempt, alignment_speed, error,
                time.time() - (deadline - self.ocr_alignment_timeout),
                self.ocr_alignment_timeout)
            previous_error = error
            previous_capture_time = capture_time
            try:
                response = self.capture_ocr_while_turning(
                    alignment_speed, observation_label, attempt + 1,
                    deadline=deadline)
            except OcrAlignmentTimeout:
                break

        self.stop_motion()
        self.wait_for_chassis_stop(observation_label + " OCR timeout")
        rospy.logwarn(
            "OCR_SEARCH_OCR_NOT_ALIGNED point=%d timeout=%.1f; skip point",
            point_number, self.ocr_alignment_timeout)
        return None

    def finish_candidate(self, point_number, progress, label, response):
        """Continuously align the moving-scan candidate within the time budget."""
        aligned_response = self.align_ocr_candidate(
            point_number, label + " alignment", response)
        if aligned_response is None:
            self.ocr_failed_points.append(int(point_number))
            self.save_results()
            self.publish_state("SKIP_OCR_ALIGN_%03d" % point_number)
            return None
        self.record_candidate(point_number, progress, aligned_response)
        return aligned_response

    def scan_point(self, point_number):
        """Step-and-settle scan: turn to each of a fixed number of headings,
        dwell briefly while capturing OCR, then continue to the next heading.

        The scan keeps turning in one direction and uses the accumulated
        actual turn progress (positive_turn_increment) to decide when a
        heading is reached.  A strong candidate triggers continuous
        alignment; an alignment failure continues with the remaining
        headings instead of abandoning the point.
        """
        label = "OCR_SEARCH_TURN_%03d" % point_number
        self.publish_state(label)
        self.move_base.cancel_all_goals()
        self.stop_motion()
        self.wait_for_chassis_stop(label + " start")
        progress = 0.0
        angle_step = 2.0 * math.pi / self.ocr_scan_positions
        timeout = (2.0 * math.pi / self.ocr_scan_rotation_speed *
                   self.rotation_timeout_scale +
                   self.ocr_scan_positions * self.ocr_scan_dwell_seconds + 2.0)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(self.rotation_control_rate)
        rospy.loginfo(
            "OCR_SEARCH_TURN_START point=%d positions=%d speed=%.3f "
            "dwell=%.1f timeout=%.1f",
            point_number, self.ocr_scan_positions,
            self.ocr_scan_rotation_speed, self.ocr_scan_dwell_seconds,
            timeout)
        try:
            for position_index in range(1, self.ocr_scan_positions + 1):
                turn_target = position_index * angle_step
                previous_yaw = self.current_odom_yaw(
                    label + " position %d start" % position_index)
                while (not rospy.is_shutdown() and
                        rospy.Time.now() < deadline):
                    self.require_safe()
                    current_yaw = self.current_odom_yaw(label)
                    progress += positive_turn_increment(
                        previous_yaw, current_yaw, 1.0)
                    previous_yaw = current_yaw
                    if progress >= turn_target - \
                            self.rotation_completion_tolerance:
                        break
                    command = Twist()
                    command.angular.z = self.ocr_scan_rotation_speed
                    self.cmd_vel_pub.publish(command)
                    rate.sleep()
                if rospy.Time.now() >= deadline:
                    raise OcrSearchError(
                        "%s timed out at position %d/%d after %.1f s" %
                        (label, position_index, self.ocr_scan_positions,
                         timeout))
                self.stop_motion()
                self.wait_for_chassis_stop(
                    label + " position %d" % position_index)
                dwell_start = time.time()
                response = self.capture_ocr(
                    "%s_pos_%02d" % (label, position_index),
                    position_index)
                remaining = self.ocr_scan_dwell_seconds - (
                    time.time() - dwell_start)
                if remaining > 0.0:
                    rospy.sleep(remaining)
                self.log_small_candidate(response, label)
                if is_navigation_ocr_candidate(
                        response,
                        self.ocr_scan_candidate_confidence,
                        self.ocr_candidate_min_bbox_area_px):
                    aligned_response = self.finish_candidate(
                        point_number, progress, label, response)
                    if aligned_response is not None:
                        return aligned_response
                    self.publish_state(
                        "SCAN_CONTINUE_AFTER_ALIGN_FAIL_%03d" % point_number)
                    rospy.logwarn(
                        "OCR_SEARCH_TURN_ALIGN_FAILED point=%d "
                        "position=%d/%d; continuing remaining headings",
                        point_number, position_index, self.ocr_scan_positions)
            rospy.loginfo(
                "OCR_SEARCH_TURN_COMPLETE point=%d positions=%d progress=%.3f",
                point_number, self.ocr_scan_positions, progress)
            return None
        finally:
            self.stop_motion()
        raise OcrSearchError(
            "%s did not complete %d positions within %.1f s" %
            (label, self.ocr_scan_positions, timeout))

    def save_results(self):
        target = os.path.join(self.run_directory, "ocr_search_results.json")
        payload = {
            "start_point_number": self.start_point_number,
            "start_heading_point_number": self.start_heading_point_number,
            "route_numbers": self.route_numbers,
            "skipped_points": self.skipped_points,
            "ocr_failed_points": self.ocr_failed_points,
            "results": self.results,
        }
        with open(target, "w") as handle:
            json.dump(payload, handle, ensure_ascii=True, indent=2,
                      sort_keys=True)

    def publish_result(self, success, reason):
        payload = {
            "success": bool(success),
            "reason": str(reason),
            "start_point_number": self.start_point_number,
            "start_heading_point_number": self.start_heading_point_number,
            "route_numbers": self.route_numbers,
            "skipped_points": self.skipped_points,
            "ocr_failed_points": self.ocr_failed_points,
            "results": self.results,
            "result_file": (
                os.path.join(self.run_directory, "ocr_search_results.json")
                if self.run_directory else ""),
        }
        encoded = json.dumps(payload, ensure_ascii=True, sort_keys=True)
        self.result_pub.publish(String(data=encoded))
        rospy.loginfo("OCR_SEARCH_RESULT %s", encoded)

    def run(self):
        self.started = True
        self.prepare_result_directory()
        self.publish_state("WAIT_FOR_MOVE_BASE")
        if not self.move_base.wait_for_server(
                rospy.Duration(self.move_base_ready_timeout)):
            raise OcrSearchError(
                "move_base action server did not become ready within %.1f s" %
                self.move_base_ready_timeout)
        self.prepare_start_pose()
        self.publish_state("OCR_HELPER_START")
        try:
            self.start_ocr()
            for index, point_number in enumerate(self.route_numbers):
                heading = self.route_headings[index]
                label = "OCR search route %d/%d point %d" % (
                    index + 1, len(self.route_numbers), point_number)
                self.publish_state("NAVIGATE_%03d" % point_number)
                if not self.navigate_search_point(point_number, heading, label):
                    continue
                self.scan_point(point_number)
            if not self.results:
                destination = self.points[441]
                destination_heading = self.points[
                    self.destination_heading_point_number]
                self.publish_state("NAVIGATE_441_NO_OCR")
                self.navigate_to_point(
                    441, bearing(destination, destination_heading),
                    "OCR search no-result destination 441")
            self.stop_motion()
            self.wait_for_chassis_stop("OCR search complete")
            self.save_results()
            self.publish_state("SUCCEEDED")
            self.publish_result(
                True,
                "route exhausted with OCR" if self.results else
                "route exhausted without OCR; sent to 441")
        finally:
            self.stop_ocr()

    def stop_ocr(self):
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
        if process is not None:
            rospy.loginfo("OCR_SEARCH_OCR_CLOSED")

    def shutdown(self):
        self.move_base.cancel_all_goals()
        self.stop_motion()
        self.stop_ocr()


def main():
    rospy.init_node("ocr_search")
    task = OcrSearch()
    try:
        task.run()
    except OcrSearchError as exc:
        task.stop_motion()
        task.publish_state("ABORTED")
        task.publish_result(False, str(exc))
        rospy.logfatal("OCR_SEARCH_ABORTED %s", exc)
        raise


if __name__ == "__main__":
    main()
