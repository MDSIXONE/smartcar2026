#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Fail-safe ROS state machine for the requested 2026 production mission.

Flow:
  1. Navigate to centre 52.
  2. From 52, face QR observation points 262, 232, 295, 61, 41, and 43
     in order (180°, 90°, -90°, -135°, 135°, 45°).
     If a fresh QR is not decoded while facing a point, turn slowly for at
     most one complete revolution while scanning.
  3. After both QR items are classified, announce their collection before
     moving to point 3; then disable QR decoding and start the Python 3 OCR
     helper.  In the default configuration the helper consumes frames saved
     from the ROS usb_cam topic.
  4. Lock normal legs to CymPlanner's front-lookahead point mode; use the
     tighter destination profile only for the final 441 approach.
  5. Navigate the grouped production targets.  The forward pass tries one
     reachable point per group; after the last group, it reverse-completes
     points that were not tried.  After arrival, turn at most one full
     revolution while querying OCR.  A candidate stops the turn, passes a
     fresh-odometry stop gate, then aligns the box and reads the front lidar
     distance.
  6. If the primary route misses a requested category, scan the configured
     fallback perimeter route once.  If a required category remains absent,
     release OCR and continue directly to the 441 handoff destination.
  7. Save every attempt plus the three strongest distinct wall observations.
  8. On SUCCEEDED, activate the already-resident lane follower and switch
     the single chassis-command owner.  The shared ROS camera and chassis
     driver stay alive, eliminating the old launch restart pause.

The node never drives to the QR edge points: they lie on the field boundary.
They are gaze targets while the chassis remains at centre 52.
"""

from __future__ import print_function

import json
import math
import os
import select
import subprocess
import sys
import threading
import time
import httplib
import urllib2
from collections import deque

import actionlib
import cv2
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int8, String
from std_srvs.srv import Empty, SetBool

from production_task_geometry import (
    DEFAULT_QR_OBSERVATION_NUMBERS,
    DEFAULT_FALLBACK_PRODUCTION_OBSERVATION_HEADINGS_DEG,
    DEFAULT_FALLBACK_PRODUCTION_ROUTE,
    DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG,
    DEFAULT_PRODUCTION_ROUTE,
    DEFAULT_PRODUCTION_ROUTE_GROUPS,
    TaskDefinitionError,
    bearing,
    is_finite,
    load_numbered_points,
    load_middle_target_guard_points,
    load_middle_zone_geometry,
    load_wall_reference_points,
    needs_recenter,
    normalize_angle,
    normalize_production_route_groups,
    position_error,
    positive_turn_increment,
    require_points,
    shortest_yaw_delta,
    stop_point_for_wall_point,
)
from production_task_perception import (
    alignment_angular_speed,
    front_scan_distance,
    forward_ray_wall_intersection,
    horizontal_pixel_error,
    is_navigation_ocr_candidate,
    nearest_numbered_point,
    normalize_production_category,
    ocr_detection_bbox_area,
    odom_velocity_is_stopped,
    select_three_processing_observations,
    target_guard_scan_matches,
)


class MissionAbort(RuntimeError):
    pass


# Warehouse names announced after QR classification.  The wording matches the
# OCR workshop signs on the field; the electronics sign reads
# "电子产品生产车间" (production, not 加工).
PRODUCTION_WAREHOUSE_NAMES = {
    u"食品": u"食品加工车间",
    u"日用品": u"日用品加工车间",
    u"电子产品": u"电子产品生产车间",
}

# The microphone listener emits one of these categories for each slot in its
# fixed sentence.  The QR scanner still supplies the actual item name, which
# is retained for QR matching, announcements, result records and simulation.
VOICE_REQUEST_CATEGORIES = frozenset(PRODUCTION_WAREHOUSE_NAMES.keys())


class ProductionTask2026(object):
    def __init__(self):
        self.grid_path = rospy.get_param("~grid_path")
        self.staging_point_number = int(
            rospy.get_param("~staging_point_number", 52))
        self.sprint_enabled = bool(
            rospy.get_param("~sprint_enabled", False))
        self.sprint_start_point_number = int(
            rospy.get_param("~sprint_start_point_number", 70))
        self.sprint_end_point_number = int(
            rospy.get_param("~sprint_end_point_number", 288))
        self.sprint_end_x = rospy.get_param("~sprint_end_x", "")
        self.sprint_end_y = rospy.get_param("~sprint_end_y", "")
        self.sprint_arrival_tolerance = float(
            rospy.get_param("~sprint_arrival_tolerance", 0.30))
        self.sprint_end_xy = None
        if (str(self.sprint_end_x).strip() and
                str(self.sprint_end_y).strip()):
            self.sprint_end_xy = (
                float(self.sprint_end_x), float(self.sprint_end_y))
        # 冲刺段朝向（度）：起点→70 的到达朝向与 70→冲刺终点的运动方向。
        # 实车 2026-08-16 反馈 180° 偏一点，改 175 微调。
        self.sprint_yaw_deg = float(
            rospy.get_param("~sprint_yaw_deg", 180.0))
        # 冲刺段横向平移实验：true 时切换 CymPlanner transverse 模式
        # （车头保持 90°，linear.y 横向平移过坡），false 走原前进冲刺。
        self.sprint_transverse_enabled = bool(
            rospy.get_param("~sprint_transverse_enabled", False))
        self.qr_observation_numbers = [
            int(value) for value in
            rospy.get_param(
                "~qr_observation_numbers", DEFAULT_QR_OBSERVATION_NUMBERS)
        ]
        self.production_route_numbers = [
            int(value) for value in
            rospy.get_param(
                "~production_route_numbers",
                DEFAULT_PRODUCTION_ROUTE)
        ]
        self.production_route_groups = normalize_production_route_groups(
            rospy.get_param(
                "~production_route_groups", DEFAULT_PRODUCTION_ROUTE_GROUPS),
            self.production_route_numbers)
        self.production_observation_headings = [
            math.radians(float(value)) for value in
            rospy.get_param(
                "~production_observation_headings_deg",
                DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG)
        ]
        self.fallback_production_route_numbers = [
            int(value) for value in
            rospy.get_param(
                "~fallback_production_route_numbers",
                DEFAULT_FALLBACK_PRODUCTION_ROUTE)
        ]
        self.fallback_production_observation_headings = [
            math.radians(float(value)) for value in
            rospy.get_param(
                "~fallback_production_observation_headings_deg",
                DEFAULT_FALLBACK_PRODUCTION_OBSERVATION_HEADINGS_DEG)
        ]
        self.destination_point_number = int(
            rospy.get_param("~destination_point_number", 170))
        self.destination_heading_point_number = int(
            rospy.get_param("~destination_heading_point_number", 319))
        self.destination_midpoint_point_numbers = [
            int(value) for value in
            rospy.get_param(
                "~destination_midpoint_point_numbers", "").split(",")
            if value.strip()
        ]
        self.processing_dwell_seconds = float(rospy.get_param(
            "~processing_dwell_seconds", 3.0))
        # The simulation HTTP server is on the PC.  It is deliberately
        # independent of ROS_MASTER_URI: the ROS Master runs on the vehicle.
        self.simulation_port = int(rospy.get_param(
            "~simulation_port", 11313))
        self.simulation_host = str(
            rospy.get_param("~simulation_host", ""))
        self.simulation_start_timeout = float(rospy.get_param(
            "~simulation_start_timeout", 30.0))
        self.simulation_start_retries = max(
            1, int(rospy.get_param("~simulation_start_retries", 3)))
        self.simulation_done_timeout = float(rospy.get_param(
            "~simulation_done_timeout", 75.0))
        self.simulation_poll_period = float(rospy.get_param(
            "~simulation_poll_period", 2.0))
        self.speak_wait_timeout = float(rospy.get_param(
            "~speak_wait_timeout", 60.0))
        # lane_proto is resident from launch time.  At the destination this
        # node only activates it and switches the sole /cmd_vel owner; it
        # never tears down and restarts another launch.
        self.lane_handoff_enabled = bool(
            rospy.get_param("~lane_handoff_enabled", True))
        self.lane_activate_service = str(rospy.get_param(
            "~lane_activate_service", "/lane_proto/set_active"))
        self.lane_owner_service = str(rospy.get_param(
            "~lane_owner_service", "/cmd_vel_owner/set_lane_mode"))
        self.lane_state_topic = str(rospy.get_param(
            "~lane_state_topic", "/lane_proto/state"))
        self.lane_result_topic = str(rospy.get_param(
            "~lane_result_topic", "/lane_proto/result"))
        self.lane_handoff_timeout = float(rospy.get_param(
            "~lane_handoff_timeout", 360.0))
        self.tf_lookup_retry_seconds = float(rospy.get_param(
            "~tf_lookup_retry_seconds", 0.5))
        self.tts_enabled = bool(
            rospy.get_param("~tts_enabled", True))
        self.tts_python = str(rospy.get_param(
            "~tts_python", "/usr/bin/python3"))
        self.tts_helper_path = str(rospy.get_param(
            "~tts_helper_path", "/home/ucar/wake/tts_say.py"))
        # Voice input is intentionally category based: wake_listen.py returns
        # a JSON object with the two requested production categories.  The QR
        # phase resolves those categories to the actual field item names.
        self.item_input_mode = str(rospy.get_param(
            "~item_input_mode", "voice")).strip().lower()
        if self.item_input_mode not in ("voice", "stdin"):
            raise TaskDefinitionError(
                "item_input_mode must be voice or stdin, got %s" %
                self.item_input_mode)
        self.voice_listener_python = str(rospy.get_param(
            "~voice_listener_python", "/usr/bin/python3"))
        self.voice_listener_path = str(rospy.get_param(
            "~voice_listener_path", "/home/ucar/wake/micarray/wake_listen.py"))
        self.voice_wake_word = rospy.get_param(
            "~voice_wake_word", u"小飞小飞")
        self.voice_input_timeout = float(rospy.get_param(
            "~voice_input_timeout", 0.0))
        self.post_qr_waypoint_number = int(
            rospy.get_param("~post_qr_waypoint_number", 3))
        self.post_qr_waypoint_heading_point_number = int(
            rospy.get_param("~post_qr_waypoint_heading_point_number", 0))
        self.local_costmap_layer_control_enabled = bool(rospy.get_param(
            "~local_costmap_layer_control_enabled", True))
        self.local_costmap_enable_waypoint_number = int(rospy.get_param(
            "~local_costmap_enable_waypoint_number", 3))
        self.local_costmap_reconfigure_timeout = float(rospy.get_param(
            "~local_costmap_reconfigure_timeout", 5.0))
        self.local_costmap_obstacle_layer = str(rospy.get_param(
            "~local_costmap_obstacle_layer",
            "/move_base/local_costmap/obstacle_layer"))
        self.local_costmap_inflation_layer = str(rospy.get_param(
            "~local_costmap_inflation_layer",
            "/move_base/local_costmap/inflation_layer"))
        self.global_costmap_inflation_layer = str(rospy.get_param(
            "~global_costmap_inflation_layer",
            "/move_base/global_costmap/inflation_layer"))
        self.global_costmap_inflation_radius_m = float(rospy.get_param(
            "~global_costmap_inflation_radius_m", 0.224))
        self.pre_point_3_global_costmap_inflation_radius_m = float(
            rospy.get_param(
                "~pre_point_3_global_costmap_inflation_radius_m", 0.21))
        self.processing_parking_profile_enabled = bool(rospy.get_param(
            "~processing_parking_profile_enabled", True))
        self.processing_parking_inflation_radius_m = float(
            rospy.get_param("~processing_parking_inflation_radius_m", 0.07))

        self.start_delay = float(rospy.get_param("~start_delay", 2.0))
        self.resume_production_only = bool(
            rospy.get_param("~resume_production_only", False))
        self.move_base_ready_timeout = float(
            rospy.get_param("~move_base_ready_timeout", 90.0))
        self.safe_start_timeout = float(
            rospy.get_param("~safe_start_timeout", 45.0))
        self.plan_timeout = float(rospy.get_param("~plan_timeout", 15.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.target_guard_fallback_timeout = float(rospy.get_param(
            "~target_guard_fallback_timeout", 25.0))
        self.goal_cancel_timeout = float(
            rospy.get_param("~goal_cancel_timeout", 3.0))
        self.arrival_tolerance = float(
            rospy.get_param("~arrival_tolerance", 0.12))
        self.post_turn_recenter_trigger = float(
            rospy.get_param("~post_turn_recenter_trigger", 0.06))
        self.navigation_arrival_retry_attempts = max(
            0, int(rospy.get_param("~navigation_arrival_retry_attempts", 3)))
        self.continue_on_arrival_error = bool(rospy.get_param(
            "~continue_on_arrival_error", True))
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
            rospy.get_param("~qr_hold_seconds", 2.0)))
        self.qr_rotation_speed = abs(float(
            rospy.get_param("~qr_rotation_speed", 0.18)))
        self.fixed_heading_rotation_speed = abs(float(
            rospy.get_param("~fixed_heading_rotation_speed", 0.70)))
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
            rospy.get_param("~video_device", "/dev/ucar_camera"))
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
            "~ocr_scan_rotation_speed", 0.35)))
        self.ocr_scan_poll_period = float(rospy.get_param(
            "~ocr_scan_poll_period",
            rospy.get_param("~navigation_ocr_poll_period", 0.20)))
        self.ocr_scan_candidate_confidence = float(rospy.get_param(
            "~ocr_scan_candidate_confidence",
            rospy.get_param("~navigation_ocr_candidate_confidence", 60.0)))
        self.ocr_min_confidence = float(
            rospy.get_param("~ocr_min_confidence", 0.30))
        self.ocr_alignment_tolerance_px = float(
            rospy.get_param("~ocr_alignment_tolerance_px", 30.0))
        self.ocr_alignment_retry_tolerance_increment_px = float(
            rospy.get_param(
                "~ocr_alignment_retry_tolerance_increment_px", 20.0))
        self.ocr_candidate_min_bbox_area_px = float(
            rospy.get_param(
                "~ocr_candidate_min_bbox_area_px",
                1000.0))
        self.ocr_alignment_kp = float(
            rospy.get_param("~ocr_alignment_kp", 0.0025))
        self.ocr_alignment_kd = float(
            rospy.get_param("~ocr_alignment_kd", 0.00035))
        self.ocr_alignment_max_speed = abs(float(
            rospy.get_param("~ocr_alignment_max_speed", 0.22)))
        self.ocr_alignment_attempts = max(
            1, int(rospy.get_param("~ocr_alignment_attempts", 12)))
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
            rospy.get_param("~stop_confirmation_timeout", 4.0))
        self.stopped_odom_speed_epsilon = float(
            rospy.get_param("~stopped_odom_speed_epsilon", 0.02))
        self.stopped_odom_samples = max(
            1, int(rospy.get_param("~stopped_odom_samples", 3)))
        self.result_directory = os.path.expanduser(str(
            rospy.get_param(
                "~result_directory", "~/.ros/ucar_2026_national_observations")))
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
            0, int(rospy.get_param("~spark_retries", 0)))
        self.spark_timeout = max(
            0.5, float(rospy.get_param("~spark_timeout", 8.0)))
        self.spark_helper_ready_timeout = float(
            rospy.get_param("~spark_helper_ready_timeout", 10.0))

        arrival_tolerance_values = (
            self.arrival_tolerance, self.sprint_arrival_tolerance)
        if (not all(is_finite(value) for value in arrival_tolerance_values) or
                any(value <= 0.0 for value in arrival_tolerance_values)):
            raise TaskDefinitionError(
                "arrival tolerances must be finite and positive")
        if self.qr_rotation_speed <= 0.0:
            raise TaskDefinitionError("qr_rotation_speed must be positive")
        if self.fixed_heading_rotation_speed <= 0.0:
            raise TaskDefinitionError(
                "fixed_heading_rotation_speed must be positive")
        if (not is_finite(self.processing_dwell_seconds) or
                self.processing_dwell_seconds < 0.0):
            raise TaskDefinitionError(
                "processing_dwell_seconds must be finite and non-negative")
        simulation_timing_values = (
            self.simulation_start_timeout,
            self.simulation_done_timeout,
            self.simulation_poll_period,
            self.speak_wait_timeout,
        )
        if (not all(is_finite(value)
                    for value in simulation_timing_values) or
                self.simulation_start_timeout <= 0.0 or
                self.simulation_done_timeout <= 0.0 or
                self.simulation_poll_period <= 0.0 or
                self.speak_wait_timeout <= 0.0 or
                not (1 <= self.simulation_port <= 65535)):
            raise TaskDefinitionError(
                "simulation communication parameters are invalid")
        if (not is_finite(self.lane_handoff_timeout) or
                self.lane_handoff_timeout <= 0.0):
            raise TaskDefinitionError(
                "lane_handoff_timeout must be finite and positive")
        if (self.lane_handoff_enabled and not all((
                self.lane_activate_service.strip(),
                self.lane_owner_service.strip(),
                self.lane_state_topic.strip(),
                self.lane_result_topic.strip()))):
            raise TaskDefinitionError(
                "lane activation, owner, and state endpoints must be set")
        self.simulation_host = self.resolve_simulation_host()
        rospy.loginfo(
            "PRODUCTION_TASK_SIMULATION_HOST host=%s port=%d",
            self.simulation_host, self.simulation_port)
        if (not is_finite(self.tf_lookup_retry_seconds) or
                self.tf_lookup_retry_seconds < 0.0):
            raise TaskDefinitionError(
                "tf_lookup_retry_seconds must be finite and non-negative")
        if self.tts_enabled:
            if not self.tts_python.strip() or not self.tts_helper_path.strip():
                raise TaskDefinitionError(
                    "tts_python and tts_helper_path must be set when "
                    "tts_enabled")
        if (
                len(self.production_route_numbers) !=
                len(self.production_observation_headings)):
            raise TaskDefinitionError(
                "production route and observation headings have different "
                "lengths")
        if (
                len(self.fallback_production_route_numbers) !=
                len(self.fallback_production_observation_headings)):
            raise TaskDefinitionError(
                "fallback production route and observation headings have "
                "different lengths")
        if not self.fallback_production_route_numbers:
            raise TaskDefinitionError("fallback production route is empty")
        if self.ocr_alignment_max_speed <= 0.0:
            raise TaskDefinitionError(
                "ocr_alignment_max_speed must be positive")
        if (not is_finite(self.ocr_candidate_min_bbox_area_px) or
                self.ocr_candidate_min_bbox_area_px <= 0.0):
            raise TaskDefinitionError(
                "ocr_candidate_min_bbox_area_px must be positive")
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
        if len(self.destination_midpoint_point_numbers) not in (0, 2):
            raise TaskDefinitionError(
                "destination_midpoint_point_numbers must list exactly two "
                "point numbers or stay empty")

        all_required_numbers = (
            [self.staging_point_number] +
            self.qr_observation_numbers +
            self.production_route_numbers +
            self.fallback_production_route_numbers +
            [self.destination_point_number,
             self.destination_heading_point_number] +
            self.destination_midpoint_point_numbers +
            ([self.post_qr_waypoint_number]
             if self.post_qr_waypoint_number else []) +
            ([self.post_qr_waypoint_heading_point_number]
             if self.post_qr_waypoint_heading_point_number else []))
        if self.sprint_enabled:
            all_required_numbers += [self.sprint_start_point_number,
                                     self.sprint_end_point_number]
        self.points = load_numbered_points(self.grid_path)
        require_points(self.points, all_required_numbers)
        self.production_navigation_legs = [
            (self.staging_point_number, self.production_route_numbers[0])
        ] + list(zip(
            self.production_route_numbers[:-1],
            self.production_route_numbers[1:]))
        self.fallback_navigation_legs = [
            (self.production_route_numbers[-1],
             self.fallback_production_route_numbers[0])
        ] + list(zip(
            self.fallback_production_route_numbers[:-1],
            self.fallback_production_route_numbers[1:]))
        self.target_guard_points = load_middle_target_guard_points(
            self.grid_path, self.production_route_numbers)
        self.wall_reference_points = load_wall_reference_points(
            self.grid_path)
        (self.middle_zone_x_min, self.middle_zone_x_max,
         self.middle_zone_y_min, self.middle_zone_y_max,
         self.middle_zone_square_side) = load_middle_zone_geometry(
            self.grid_path)
        self.middle_zone_bounds = (
            self.middle_zone_x_min, self.middle_zone_x_max,
            self.middle_zone_y_min, self.middle_zone_y_max)
        self.ocr_stop_offset_m = float(rospy.get_param(
            "~ocr_stop_offset_m", self.middle_zone_square_side / 2.0))
        if (not is_finite(self.ocr_stop_offset_m) or
                self.ocr_stop_offset_m <= 0.0):
            raise TaskDefinitionError(
                "ocr_stop_offset_m must be finite and positive")
        if (not is_finite(self.processing_parking_inflation_radius_m) or
                self.processing_parking_inflation_radius_m <= 0.0 or
                self.processing_parking_inflation_radius_m >=
                self.ocr_stop_offset_m):
            raise TaskDefinitionError(
                "processing_parking_inflation_radius_m must be finite, "
                "positive, and smaller than ocr_stop_offset_m")
        if (not is_finite(self.global_costmap_inflation_radius_m) or
                self.global_costmap_inflation_radius_m <= 0.0):
            raise TaskDefinitionError(
                "global_costmap_inflation_radius_m must be finite and "
                "positive")
        if (not is_finite(
                self.pre_point_3_global_costmap_inflation_radius_m) or
                self.pre_point_3_global_costmap_inflation_radius_m <= 0.0):
            raise TaskDefinitionError(
                "pre_point_3_global_costmap_inflation_radius_m must be "
                "finite and positive")
        self._processing_parking_original_inflation_radius_m = None
        self._processing_parking_original_global_inflation_radius_m = None

        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.make_plan = rospy.ServiceProxy("move_base/make_plan", GetPlan)
        self.tf_listener = tf.TransformListener()
        self.cv_bridge = CvBridge()

        self.cmd_vel_topic = str(rospy.get_param(
            "~cmd_vel_topic", "/cmd_vel/navigation"))
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=10)
        self.qr_enable_pub = rospy.Publisher(
            "/qrcode_start_flag", Int8, queue_size=1, latch=True)
        self.navigation_mode_pub = rospy.Publisher(
            "/ucar/navigation_mode", String, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher(
            "/ucar_2026_national/task_state", String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher(
            "/ucar_2026_national/task_result", String, queue_size=1, latch=True)

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
        # QR codes on the real field carry a URL; qrcode_scanner resolves the
        # item name through the API and publishes it on /qr_api_result.  The
        # scan matcher consumes this API-resolved stream.
        self.api_sequence = 0
        self.latest_api_text = ""
        self.api_events = deque()
        self.first_qr_item_by_code = {}
        self.api_event = threading.Event()
        self.mission_started = False
        self.mission_finished = False
        self.ocr_process = None
        self.ocr_log_handle = None
        self.spark_process = None
        self.spark_request_sequence = 0
        self.spark_log_handle = None
        self.voice_input_process = None
        self.requested_real_category = None
        self.requested_sim_category = None
        self.qr_classifications = []
        self.observations = []
        self.target_scan_events = []
        self.target_guard_events = []
        self.run_directory = None
        self.capture_sequence = 0
        self.expected_item_text = u""
        self.expected_production_category = None
        self.expected_real_item_text = u""
        self.expected_sim_item_text = u""
        self.expected_real_category = None
        self.expected_sim_category = None
        self.served_wall_points = set()
        self._ocr_turn_stop_flag = False
        self.lane_state = ""
        self.lane_result = ""
        self.lane_state_event = threading.Event()

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
            "/qr_api_result", String, self.qr_api_result_cb, queue_size=20)
        rospy.Subscriber(
            "/rosout_agg", Log, self.rosout_cb, queue_size=100)
        rospy.Subscriber(
            self.lane_state_topic, String, self.lane_state_cb, queue_size=10)
        rospy.Subscriber(
            self.lane_result_topic, String, self.lane_result_cb, queue_size=10)
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
            "items": [self.expected_real_item_text,
                      self.expected_sim_item_text],
            "target_categories": {
                self.expected_real_item_text: self.expected_real_category,
                self.expected_sim_item_text: self.expected_sim_category,
            },
            "item_text": self.expected_item_text,
            "target_category": self.expected_production_category,
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

    def lane_state_cb(self, message):
        with self.lock:
            self.lane_state = message.data.strip()
        self.lane_state_event.set()

    def lane_result_cb(self, message):
        with self.lock:
            self.lane_result = message.data.strip()
        self.lane_state_event.set()

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

    def qr_api_result_cb(self, message):
        """Consume the API-resolved item name from qrcode_scanner.

        Real-field QR codes carry a URL (http://192.168.8.1:3663/<key>);
        qrcode_scanner queries the API and publishes
        ``{"ok": true, "response": {"code": 200, "result": "<item>"}}`` on
        /qr_api_result.  Only ok=true results with a non-empty item name
        feed the scan matcher (api_sequence / latest_api_text).
        """
        raw = message.data
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8", "replace")
        try:
            payload = json.loads(raw)
        except ValueError:
            rospy.logwarn(
                "PRODUCTION_QR_API_BAD_JSON %s", self.log_safe_text(raw))
            return
        if not payload.get("ok"):
            return
        response = payload.get("response")
        if not isinstance(response, dict):
            return
        item = self.normalize_qr_text(response.get("result", ""))
        if not item:
            return
        qr_code = self.normalize_qr_text(payload.get("qr_text", ""))
        with self.lock:
            if qr_code:
                first_item = self.first_qr_item_by_code.get(qr_code)
                if first_item is None:
                    self.first_qr_item_by_code[qr_code] = item
                elif first_item != item:
                    rospy.logwarn(
                        "PRODUCTION_QR_FIRST_ITEM_LOCKED code=%s first=%s "
                        "later=%s use_first=true",
                        self.log_safe_text(qr_code),
                        self.log_safe_text(first_item),
                        self.log_safe_text(item))
                    item = first_item
            self.api_sequence += 1
            self.latest_api_text = item
            self.api_events.append((self.api_sequence, qr_code, item))
        self.api_event.set()
        rospy.loginfo(
            "PRODUCTION_QR_API_EVENT sequence=%d item=%s",
            self.api_sequence, self.log_safe_text(item))

    def rosout_cb(self, message):
        text = message.msg.lower()
        if "crc16" in text and "imu" in text:
            rospy.logwarn_throttle(
                5.0, "PRODUCTION_IMU_CRC_IGNORED %s",
                self.log_safe_text(message.msg))
            return
        if "crc16" in text and "ahrs" in text:
            rospy.logwarn_throttle(
                5.0, "PRODUCTION_AHRS_CRC_IGNORED %s",
                self.log_safe_text(message.msg))
            return
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
        # Unit tests and a few maintenance tools construct this class via
        # object.__new__ instead of __init__; preserve their historical stdin
        # behaviour when the new launch parameter is absent.
        input_mode = getattr(self, "item_input_mode", "stdin")
        self.publish_state("WAITING_FOR_ITEM")
        if input_mode == "voice":
            # Do not run the microphone while the asynchronous start message
            # is still playing through the speaker: it can be recognised as a
            # command.  speak_wait has a bounded timeout and never aborts.
            self.speak_wait(
                u"初始化完成。请说小飞小飞后下达双物品指令")
        else:
            self.speak(u"初始化完成，准备开始任务")
        real_request, sim_request = self.wait_for_item_inputs()
        self.publish_state("WAITING_SAFE_START")
        if not self.move_base.wait_for_server(
                rospy.Duration(self.move_base_ready_timeout)):
            raise MissionAbort(
                "move_base unavailable after %.1f s" %
                self.move_base_ready_timeout)
        self.wait_for_safe_start()
        self.switch_to_point_mode()
        if self.resume_production_only:
            # Resume mode starts after the point-3 leg has already completed.
            self.set_global_costmap_inflation_radius(
                self.global_costmap_inflation_radius_m,
                "resume_after_point_3")
            self.set_local_costmap_dynamic_layers_enabled(
                True, "resume_after_point_3")
        else:
            self.set_global_costmap_inflation_radius(
                self.pre_point_3_global_costmap_inflation_radius_m,
                "before_point_3")
            self.set_local_costmap_dynamic_layers_enabled(
                False, "before_point_3")
        # The Spark QR classifier writes into result_directory during the QR
        # phase; create it before any classification (also covers resume).
        self.prepare_result_directory()

        if self.resume_production_only:
            if input_mode == "voice":
                raise MissionAbort(
                    "voice category input requires the QR phase; "
                    "resume_production_only is unsupported")
            self.qr_enable_pub.publish(Int8(data=0))
            rospy.logwarn(
                "PRODUCTION_TASK_RESUME production-only route=%s",
                self.production_route_numbers)
            self.classify_qr_text(0, real_request.encode("utf-8"))
            self.classify_qr_text(0, sim_request.encode("utf-8"))
            real_item = real_request
            sim_item = sim_request
            collected = {real_item: 0, sim_item: 0}
        else:
            staging = self.points[self.staging_point_number]
            first_observation = self.points[self.qr_observation_numbers[0]]
            staging_yaw = bearing(staging, first_observation)
            sprint_enabled = bool(getattr(self, "sprint_enabled", False))
            if sprint_enabled:
                sprint_start = self.points[self.sprint_start_point_number]
                if getattr(self, "sprint_end_xy", None) is not None:
                    sprint_end = self.sprint_end_xy
                    sprint_end_label = (
                        "sprint end midpoint (%.3f, %.3f)" % sprint_end)
                else:
                    sprint_end = self.points[self.sprint_end_point_number]
                    sprint_end_label = "sprint end point %d" % (
                        self.sprint_end_point_number)
                # 180 deg: after arriving at the start point the chassis
                # faces the y=1.75 corridor used for the sprint leg.
                sprint_yaw = math.radians(
                    float(getattr(self, "sprint_yaw_deg", 180.0)))
                self.publish_state(
                    "STAGING_%d" % self.sprint_start_point_number)
                self.navigate_coordinates(
                    sprint_start[0], sprint_start[1], sprint_yaw,
                    "sprint start point %d" % self.sprint_start_point_number,
                    require_plan=True)
                rospy.loginfo(
                    "PRODUCTION_SPRINT_LEG %d -> %d yaw=%.3f",
                    self.sprint_start_point_number,
                    self.sprint_end_point_number, sprint_yaw)
                self.publish_state(
                    "SPRINT_%d_%d" % (
                        self.sprint_start_point_number,
                        self.sprint_end_point_number))
                if bool(getattr(self, "sprint_transverse_enabled", False)):
                    self.switch_navigation_mode("transverse")
                else:
                    self.switch_navigation_mode("sprint")
                self.navigate_coordinates(
                    sprint_end[0], sprint_end[1], sprint_yaw,
                    sprint_end_label,
                    require_plan=True,
                    arrival_tolerance_override=self.sprint_arrival_tolerance)
                self.switch_navigation_mode("point")
                self.navigate_to(
                    self.staging_point_number, staging_yaw, "STAGING_52")
            else:
                self.navigate_to(
                    self.staging_point_number, staging_yaw, "STAGING_52")

            self.publish_state("QR_SEQUENCE")
            self.move_base.cancel_all_goals()
            self.stop_motion()
            self.wait_for_chassis_stop("camera start before QR sequence")
            self.wait_for_qr_scanner()
            self.start_ros_camera_and_wait("QR sequence")
            with self.lock:
                self.api_events.clear()
            self.qr_enable_pub.publish(Int8(data=1))
            if input_mode == "voice":
                collected_by_category = self.collect_target_qr_codes_by_category(
                    set([real_request, sim_request]))
                if len(collected_by_category) < 2:
                    raise MissionAbort(
                        "not all requested voice categories were collected "
                        "after 2 full QR rounds")
                real_item = collected_by_category[real_request]["item"]
                sim_item = collected_by_category[sim_request]["item"]
                self.set_expected_item_texts(real_item, sim_item)
                collected = {
                    real_item: collected_by_category[real_request][
                        "observation"],
                    sim_item: collected_by_category[sim_request][
                        "observation"],
                }
            else:
                real_item = real_request
                sim_item = sim_request
                collected = self.collect_target_qr_codes(
                    set([real_item, sim_item]))
            self.qr_enable_pub.publish(Int8(data=0))
            if len(collected) < 2:
                raise MissionAbort(
                    "not all target QR codes were collected after 2 "
                    "full rounds")
            if input_mode != "voice":
                self.classify_qr_text(
                    collected[real_item], real_item.encode("utf-8"))
                self.classify_qr_text(
                    collected[sim_item], sim_item.encode("utf-8"))
            self.stop_qr_classifier()
            self.stop_ros_camera_streaming(required=True)

            # This must happen while the car is still at the QR area: the
            # operator receives confirmation only after both QR item names
            # and their categories are known, but before the point-3 leg.
            real_category, sim_category = self.set_target_categories_from_qr(
                collected, real_item, sim_item)
            self.announce_qr_collection(real_item, sim_item)
            self.announce_item_destinations(
                real_item, real_category, sim_item, sim_category)

            if self.post_qr_waypoint_number:
                waypoint = self.points[self.post_qr_waypoint_number]
                waypoint_yaw = 0.0
                if self.post_qr_waypoint_heading_point_number:
                    heading_point = self.points[
                        self.post_qr_waypoint_heading_point_number]
                    waypoint_yaw = bearing(waypoint, heading_point)
                rospy.loginfo(
                    "PRODUCTION_POST_QR_WAYPOINT %d (no rotation)",
                    self.post_qr_waypoint_number)
                self.publish_state(
                    "WAYPOINT_%d" % self.post_qr_waypoint_number)
                self.navigate_coordinates(
                    waypoint[0], waypoint[1], waypoint_yaw,
                    "post-QR waypoint %d" % self.post_qr_waypoint_number,
                    require_plan=True)
                if getattr(self, "local_costmap_layer_control_enabled", False):
                    if (self.post_qr_waypoint_number !=
                            self.local_costmap_enable_waypoint_number):
                        raise MissionAbort(
                            "local costmap enable waypoint is %d, but the "
                            "post-QR waypoint is %d" % (
                                self.local_costmap_enable_waypoint_number,
                                self.post_qr_waypoint_number))
                    self.set_global_costmap_inflation_radius(
                        self.global_costmap_inflation_radius_m,
                        "reached_point_%d" %
                        self.post_qr_waypoint_number)
                    self.set_local_costmap_dynamic_layers_enabled(
                        True, "reached_point_%d" %
                        self.post_qr_waypoint_number)
            elif getattr(self, "local_costmap_layer_control_enabled", False):
                raise MissionAbort(
                    "post_qr_waypoint_number must be point %d when local "
                    "costmap layer control is enabled" %
                    self.local_costmap_enable_waypoint_number)

        if self.resume_production_only:
            real_category, sim_category = self.set_target_categories_from_qr(
                collected, real_item, sim_item)
            self.announce_item_destinations(
                real_item, real_category, sim_item, sim_category)

        if self.use_ros_camera_for_ocr:
            self.publish_state("OPEN_ROS_IMAGE_OCR")
            rospy.loginfo(
                "PRODUCTION_CAMERA_MODE ros_image topic=%s",
                self.camera_image_topic)
        else:
            self.release_ros_camera()
        self.start_native_ocr()

        # Cruise for the real category while also recording a simulation
        # category encountered first.  The latter is only a location record:
        # the real item must be parked and announced before the vehicle ever
        # parks for, or starts, the simulation item.
        categories_to_record = set([real_category, sim_category])
        rospy.loginfo(
            "PRODUCTION_TARGET_GROUPS %s arrival_ocr_turn=360deg "
            "real_category=%s record_categories=%s",
            self.production_route_groups,
            self.log_safe_text(real_category),
            self.log_safe_text(sorted(categories_to_record)))
        self.publish_state("PRODUCTION_CRUISE_1")
        self.reset_grouped_production_route()
        grouped_found = self.cruise_grouped_production_route(
            real_category, real_item, record_categories=categories_to_record)
        found_leg_index = 0 if grouped_found is not None else None
        active_legs = self.production_navigation_legs
        active_headings = self.production_observation_headings
        active_route_name = "primary"
        primary_grouped_route = grouped_found is not None
        if found_leg_index is None:
            rospy.loginfo(
                "PRODUCTION_FALLBACK_TARGET_LEGS %s target_category=%s",
                self.fallback_navigation_legs,
                self.log_safe_text(real_category))
            self.publish_state("PRODUCTION_FALLBACK_CRUISE_1")
            found_leg_index = self.cruise_production_route(
                self.fallback_navigation_legs, 1, real_category, real_item,
                observation_headings=
                self.fallback_production_observation_headings,
                record_categories=categories_to_record,
                route_name="fallback")
            active_legs = self.fallback_navigation_legs
            active_headings = self.fallback_production_observation_headings
            active_route_name = "fallback"
        if found_leg_index is None:
            self.finish_after_route_exhausted(
                [category for category in categories_to_record
                 if not self.production_category_recorded(category)])
            return

        self.park_at_recorded_production_category(
            real_item, real_category, announce=True)

        # If simulation OCR was seen before the real item, its location is
        # already stored but we deliberately have not driven to it yet.  When
        # it was not seen, continue from the next unvisited leg only after the
        # real-item announcement has completed.
        if not self.production_category_recorded(sim_category):
            if primary_grouped_route:
                rospy.loginfo(
                    "PRODUCTION_SIM_GROUPED_RESUME target_category=%s",
                    self.log_safe_text(sim_category))
                self.publish_state("PRODUCTION_CRUISE_2")
                self.cruise_grouped_production_route(
                    sim_category, sim_item,
                    record_categories=set([sim_category]))
            else:
                second_legs = active_legs[found_leg_index + 1:]
                if second_legs:
                    rospy.loginfo(
                        "PRODUCTION_SIM_LEGS route=%s start_leg_index=%d "
                        "legs=%s target_category=%s",
                        active_route_name, found_leg_index + 1, second_legs,
                        self.log_safe_text(sim_category))
                    self.publish_state("PRODUCTION_CRUISE_2")
                    self.cruise_production_route(
                        second_legs, found_leg_index + 2, sim_category, sim_item,
                        observation_headings=active_headings,
                        record_categories=set([sim_category]),
                        route_name=active_route_name)
        if (not self.production_category_recorded(sim_category) and
                active_route_name == "primary"):
            rospy.loginfo(
                "PRODUCTION_FALLBACK_SIM_LEGS %s target_category=%s",
                self.fallback_navigation_legs,
                self.log_safe_text(sim_category))
            self.publish_state("PRODUCTION_FALLBACK_CRUISE_2")
            self.cruise_production_route(
                self.fallback_navigation_legs, 1, sim_category, sim_item,
                observation_headings=self.fallback_production_observation_headings,
                record_categories=set([sim_category]),
                route_name="fallback")
        if not self.production_category_recorded(sim_category):
            self.finish_after_route_exhausted([sim_category])
            return

        # This is intentionally after the real-item announcement above.
        self.park_at_recorded_production_category(
            sim_item, sim_category, announce=False)

        self.stop_native_ocr()
        self.ensure_ros_camera_released()
        self.save_observation_summary()

        self.publish_state("SIMULATION_START")
        # /start 失败时也进入 /status 兜底轮询，simulation_done_timeout（75s）到期后继续任务（与 08-14 的超时继续语义一致）
        self.simulation_request_start(sim_item, sim_category)
        self.publish_state("SIMULATION_WAIT_DONE")
        simulation_completed = self.simulation_wait_done()
        if simulation_completed:
            self.publish_state("SIMULATION_DONE")
            rospy.loginfo(
                "PRODUCTION_SIMULATION_FINISHED item=%s category=%s",
                self.log_safe_text(sim_item),
                self.log_safe_text(sim_category))
            self.speak_wait(
                u"仿真任务已完成，已将%s放入%s" % (sim_item, sim_category))
        else:
            self.publish_state("SIMULATION_TIMEOUT_CONTINUE")
            rospy.logwarn(
                "PRODUCTION_SIMULATION_TIMEOUT_CONTINUE timeout=%.1f",
                self.simulation_done_timeout)
            self.speak_wait(
                u"仿真任务已完成，已将%s放入%s" % (sim_item, sim_category))

        self.finish_at_destination(
            "recognized both target categories; announced processing "
            "stops; simulation completed")

    def finish_after_route_exhausted(self, missing_categories):
        """Release OCR and continue to 441 after both cruise routes miss."""
        missing_categories = sorted(set(missing_categories))
        rospy.logwarn(
            "PRODUCTION_ROUTE_EXHAUSTED missing_categories=%s; "
            "continuing to destination",
            self.log_safe_text(missing_categories))
        self.stop_native_ocr()
        self.ensure_ros_camera_released()
        self.save_observation_summary()
        self.publish_state("PRODUCTION_ROUTE_EXHAUSTED")
        self.finish_at_destination(
            "production routes exhausted before categories %s were located; "
            "continued to destination" %
            self.log_safe_text(missing_categories))

    def finish_at_destination(self, reason):
        """Navigate near the final area, finish radar parking, then publish success."""
        if self.destination_midpoint_point_numbers:
            first = self.points[
                self.destination_midpoint_point_numbers[0]]
            second = self.points[
                self.destination_midpoint_point_numbers[1]]
            destination = (
                (first[0] + second[0]) / 2.0,
                (first[1] + second[1]) / 2.0)
            destination_label = "midpoint %d-%d" % tuple(
                self.destination_midpoint_point_numbers)
        else:
            destination = self.points[self.destination_point_number]
            destination_label = "point %d" % self.destination_point_number
        heading_point = self.points[self.destination_heading_point_number]
        destination_yaw = bearing(destination, heading_point)
        self.publish_state(
            "DESTINATION_%s" %
            "_".join(str(value)
                     for value in self.destination_midpoint_point_numbers)
            if self.destination_midpoint_point_numbers
            else "DESTINATION_%d" % self.destination_point_number)
        self.switch_to_destination_mode()
        self.navigate_coordinates(
            destination[0], destination[1], destination_yaw,
            "destination %s" % destination_label,
            require_plan=True)
        # Navigation only brings the vehicle to the final-area handoff point.
        # The task is not successful until the resident lane node reports the
        # radar corner controller's GOAL result.
        self.handoff_to_lane()
        self.publish_state("SUCCEEDED")
        self.publish_result(
            True, "%s; arrived and parked at destination %s" %
            (reason, destination_label))
        rospy.signal_shutdown("lane following completed")

    @staticmethod
    def normalize_qr_text(value):
        """Decode a QR-derived text to unicode for item-name comparison.

        /qr_result and the /qr_api_result item name carry UTF-8 bytes on
        Python 2; item names are unicode.
        """
        if isinstance(value, bytes):
            try:
                return value.decode("utf-8")
            except UnicodeDecodeError:
                return value.decode("utf-8", "replace")
        return value

    def _reject_qr_code(self, detected, observation_number):
        """Remember a published code (first occurrence only) and keep going."""
        self.used_qr_codes.add(detected)
        rospy.loginfo(
            "PRODUCTION_QR_IGNORED observation=%s value=%s",
            observation_number, self.log_safe_text(detected))

    def collect_target_qr_codes(self, targets, rounds=2):
        """Scan the QR faces until every target code is collected once.

        Each round runs in two stages.  Stage 1 faces every fixed
        observation direction (navigate-face / fresh-wait skeleton) without
        any revolution fallback, so the vehicle searches all three
        directions by face-first only.  Only when every fixed direction has
        been searched and the targets are still not all collected does stage
        2 revisit each direction with the full 360-degree revolution
        fallback (fresh events are accepted while turning); stage 2 stops as
        soon as every target is collected.  A fresh code is accepted only
        when it is one of ``targets`` and has not been collected yet.
        Non-target codes and repeat publications are ignored (they still
        enter used_qr_codes at their first publication).
        Returns {item_name: observation_number}.
        """
        collected = {}

        def accept_text(raw_text):
            text = self.normalize_qr_text(raw_text)
            return text in targets and text not in collected

        for round_index in range(1, rounds + 1):
            if len(collected) >= len(targets):
                break
            rospy.loginfo(
                "PRODUCTION_QR_ROUND %d/%d collected=%d/%d",
                round_index, rounds, len(collected), len(targets))
            for observation_number in self.qr_observation_numbers:
                if len(collected) >= len(targets):
                    break
                detected = self.scan_observation_point(
                    observation_number, accept_text, allow_revolution=False)
                if detected is None:
                    continue
                text = self.normalize_qr_text(detected)
                if text in targets and text not in collected:
                    collected[text] = int(observation_number)
                    rospy.loginfo(
                        "PRODUCTION_QR_COLLECTED observation=%d "
                        "value=%s collected=%d/%d",
                        observation_number, self.log_safe_text(text),
                        len(collected), len(targets))
            if len(collected) < len(targets):
                for observation_number in self.qr_observation_numbers:
                    if len(collected) >= len(targets):
                        break
                    rospy.loginfo(
                        "PRODUCTION_QR_REVOLUTION_FALLBACK observation=%d",
                        observation_number)
                    detected = self.scan_observation_point(
                        observation_number, accept_text, allow_revolution=True)
                    if detected is None:
                        continue
                    text = self.normalize_qr_text(detected)
                    if text in targets and text not in collected:
                        collected[text] = int(observation_number)
                        rospy.loginfo(
                            "PRODUCTION_QR_COLLECTED observation=%d "
                            "value=%s collected=%d/%d",
                            observation_number, self.log_safe_text(text),
                            len(collected), len(targets))
        return collected

    def collect_target_qr_codes_by_category(
            self, requested_categories, rounds=2):
        """Resolve voice-requested categories to real QR item names.

        ``wake_listen.py --json`` emits the requested production categories,
        not the text printed by field QR codes.  This collector accepts each
        distinct QR item once, classifies it immediately, and retains the
        first item for every requested category.  Thus the mission never
        pretends a category name is a QR item name, while the later simulation
        request and announcements still carry the actual item.

        Each round runs in two stages, like collect_target_qr_codes: stage 1
        faces every fixed observation direction without any revolution
        fallback; only when all fixed directions have been searched and the
        requested categories are still not all collected does stage 2 revisit
        each direction with the 360-degree revolution fallback (stopping as
        soon as every category is collected).
        """
        requested_categories = set(requested_categories)
        invalid = requested_categories.difference(VOICE_REQUEST_CATEGORIES)
        if invalid:
            raise MissionAbort(
                "voice input requested unsupported categories: %s" %
                self.log_safe_text(sorted(invalid)))
        collected = {}
        seen_items = set()

        def accept_text(raw_text):
            text = self.normalize_qr_text(raw_text)
            return bool(text) and text not in seen_items

        for round_index in range(1, rounds + 1):
            if len(collected) >= len(requested_categories):
                break
            rospy.loginfo(
                "PRODUCTION_VOICE_QR_ROUND %d/%d collected=%d/%d",
                round_index, rounds, len(collected),
                len(requested_categories))
            for observation_number in self.qr_observation_numbers:
                if len(collected) >= len(requested_categories):
                    break
                detected = self.scan_observation_point(
                    observation_number, accept_text, allow_revolution=False)
                if detected is None:
                    continue
                item_text = self.normalize_qr_text(detected)
                if not item_text or item_text in seen_items:
                    continue
                self.classify_qr_text(
                    observation_number, item_text.encode("utf-8"))
                category = self.qr_classification_category_for_item(
                    observation_number, item_text)
                if category is None:
                    self.used_qr_codes.discard(item_text)
                    rospy.logwarn(
                        "PRODUCTION_VOICE_QR_UNCLASSIFIED observation=%d "
                        "item=%s retry=true",
                        observation_number, self.log_safe_text(item_text))
                    continue
                seen_items.add(item_text)
                if category not in requested_categories:
                    rospy.loginfo(
                        "PRODUCTION_VOICE_QR_IGNORED observation=%d "
                        "item=%s category=%s",
                        observation_number, self.log_safe_text(item_text),
                        self.log_safe_text(category))
                    continue
                if category in collected:
                    rospy.loginfo(
                        "PRODUCTION_VOICE_QR_DUPLICATE_CATEGORY "
                        "observation=%d item=%s category=%s",
                        observation_number, self.log_safe_text(item_text),
                        self.log_safe_text(category))
                    continue
                collected[category] = {
                    "item": item_text,
                    "observation": int(observation_number),
                }
                rospy.loginfo(
                    "PRODUCTION_VOICE_QR_COLLECTED observation=%d "
                    "item=%s category=%s collected=%d/%d",
                    observation_number, self.log_safe_text(item_text),
                    self.log_safe_text(category), len(collected),
                    len(requested_categories))
            if len(collected) < len(requested_categories):
                for observation_number in self.qr_observation_numbers:
                    if len(collected) >= len(requested_categories):
                        break
                    rospy.loginfo(
                        "PRODUCTION_VOICE_QR_REVOLUTION_FALLBACK "
                        "observation=%d", observation_number)
                    detected = self.scan_observation_point(
                        observation_number, accept_text, allow_revolution=True)
                    if detected is None:
                        continue
                    item_text = self.normalize_qr_text(detected)
                    if not item_text or item_text in seen_items:
                        continue
                    self.classify_qr_text(
                        observation_number, item_text.encode("utf-8"))
                    category = self.qr_classification_category_for_item(
                        observation_number, item_text)
                    if category is None:
                        self.used_qr_codes.discard(item_text)
                        rospy.logwarn(
                            "PRODUCTION_VOICE_QR_UNCLASSIFIED observation=%d "
                            "item=%s retry=true",
                            observation_number,
                            self.log_safe_text(item_text))
                        continue
                    seen_items.add(item_text)
                    if category not in requested_categories:
                        rospy.loginfo(
                            "PRODUCTION_VOICE_QR_IGNORED observation=%d "
                            "item=%s category=%s",
                            observation_number, self.log_safe_text(item_text),
                            self.log_safe_text(category))
                        continue
                    if category in collected:
                        rospy.loginfo(
                            "PRODUCTION_VOICE_QR_DUPLICATE_CATEGORY "
                            "observation=%d item=%s category=%s",
                            observation_number, self.log_safe_text(item_text),
                            self.log_safe_text(category))
                        continue
                    collected[category] = {
                        "item": item_text,
                        "observation": int(observation_number),
                    }
                    rospy.loginfo(
                        "PRODUCTION_VOICE_QR_COLLECTED observation=%d "
                        "item=%s category=%s collected=%d/%d",
                        observation_number, self.log_safe_text(item_text),
                        self.log_safe_text(category), len(collected),
                        len(requested_categories))
        return collected

    def scan_observation_point(self, observation_number, accept_text=None,
                               allow_revolution=True):
        """Face one QR observation point; return the first accepted code.

        ``accept_text`` decides whether a freshly decoded code is kept; the
        default keeps the first distinct code (legacy behaviour).  A fresh
        code that is rejected is recorded in used_qr_codes and the scan
        continues.  The vehicle faces the point and waits up to
        qr_search_timeout.  If a QR code is seen but rejected (for example,
        it names an item outside the input targets), this face is complete:
        return None so the collection loop advances to the next fixed face.

        ``allow_revolution=False`` makes this face do nothing but "face the
        point and wait for a scan" -- no 360-degree revolution fallback.  In
        that mode a face that still yields nothing returns None and the
        collection loop advances to the next fixed direction; the
        full-revolution fallback is triggered by the caller only after every
        fixed direction has been searched.  The full-revolution fallback is
        otherwise reserved for a face where no QR code was seen at all.
        """
        staging = self.points[self.staging_point_number]
        observation = self.points[observation_number]
        observation_yaw = bearing(staging, observation)
        self.publish_state("QR_FACE_%d" % observation_number)
        # Capture the sequence before turning so a code acquired while the
        # chassis is settling at this new face is accepted immediately.
        with self.lock:
            baseline = self.api_sequence
        self.navigate_coordinates(
            staging[0], staging[1], observation_yaw,
            "QR face point %d" % observation_number,
            require_plan=False)

        detected, rejected = self.accepted_qr_after(
            baseline, accept_text, observation_number,
            report_rejection=True)
        if detected is None and not rejected:
            detected, wait_rejected = self.wait_for_fresh_qr(
                baseline, self.qr_search_timeout, accept_text,
                observation_number, report_rejection=True)
            rejected = rejected or wait_rejected
        if detected is None:
            if rejected:
                rospy.loginfo(
                    "PRODUCTION_QR_FACE_REJECTED observation=%d "
                    "advance_to_next_face=true",
                    observation_number)
                return None
            if not allow_revolution:
                return None
            self.publish_state("QR_SEARCH_TURN_%d" % observation_number)
            detected = self.rotate_full_revolution(
                "QR observation point %d" % observation_number,
                self.qr_rotation_speed,
                stop_for_qr=True,
                qr_baseline=baseline,
                qr_accept=accept_text,
                qr_observation_number=observation_number)
        if detected is None:
            return None
        self.used_qr_codes.add(detected)
        rospy.loginfo(
            "PRODUCTION_QR_ACCEPTED observation=%d value=%s",
            observation_number, self.log_safe_text(detected))
        return detected

    def accepted_qr_after(self, baseline, accept_text, observation_number,
                          report_rejection=False):
        """Inspect the first fresh QR; optionally report a rejection."""
        detected = self.fresh_qr_after(baseline)
        if detected is None:
            return (None, False) if report_rejection else None
        if accept_text is None or accept_text(detected):
            return (detected, False) if report_rejection else detected
        self._reject_qr_code(detected, observation_number)
        return (None, True) if report_rejection else None

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

    def wait_for_fresh_qr(self, baseline, timeout, accept_text=None,
                          observation_number=None, report_rejection=False):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        search_baseline = baseline
        rejected = False
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            detected = self.fresh_qr_after(search_baseline)
            if detected is not None:
                if accept_text is None or accept_text(detected):
                    return (
                        (detected, rejected)
                        if report_rejection else detected)
                self._reject_qr_code(detected, observation_number)
                if report_rejection:
                    return None, True
                rejected = True
                with self.lock:
                    search_baseline = self.api_sequence
            # Wait on the subscription callback instead of imposing a fixed
            # observation delay.  The bounded wait still lets safety state
            # and ROS shutdown be checked when no QR is visible.
            self.api_event.wait(0.05)
            self.api_event.clear()
        return (None, rejected) if report_rejection else None

    def fresh_qr_after(self, baseline):
        """Return the next queued API-resolved item name.

        The scanner can publish multiple QR results for one camera frame.  A
        FIFO queue preserves all of them; ``baseline`` remains in the method
        signature for the turn/search callers but no longer selects only the
        latest result.  Repeated item names are consumed and skipped.
        """
        with self.lock:
            while self.api_events:
                _sequence, _qr_code, text = self.api_events.popleft()
                if (self.require_distinct_qr_codes and
                        text in self.used_qr_codes):
                    continue
                return text
        return None

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
        with self.lock:
            baseline_sequence = self.camera_sequence
        deadline = (
            rospy.Time.now() +
            rospy.Duration(self.camera_frame_timeout))
        message = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.require_safe()
            with self.lock:
                sequence = self.camera_sequence
                receipt = self.latest_camera_receipt
                candidate = self.latest_camera_image
            if (
                    sequence > baseline_sequence and
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
            # task owns the map pose at the exposure request time.  Preserve
            # both fields for observation audit records after asynchronous
            # inference; the parked observation uses the current yaw.
            response = dict(response)
            response["capture_requested_at"] = task["capture_requested_at"]
            response["capture_requested_pose_map"] = list(
                task["capture_requested_pose_map"])
        return response

    def capture_ocr_while_turning(
            self, signed_speed, capture_label, attempt):
        """Capture one fresh OCR frame while continuously turning in place.

        The OCR helper runs in a worker while this supervisor keeps publishing
        a bounded angular command at ``rotation_control_rate``.  A successful
        return deliberately leaves the command active: the caller either
        immediately requests the next frame with an updated speed or sends
        zero after the image is aligned.  Any failure path stops first.
        """
        speed = float(signed_speed)
        if speed == 0.0:
            raise MissionAbort("continuous OCR alignment has zero turn speed")
        direction = 1.0 if speed > 0.0 else -1.0
        if abs(speed) < self.ocr_alignment_min_speed:
            speed = self.ocr_alignment_min_speed * direction
        command = Twist()
        command.angular.z = speed
        task = None
        completed = False
        try:
            # Publish before asking the worker for the next image so its
            # exposure belongs to the continuous turn rather than the prior
            # parked pose.  The loop below keeps refreshing the command.
            self.require_safe()
            self.cmd_vel_pub.publish(command)
            task = self.start_async_motion_ocr(
                "%s_moving_%02d" % (capture_label, attempt))
            deadline = time.time() + self.ocr_capture_timeout + 2.0
            rate = rospy.Rate(self.rotation_control_rate)
            while not task["done"].is_set():
                self.require_safe()
                if time.time() >= deadline:
                    raise MissionAbort(
                        "continuous OCR capture %s timed out" %
                        capture_label)
                self.cmd_vel_pub.publish(command)
                rate.sleep()
            response = self.finish_async_motion_ocr(task)
            completed = True
            return response
        finally:
            if not completed:
                self.stop_motion()
                if task is not None:
                    self.cleanup_async_motion_ocr(task)

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
            # Leave one second for the helper to emit its local fallback
            # before the task-level response deadline expires.
            "--timeout", str(max(0.5, self.spark_timeout - 1.0)),
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

    def read_spark_message(self, timeout, context, expected_request_id=None):
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
                message = json.loads(raw_line)
            except ValueError:
                rospy.logwarn(
                    "PRODUCTION_SPARK_NON_JSON %s",
                    raw_line.decode("utf-8", "replace").strip()
                    if not isinstance(raw_line, str) else raw_line.strip())
                continue
            if (expected_request_id is not None and
                    message.get("request_id") != expected_request_id):
                rospy.logwarn(
                    "PRODUCTION_SPARK_STALE_RESPONSE context=%s "
                    "expected=%s actual=%s",
                    context, expected_request_id,
                    message.get("request_id"))
                continue
            return message
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

    @staticmethod
    def print_terminal(text):
        """Print a human-facing UTF-8 line to the launch terminal.

        rospy log lines must stay ASCII (Python 2 crash lesson 2026-08-06),
        so Chinese meant for the operator goes straight to stdout instead.
        The task node runs with output="screen", so this appears in the
        mission terminal.  Text arrives as unicode and is encoded to UTF-8
        bytes before printing.
        """
        if isinstance(text, unicode):
            text = text.encode("utf-8")
        print(text)
        sys.stdout.flush()

    def speak(self, text):
        """Play one TTS announcement asynchronously; never aborts the task."""
        if not self.tts_enabled:
            return
        try:
            payload = text
            if isinstance(payload, bytes):
                payload = payload.decode("utf-8", "replace")
            rospy.loginfo(
                "PRODUCTION_TTS_SPEAK text=%s",
                self.log_safe_text(payload))
            self.print_terminal(u"[播报] %s" % payload)
            argv = [
                self.tts_python, self.tts_helper_path,
                payload.encode("utf-8")]
            devnull = open(os.devnull, "wb")
            try:
                subprocess.Popen(argv, stdout=devnull, stderr=devnull)
            finally:
                devnull.close()
        except (OSError, IOError, ValueError) as exc:
            rospy.logwarn("PRODUCTION_TASK_TTS_FAILED %s", exc)

    def warehouse_name_for_category(self, category):
        """Warehouse name announced for a production category.

        The wording matches the OCR workshop signs on the field; the
        electronics sign reads "电子产品生产车间" (production, not 加工).
        An unknown category falls back to "<category>加工车间" so the
        announcement still plays, with a warning logged.
        """
        name = PRODUCTION_WAREHOUSE_NAMES.get(category)
        if name is None:
            rospy.logwarn(
                "PRODUCTION_TTS_WAREHOUSE_UNKNOWN category=%s",
                self.log_safe_text(category))
            return u"%s加工车间" % category
        return name

    def set_target_categories_from_qr(self, collected, real_item, sim_item):
        """Resolve and persist both QR item categories before production OCR."""
        real_category = self.qr_classification_category_for_item(
            collected[real_item], real_item)
        if real_category is None:
            raise MissionAbort(
                "QR item category was not recognised for observation %d" %
                collected[real_item])
        sim_category = self.qr_classification_category_for_item(
            collected[sim_item], sim_item)
        if sim_category is None:
            raise MissionAbort(
                "QR item category was not recognised for observation %d" %
                collected[sim_item])
        self.expected_real_category = real_category
        self.expected_sim_category = sim_category
        self.expected_production_category = real_category
        rospy.loginfo(
            "PRODUCTION_TASK_TARGET_CATEGORIES real=%s sim=%s",
            self.log_safe_text(real_category),
            self.log_safe_text(sim_category))
        return real_category, sim_category

    def announce_qr_collection(self, real_item, sim_item):
        """Confirm the complete QR collection before leaving the QR area."""
        self.publish_state("QR_ITEMS_ANNOUNCE")
        # 按用户要求不再播报"二维码识别完成，已获取XX和XX"

    def announce_item_destinations(
            self, real_item, real_category, sim_item, sim_category):
        """Announce the two resolved category-to-workshop assignments."""
        real_warehouse = self.warehouse_name_for_category(real_category)
        sim_warehouse = self.warehouse_name_for_category(sim_category)
        self.speak_wait(
            u"取得*%s*属于*%s*应放置在*%s" % (
                real_item, real_category, real_warehouse))
        self.speak_wait(
            u"仿真环境中取得*%s*属于*%s*应放置在*%s" % (
                sim_item, sim_category, sim_warehouse))

    def speak_wait(self, text, timeout=None):
        """Play one TTS announcement synchronously; timeout never aborts.

        The helper process exits only after the audio finished playing, so
        waiting for its exit means the announcement is complete.  A timeout
        terminates the helper, logs a warning and continues the mission.
        """
        if not self.tts_enabled:
            return
        if timeout is None:
            timeout = self.speak_wait_timeout
        try:
            payload = text
            if isinstance(payload, bytes):
                payload = payload.decode("utf-8", "replace")
            rospy.loginfo(
                "PRODUCTION_TTS_SPEAK text=%s",
                self.log_safe_text(payload))
            self.print_terminal(u"[播报] %s" % payload)
            argv = [
                self.tts_python, self.tts_helper_path,
                payload.encode("utf-8")]
            devnull = open(os.devnull, "wb")
            try:
                process = subprocess.Popen(argv, stdout=devnull,
                                           stderr=devnull)
            finally:
                devnull.close()
        except (OSError, IOError, ValueError) as exc:
            rospy.logwarn("PRODUCTION_TASK_TTS_FAILED %s", exc)
            return
        deadline = time.time() + float(timeout)
        while (process.poll() is None and time.time() < deadline and
                not rospy.is_shutdown()):
            time.sleep(0.1)
        if process.poll() is not None:
            rospy.loginfo(
                "PRODUCTION_TASK_TTS_WAIT_FINISHED code=%d",
                process.returncode)
            return
        process.terminate()
        time.sleep(0.2)
        if process.poll() is None:
            process.kill()
        rospy.logwarn(
            "PRODUCTION_TASK_TTS_TIMEOUT seconds=%.1f text=%s",
            float(timeout), self.log_safe_text(text))

    def resolve_simulation_host(self):
        """Return the explicitly configured reachable PC simulation host."""
        host = self.simulation_host.strip()
        if not host:
            raise MissionAbort(
                "simulation_host must be explicitly configured; "
                "ROS_MASTER_URI points at the vehicle")
        return host

    def handoff_to_lane(self):
        """Activate resident lane following without an intermediate stop."""
        if not self.lane_handoff_enabled:
            return
        self.set_ros_camera_streaming(True, required=True)
        try:
            rospy.wait_for_service(self.lane_activate_service, timeout=10.0)
            rospy.wait_for_service(self.lane_owner_service, timeout=10.0)
            activate_lane = rospy.ServiceProxy(
                self.lane_activate_service, SetBool)
            activation = activate_lane(True)
            if not activation.success:
                raise MissionAbort(
                    "lane activation rejected: %s" % activation.message)
            set_lane_owner = rospy.ServiceProxy(
                self.lane_owner_service, SetBool)
            owner_result = set_lane_owner(True)
            if not owner_result.success:
                raise MissionAbort(
                    "lane command ownership rejected: %s" %
                    owner_result.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise MissionAbort("lane handoff service failed: %s" % exc)

        self.publish_state("LANE_ACTIVE")
        deadline = time.time() + self.lane_handoff_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                lane_state = self.lane_state
                lane_result = self.lane_result
            if lane_state == "STOPPED":
                if lane_result in ("ABORT", "ESTOP", "CONFIG"):
                    raise MissionAbort(
                        "lane parking stopped with result %s" % lane_result)
                if lane_result == "GOAL":
                    rospy.loginfo("PRODUCTION_LANE_HANDOFF_COMPLETED result=GOAL")
                    return
            self.lane_state_event.wait(0.10)
            self.lane_state_event.clear()
        raise MissionAbort(
            "lane following did not reach STOPPED within %.1f s" %
            self.lane_handoff_timeout)

    def simulation_request_start(self, item_name, category):
        """Ask the local simulation to start; retries then falls back.

        POST /start with the item name and its category.  A 2xx reply with
        ``accepted`` returns; HTTP 409 means the bridge already has a
        running/finished task and falls through to status polling; other
        failures retry up to
        simulation_start_retries before returning False, letting the caller
        fall back to /status polling in simulation_wait_done (whose
        simulation_done_timeout of 75 s keeps the mission moving).
        """
        url = "http://%s:%d/start" % (
            self.simulation_host, self.simulation_port)
        body = json.dumps({
            "item_name": item_name,
            "category": category,
        }).encode("utf-8")
        last_error = "unknown"
        for attempt in range(1, self.simulation_start_retries + 1):
            self.require_safe()
            try:
                request = urllib2.Request(
                    url, data=body,
                    headers={"Content-Type": "application/json"})
                response = urllib2.urlopen(
                    request, timeout=self.simulation_start_timeout)
                try:
                    payload = json.loads(response.read())
                finally:
                    response.close()
            except urllib2.HTTPError as exc:
                if exc.code == 409:
                    rospy.logwarn(
                        "PRODUCTION_SIMULATION_START_409_CONTINUE item=%s",
                        self.log_safe_text(item_name))
                    return False
                last_error = "http=%d" % exc.code
                rospy.logwarn(
                    "PRODUCTION_SIMULATION_START_RETRY attempt=%d/%d "
                    "http=%d",
                    attempt, self.simulation_start_retries, exc.code)
            except (urllib2.URLError, IOError) as exc:
                last_error = str(exc)
                rospy.logwarn(
                    "PRODUCTION_SIMULATION_START_RETRY attempt=%d/%d "
                    "error=%s",
                    attempt, self.simulation_start_retries,
                    self.log_safe_text(str(exc)))
            except ValueError:
                last_error = "non-json reply"
                rospy.logwarn(
                    "PRODUCTION_SIMULATION_START_RETRY attempt=%d/%d "
                    "non-json reply",
                    attempt, self.simulation_start_retries)
            else:
                if not payload.get("accepted"):
                    raise MissionAbort(
                        "simulation /start was not accepted: %s" % payload)
                rospy.loginfo(
                    "PRODUCTION_SIMULATION_START_ACCEPTED item=%s "
                    "category=%s",
                    self.log_safe_text(item_name),
                    self.log_safe_text(category))
                return True
            if attempt < self.simulation_start_retries:
                time.sleep(2.0)
        rospy.logwarn(
            "PRODUCTION_SIMULATION_START_FAILED_CONTINUE item=%s error=%s",
            self.log_safe_text(item_name), self.log_safe_text(last_error))
        return False

    def simulation_wait_done(self):
        """Poll /status until the simulation reports done; safety stays on.

        Every poll opens a new HTTP connection, so transport failures retry
        on the next period.  The vehicle continues after the configured
        timeout whether the simulator reports running, failed, or no status.
        """
        url = "http://%s:%d/status" % (
            self.simulation_host, self.simulation_port)
        deadline = time.time() + self.simulation_done_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            self.require_safe()
            try:
                response = urllib2.urlopen(url, timeout=10.0)
                try:
                    payload = json.loads(response.read())
                finally:
                    response.close()
            except (urllib2.URLError, IOError, httplib.HTTPException) as exc:
                rospy.logwarn_throttle(
                    2.0, "PRODUCTION_SIMULATION_STATUS_RECONNECT error=%s",
                    self.log_safe_text(str(exc)))
                time.sleep(self.simulation_poll_period)
                continue
            except ValueError:
                rospy.logwarn_throttle(
                    2.0, "PRODUCTION_SIMULATION_STATUS_BAD_JSON")
                time.sleep(self.simulation_poll_period)
                continue
            state = payload.get("state")
            if state == "done":
                rospy.loginfo(
                    "PRODUCTION_SIMULATION_STATUS_DONE %s",
                    json.dumps(payload, ensure_ascii=True))
                return True
            if state == "failed":
                rospy.logwarn_throttle(
                    2.0, "PRODUCTION_SIMULATION_STATUS_FAILED_WAITING %s",
                    self.log_safe_text(str(payload.get("detail", payload))))
            time.sleep(self.simulation_poll_period)
        if rospy.is_shutdown():
            raise MissionAbort("ROS shutdown while waiting for simulation")
        rospy.logwarn(
            "PRODUCTION_SIMULATION_WAIT_TIMEOUT_CONTINUE timeout=%.1f",
            self.simulation_done_timeout)
        return False

    def wait_for_item_inputs(self):
        """Wait for the two task requests from voice or standard input.

        Voice mode receives the two requested categories from the microphone
        listener.  QR scanning resolves those categories to actual field item
        names before the production route begins.  ``stdin`` remains an
        explicit development fallback and receives the two actual item names.
        """
        if getattr(self, "item_input_mode", "stdin") == "voice":
            real_category, sim_category = self.wait_for_voice_item_categories()
            self.requested_real_category = real_category
            self.requested_sim_category = sim_category
            return real_category, sim_category
        real_item = self._read_item_input(
            u"PRODUCTION_TASK_INPUT_PROMPT: 请输入现实物品名称后回车")
        sim_item = self._read_item_input(
            u"PRODUCTION_TASK_INPUT_PROMPT: 请输入仿真物品名称后回车")
        if real_item == sim_item:
            raise MissionAbort("real and simulation items must be different")
        self.set_expected_item_texts(real_item, sim_item)
        return real_item, sim_item

    def set_expected_item_texts(self, real_item, sim_item):
        """Store the real QR item names used by the rest of the mission."""
        self.expected_real_item_text = real_item
        self.expected_sim_item_text = sim_item
        self.expected_item_text = real_item
        rospy.loginfo(
            "PRODUCTION_TASK_ITEMS real=%s sim=%s",
            self.log_safe_text(real_item), self.log_safe_text(sim_item))

    def wait_for_voice_item_categories(self):
        """Run wake_listen until it emits one valid dual-category JSON line.

        The listener stays in ``--loop`` mode, so an ASR failure or incomplete
        sentence simply leads to another wake-word attempt.  Its human-facing
        progress logs share stdout with JSON; only an ``ok=true`` JSON object
        with both supported, distinct slots is accepted.  The process is
        always stopped before returning so it cannot retain the microphone or
        receive later TTS audio.
        """
        listener_path = self.voice_listener_path
        if not os.path.isfile(listener_path):
            raise MissionAbort(
                "voice listener does not exist: %s" % listener_path)
        wake_word = self.normalize_qr_text(self.voice_wake_word)
        if not wake_word:
            raise MissionAbort("voice wake word is empty")
        command = [
            self.voice_listener_python, "-u", listener_path,
            "--loop", "--asr", "--set-wake", wake_word.encode("utf-8"),
            "--json",
        ]
        try:
            self.voice_input_process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                bufsize=0)
        except (IOError, OSError) as exc:
            self.voice_input_process = None
            raise MissionAbort("cannot start voice listener: %s" % exc)
        rospy.loginfo(
            "PRODUCTION_VOICE_INPUT_STARTED listener=%s wake_word=%s",
            listener_path, self.log_safe_text(wake_word))
        rospy.loginfo(
            "PRODUCTION_VOICE_WAITING waiting for wake word and "
            "dual-category command")

        timeout = float(self.voice_input_timeout)
        deadline = time.time() + timeout if timeout > 0.0 else None
        buffered = b""
        try:
            while not rospy.is_shutdown():
                if deadline is not None and time.time() >= deadline:
                    raise MissionAbort(
                        "voice input timed out after %.1f s" % timeout)
                process = self.voice_input_process
                if process is None or process.poll() is not None:
                    raise MissionAbort(
                        "voice listener exited before a valid command")
                readable, _writable, _errors = select.select(
                    [process.stdout], [], [], 0.1)
                if not readable:
                    continue
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk:
                    continue
                buffered += chunk
                lines = buffered.split(b"\n")
                buffered = lines.pop()
                for raw_line in lines:
                    decoded_line = raw_line
                    if isinstance(decoded_line, bytes):
                        decoded_line = decoded_line.decode(
                            "utf-8", "replace")
                    decoded_line = decoded_line.strip()
                    if decoded_line:
                        display_line = decoded_line
                        if u"\r" in display_line:
                            display_line = display_line.split(
                                u"\r")[-1].strip()
                        if display_line.startswith(u"音量"):
                            display_line = u""
                        try:
                            json.loads(decoded_line)
                        except ValueError:
                            if display_line:
                                self.print_terminal(
                                    u"[语音] %s" % display_line)
                    parsed = self.parse_voice_listener_message(raw_line)
                    if parsed is None:
                        continue
                    real_category, sim_category = parsed
                    if real_category == sim_category:
                        rospy.logwarn(
                            "PRODUCTION_VOICE_INPUT_REJECTED "
                            "duplicate_category=%s; please repeat command",
                            self.log_safe_text(real_category))
                        continue
                    rospy.loginfo(
                        "PRODUCTION_VOICE_INPUT_ACCEPTED real_category=%s "
                        "sim_category=%s",
                        self.log_safe_text(real_category),
                        self.log_safe_text(sim_category))
                    return real_category, sim_category
            raise MissionAbort("ROS shutdown while waiting for voice input")
        finally:
            self.stop_voice_listener()

    def parse_voice_listener_message(self, raw_line):
        """Return two validated categories from one wake_listen output line."""
        if isinstance(raw_line, bytes):
            raw_line = raw_line.decode("utf-8", "replace")
        raw_line = raw_line.strip()
        if not raw_line:
            return None
        try:
            message = json.loads(raw_line)
        except ValueError:
            # wake_listen prints setup/progress text too; it is not a command.
            return None
        if not isinstance(message, dict) or not message.get("ok"):
            return None
        slots = message.get("slots")
        if not isinstance(slots, dict):
            rospy.logwarn("PRODUCTION_VOICE_INPUT_REJECTED missing slots")
            return None
        real_category = self.normalize_qr_text(slots.get(u"取件类别", ""))
        sim_category = self.normalize_qr_text(slots.get(u"仿真类别", ""))
        if (real_category not in VOICE_REQUEST_CATEGORIES or
                sim_category not in VOICE_REQUEST_CATEGORIES):
            rospy.logwarn(
                "PRODUCTION_VOICE_INPUT_REJECTED unsupported real=%s sim=%s",
                self.log_safe_text(real_category),
                self.log_safe_text(sim_category))
            return None
        return real_category, sim_category

    def stop_voice_listener(self):
        """Release the microphone-listener process without leaving it behind."""
        process = getattr(self, "voice_input_process", None)
        self.voice_input_process = None
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            deadline = time.time() + 3.0
            while process.poll() is None and time.time() < deadline:
                time.sleep(0.05)
            if process.poll() is None:
                process.kill()
        try:
            process.wait()
        except OSError:
            pass
        try:
            process.stdout.close()
        except (AttributeError, IOError, OSError):
            pass
        rospy.loginfo("PRODUCTION_VOICE_INPUT_STOPPED")

    def _read_item_input(self, prompt):
        if sys.version_info[0] < 3:
            sys.stdout.write(prompt.encode("utf-8"))
        else:
            sys.stdout.write(prompt)
        sys.stdout.write("\n")
        sys.stdout.flush()
        raw_line = sys.stdin.readline()
        if isinstance(raw_line, bytes):
            text = raw_line.decode("utf-8", "replace")
        else:
            text = raw_line
        item_text = text.strip()
        if not item_text:
            raise MissionAbort("no item text was provided on standard input")
        rospy.loginfo("PRODUCTION_TASK_ITEM %s", self.log_safe_text(item_text))
        return item_text

    def qr_classification_entry(self, observation_number):
        for entry in self.qr_classifications:
            if (entry.get("observation") == int(observation_number) and
                    entry.get("category")):
                return entry
        return None

    def qr_classification_category(self, observation_number):
        entry = self.qr_classification_entry(observation_number)
        if entry is None:
            return None
        category = entry.get("category")
        if category is None or category == "null":
            return None
        return category

    def qr_classification_category_for_item(self, observation_number,
                                            item_text):
        """Category classified for one item name at its observation face."""
        for entry in reversed(self.qr_classifications):
            if (entry.get("observation") == int(observation_number) and
                    entry.get("qr_text") == item_text):
                category = entry.get("category")
                if category is None or category == "null":
                    return None
                return category
        return None

    def production_category_recorded(self, category):
        return self.last_recorded_observation(category) is not None

    def last_recorded_observation(self, category):
        for observation in reversed(self.observations):
            if (observation.get("processing_category") == category and
                    observation.get("wall_point_number") is not None):
                return observation
        return None

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
        self.spark_request_sequence += 1
        request_id = self.spark_request_sequence
        try:
            payload = {
                "command": "classify",
                "request_id": request_id,
                "qr_text": qr_text,
            }
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
            observation_number, expected_request_id=request_id)
        if not response:
            entry["error"] = "classifier response timeout"
            self.stop_qr_classifier()
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

    def rotate_in_place_to_yaw(self, target_yaw, context):
        """Turn a same-position goal through the measured shortest yaw arc."""
        self.move_base.cancel_all_goals()
        self.stop_motion()
        self.require_safe()
        self.wait_for_chassis_stop(context + " start")
        previous_yaw = self.current_odom_yaw(context + " start")
        target_delta = shortest_yaw_delta(previous_yaw, target_yaw)
        target_progress = abs(target_delta)
        required_progress = max(
            0.0, target_progress - self.rotation_completion_tolerance)
        if required_progress <= 0.0:
            return

        direction = 1.0 if target_delta > 0.0 else -1.0
        speed = self.fixed_heading_rotation_speed * direction
        timeout = (
            target_progress / abs(speed) * self.rotation_timeout_scale + 2.0)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        progress = 0.0
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
                        "PRODUCTION_QR_FACE_TURN context=%s "
                        "target_yaw=%.3f direction=%s requested=%.3f "
                        "actual=%.3f",
                        context, target_yaw,
                        "ccw" if direction > 0.0 else "cw",
                        target_progress, progress)
                    return
                self.cmd_vel_pub.publish(command)
                rate.sleep()
        finally:
            self.stop_motion()

        self.wait_for_chassis_stop(context + " timeout settle")
        final_yaw = self.current_odom_yaw(context + " timeout settle")
        progress += positive_turn_increment(
            previous_yaw, final_yaw, direction)
        if progress >= required_progress:
            rospy.loginfo(
                "PRODUCTION_QR_FACE_TURN context=%s target_yaw=%.3f "
                "direction=%s requested=%.3f actual=%.3f settled=true",
                context, target_yaw,
                "ccw" if direction > 0.0 else "cw",
                target_progress, progress)
            return
        raise MissionAbort(
            "%s did not reach target yaw %.3f within %.1f s "
            "(requested=%.3f actual=%.3f)" %
            (context, target_yaw, timeout, target_progress, progress))

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
            self, leg_index, start_number, end_number, target_yaw,
            target_category=None, record_categories=None,
            route_name="primary", route_leg_count=None,
            next_target_number=None, navigation_point_number=None,
            guard_points_override=None, fallback_navigation=False):
        """Guard one logical target leg, then scan its selected pose.

        ``navigation_point_number`` can select a clear corner pose while
        ``end_number`` remains the logical production target.
        """
        navigation_number = int(
            navigation_point_number
            if navigation_point_number is not None else end_number)
        target = self.points[navigation_number]
        label = "PRODUCTION_TARGET_%03d" % end_number
        self.last_target_guard_fallback_candidates = []
        if route_leg_count is None:
            route_leg_count = len(self.production_navigation_legs)
        self.publish_state(label)
        rospy.loginfo(
            "PRODUCTION_TARGET_GOAL route=%s index=%d/%d start=%d end=%d "
            "navigation_point=%d target=(%.3f, %.3f) yaw=%.3f",
            route_name, leg_index, route_leg_count,
            start_number, end_number, navigation_number,
            target[0], target[1], target_yaw)
        if guard_points_override is None:
            monitor = self.new_target_guard_monitor(end_number)
        else:
            monitor = self.new_target_guard_monitor(
                end_number, guard_points_override)
        guard_number = self.wait_for_target_guard_precheck(monitor)
        if guard_number is not None:
            self.stop_motion()
            self.wait_for_chassis_stop(label + " target guard before goal")
            self.last_target_guard_fallback_candidates = (
                self.target_guard_fallback_candidates(end_number, monitor))
            self.record_target_guard_skip(
                leg_index, start_number, end_number, guard_number,
                "before_goal", monitor, route_name, next_target_number)
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
            abort_on_navigation_failure=not fallback_navigation,
            goal_timeout=(self.target_guard_fallback_timeout
                          if fallback_navigation else None),
            guard_callback=guard_callback)
        if navigation_guard["number"] is not None:
            self.last_target_guard_fallback_candidates = (
                self.target_guard_fallback_candidates(end_number, monitor))
            self.record_target_guard_skip(
                leg_index, start_number, end_number,
                navigation_guard["number"], "during_navigation", monitor,
                route_name, next_target_number)
            return "target_guard_skipped"
        if navigation_guard["scan_unavailable"]:
            raise MissionAbort(
                "%s target guard scan became unavailable during navigation" %
                label)
        if not reached:
            if fallback_navigation:
                rospy.logwarn(
                    "PRODUCTION_TARGET_FALLBACK_NAVIGATION_FAILED "
                    "target=%d navigation_point=%d",
                    end_number, navigation_number)
                return "target_navigation_failed"
            raise MissionAbort("%s did not reach target" % label)
        self.scan_production_point(
            leg_index, start_number, end_number, label,
            target_category=target_category,
            record_categories=record_categories)

    def target_guard_fallback_candidates(self, target_number, monitor):
        """Return target vertices not hit by the latest guard scan."""
        guard_points = monitor.get("guard_points")
        if guard_points is None:
            guard_points = getattr(self, "target_guard_points", {}).get(
                int(target_number), {})
        blocked = set(monitor.get("hit_counts", {}).keys())
        return [
            number for number in sorted(guard_points)
            if number not in blocked]

    def reset_grouped_production_route(self):
        """Reset the forward-group and reverse-completion OCR scheduler."""
        configured_groups = getattr(self, "production_route_groups", None)
        if configured_groups is None:
            configured_groups = [list(self.production_route_numbers)]
        self.grouped_route_groups = [list(group) for group in configured_groups]
        self.grouped_point_states = dict(
            (int(number), "untried")
            for group in self.grouped_route_groups
            for number in group)
        self.grouped_route_phase = "forward"
        self.grouped_forward_group_index = 0
        self.grouped_forward_candidate_index = 0
        self.grouped_reverse_group_index = len(self.grouped_route_groups) - 1
        self.grouped_reverse_candidate_index = 0
        self.grouped_current_point_number = int(
            getattr(self, "post_qr_waypoint_number", 0) or
            getattr(self, "staging_point_number", 52))
        self.grouped_route_attempt_index = 0

    def grouped_observation_heading(self, point_number):
        """Return the OCR heading configured for a target point."""
        try:
            route_index = self.production_route_numbers.index(int(point_number))
        except ValueError:
            raise MissionAbort(
                "grouped OCR point %d is absent from production route" %
                int(point_number))
        return self.production_observation_headings[route_index]

    def next_grouped_production_target(self):
        """Return and advance the next target in the current group phase."""
        while True:
            if self.grouped_route_phase == "forward":
                while (self.grouped_forward_group_index <
                       len(self.grouped_route_groups)):
                    group = self.grouped_route_groups[
                        self.grouped_forward_group_index]
                    while (self.grouped_forward_candidate_index <
                           len(group)):
                        point_number = group[self.grouped_forward_candidate_index]
                        self.grouped_forward_candidate_index += 1
                        if (self.grouped_point_states[point_number] ==
                                "untried"):
                            return ("forward",
                                    self.grouped_forward_group_index,
                                    point_number)
                    self.grouped_forward_group_index += 1
                    self.grouped_forward_candidate_index = 0
                self.grouped_route_phase = "reverse"
                continue

            while self.grouped_reverse_group_index >= 0:
                group = self.grouped_route_groups[
                    self.grouped_reverse_group_index]
                while (self.grouped_reverse_candidate_index < len(group)):
                    point_number = group[self.grouped_reverse_candidate_index]
                    self.grouped_reverse_candidate_index += 1
                    if self.grouped_point_states[point_number] == "untried":
                        return ("reverse",
                                self.grouped_reverse_group_index,
                                point_number)
                self.grouped_reverse_group_index -= 1
                self.grouped_reverse_candidate_index = 0
            return None

    def peek_grouped_production_target(self):
        """Return the next untried point without advancing the scheduler."""
        phase = self.grouped_route_phase
        if phase == "forward":
            for group_index in range(
                    self.grouped_forward_group_index,
                    len(self.grouped_route_groups)):
                group = self.grouped_route_groups[group_index]
                first = (self.grouped_forward_candidate_index
                         if group_index == self.grouped_forward_group_index
                         else 0)
                for point_number in group[first:]:
                    if self.grouped_point_states[point_number] == "untried":
                        return point_number
            phase = "reverse"
        if phase == "reverse":
            for group_index in range(
                    self.grouped_reverse_group_index, -1, -1):
                group = self.grouped_route_groups[group_index]
                first = (self.grouped_reverse_candidate_index
                         if group_index == self.grouped_reverse_group_index
                         else 0)
                for point_number in group[first:]:
                    if self.grouped_point_states[point_number] == "untried":
                        return point_number
        return None

    def try_grouped_target_guard_fallback(
            self, phase, group_index, point_number, start_number,
            target_yaw, target_category, record_categories, next_point,
            candidates):
        """Try clear corner poses around a guard-blocked grouped target."""
        remaining = list(candidates)
        target_guard_points = self.target_guard_points[point_number]
        while remaining:
            candidate = remaining.pop(0)
            candidate_points = [candidate] + remaining
            guard_points = dict(
                (number, target_guard_points[number])
                for number in candidate_points
                if number in target_guard_points)
            self.grouped_route_attempt_index += 1
            rospy.loginfo(
                "PRODUCTION_TARGET_GUARD_FALLBACK target=%d "
                "navigation_point=%d clear_points=%s phase=%s group=%d",
                point_number, candidate, sorted(guard_points), phase,
                group_index + 1)
            outcome = self.navigate_target_and_scan(
                self.grouped_route_attempt_index, start_number, point_number,
                target_yaw=target_yaw,
                target_category=target_category,
                record_categories=record_categories,
                route_name="primary_grouped_%s_corner" % phase,
                route_leg_count=len(self.production_route_numbers),
                next_target_number=next_point,
                navigation_point_number=candidate,
                guard_points_override=guard_points,
                fallback_navigation=True)
            if outcome == "target_navigation_failed":
                rospy.logwarn(
                    "PRODUCTION_TARGET_GUARD_FALLBACK_NEXT target=%d "
                    "failed_navigation_point=%d remaining=%s",
                    point_number, candidate, remaining)
                continue
            if outcome != "target_guard_skipped":
                return outcome
            fresh_candidates = getattr(
                self, "last_target_guard_fallback_candidates", [])
            remaining = [
                number for number in remaining
                if number in fresh_candidates]
        return "target_guard_skipped"

    def cruise_grouped_production_route(
            self, target_category, target_item, record_categories=None):
        """Run grouped OCR forward, then reverse through every pending point."""
        if not hasattr(self, "grouped_route_phase"):
            self.reset_grouped_production_route()
        base_observation_count = len(self.observations)
        while True:
            next_target = self.next_grouped_production_target()
            if next_target is None:
                rospy.loginfo(
                    "PRODUCTION_GROUPED_ROUTE_EXHAUSTED target_category=%s "
                    "states=%s",
                    self.log_safe_text(target_category),
                    self.grouped_point_states)
                return None
            phase, group_index, point_number = next_target
            self.grouped_route_attempt_index += 1
            start_number = self.grouped_current_point_number
            next_point = self.peek_grouped_production_target()
            rospy.loginfo(
                "PRODUCTION_GROUPED_TARGET phase=%s group=%d point=%d "
                "start=%d next=%s attempt=%d/%d",
                phase, group_index + 1, point_number, start_number,
                str(next_point), self.grouped_route_attempt_index,
                len(self.production_route_numbers))
            outcome = self.navigate_target_and_scan(
                self.grouped_route_attempt_index, start_number, point_number,
                target_yaw=self.grouped_observation_heading(point_number),
                target_category=target_category,
                record_categories=record_categories,
                route_name="primary_grouped_%s" % phase,
                route_leg_count=len(self.production_route_numbers),
                next_target_number=next_point)
            if outcome == "target_guard_skipped":
                outcome = self.try_grouped_target_guard_fallback(
                    phase, group_index, point_number, start_number,
                    self.grouped_observation_heading(point_number),
                    target_category, record_categories, next_point,
                    getattr(self, "last_target_guard_fallback_candidates", []))
            if outcome == "target_guard_skipped":
                self.grouped_point_states[point_number] = "ignored"
                rospy.loginfo(
                    "PRODUCTION_GROUPED_POINT_IGNORED phase=%s group=%d "
                    "point=%d",
                    phase, group_index + 1, point_number)
                continue

            self.grouped_point_states[point_number] = "scanned"
            self.grouped_current_point_number = point_number
            if phase == "forward":
                self.grouped_forward_group_index = group_index + 1
                self.grouped_forward_candidate_index = 0
            if (len(self.observations) > base_observation_count and
                    self.production_category_recorded(target_category)):
                rospy.loginfo(
                    "PRODUCTION_GROUPED_CATEGORY_FOUND point=%d "
                    "category=%s item=%s phase=%s group=%d",
                    point_number, self.log_safe_text(target_category),
                    self.log_safe_text(target_item), phase, group_index + 1)
                return self.grouped_route_attempt_index - 1

    def cruise_production_route(self, legs, start_segment_index,
                                target_category, target_item,
                                observation_headings=None,
                                record_categories=None,
                                route_name="primary"):
        """Run ``legs`` until target_category is recorded during this round.

        Returns the 0-based leg index (into production_navigation_legs) of
        the leg that first recorded the category this round, or None when
        the route ends without a match.  ``start_segment_index`` keeps the
        1-based global leg numbering used by guard auditing and headings.
        """
        if observation_headings is None:
            observation_headings = self.production_observation_headings
        base_observation_count = len(self.observations)
        for local_leg_index, (start_number, end_number) in enumerate(legs):
            segment_index = start_segment_index + local_leg_index
            next_target_number = (
                legs[local_leg_index + 1][1]
                if local_leg_index + 1 < len(legs) else None)
            self.navigate_target_and_scan(
                segment_index, start_number, end_number,
                target_yaw=observation_headings[segment_index - 1],
                target_category=target_category,
                record_categories=record_categories,
                route_name=route_name, route_leg_count=len(legs),
                next_target_number=next_target_number)
            if (len(self.observations) > base_observation_count and
                    self.production_category_recorded(target_category)):
                rospy.loginfo(
                    "PRODUCTION_TARGET_CATEGORY_FOUND route_point=%d "
                    "category=%s item=%s leg_index=%d",
                    end_number, self.log_safe_text(target_category),
                    self.log_safe_text(target_item), segment_index - 1)
                return segment_index - 1
        return None

    def park_at_recorded_production_category(self, item, category,
                                             announce=False):
        """Park at one recorded category wall, optionally confirming delivery."""
        self.stop_motion()
        self.wait_for_chassis_stop("stop before processing area stop")
        observation = self.last_recorded_observation(category)
        if observation is None:
            raise MissionAbort(
                "recorded production category is unavailable: %s" %
                self.log_safe_text(category))
        wall_number = observation["wall_point_number"]
        wall_coordinate = observation["wall_point_coordinate"]
        intersection = observation.get(
            "forward_ray_wall_intersection_map", wall_coordinate)
        stop_x, stop_y = stop_point_for_wall_point(
            intersection, self.ocr_stop_offset_m,
            self.middle_zone_bounds)
        # Park with the chassis front facing the measured wall intersection.
        parking_yaw = normalize_angle(
            bearing((stop_x, stop_y), intersection))
        self.publish_state("PROCESSING_STOP_%03d" % wall_number)
        rospy.loginfo(
            "PRODUCTION_PROCESSING_STOP wall_point=%d wall_coordinate=%s "
            "intersection=%s stop=%.3f,%.3f item=%s",
            wall_number, wall_coordinate, intersection, stop_x, stop_y,
            self.log_safe_text(item))
        route_point_number = observation["route_point_number"]
        ocr_aligned_pose = observation["ocr_aligned_pose_map"]
        parking_profile_enabled = getattr(
            self, "processing_parking_profile_enabled", False)
        try:
            if parking_profile_enabled:
                self.publish_state(
                    "PROCESSING_APPROACH_%03d" % route_point_number)
                rospy.loginfo(
                    "PRODUCTION_PROCESSING_APPROACH route_point=%d "
                    "target=(%.3f,%.3f) yaw=%.3f item=%s "
                    "inflation=normal",
                    route_point_number, ocr_aligned_pose[0],
                    ocr_aligned_pose[1], ocr_aligned_pose[2],
                    self.log_safe_text(item))
                self.navigate_coordinates(
                    ocr_aligned_pose[0], ocr_aligned_pose[1],
                    ocr_aligned_pose[2],
                    "processing observation point %d" % route_point_number,
                    require_plan=True)
                self.enter_processing_parking_profile()
            self.navigate_coordinates(
                stop_x, stop_y, parking_yaw,
                "processing stop point %d" % wall_number,
                require_plan=True)
        finally:
            if parking_profile_enabled:
                self.exit_processing_parking_profile()
        self.stop_motion()
        if announce:
            self.publish_state("PROCESSING_ANNOUNCE_%03d" % wall_number)
            rospy.loginfo(
                "PRODUCTION_PROCESSING_ANNOUNCE wall_point=%d item=%s "
                "category=%s",
                wall_number, self.log_safe_text(item),
                self.log_safe_text(category))
            self.speak_wait(u"已将%s放入%s" % (item, category))
        return observation

    def new_target_guard_monitor(self, target_number, guard_points=None):
        """Start a new guard epoch; old scans may not affect a new target."""
        target_number = int(target_number)
        if guard_points is None:
            guard_points = getattr(self, "target_guard_points", {}).get(
                target_number, {})
        else:
            guard_points = dict(guard_points)
        guard_enabled = bool(guard_points)
        if not guard_enabled:
            # The four-point static guard is defined for middle production
            # targets, including the side-wall vertices used by boundary
            # cells.  Perimeter fallback cells keep the normal task safety
            # gate and move_base obstacle layers without a target-cell guard.
            rospy.logwarn(
                "PRODUCTION_TARGET_GUARD_UNAVAILABLE target=%d "
                "using navigation safety layers", target_number)
        with self.lock:
            sequence = self.target_guard_scan_sequence
        return {
            "target_number": target_number,
            "guard_points": guard_points,
            "guard_enabled": guard_enabled,
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
        if not monitor.get("guard_enabled", True):
            return None
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
            monitor["guard_points"],
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
        if not monitor.get("guard_enabled", True):
            return False
        last_usable_receipt = monitor.get("last_usable_receipt")
        if last_usable_receipt is None:
            return True
        return (
            (rospy.Time.now() - last_usable_receipt).to_sec() >
            self.target_guard_scan_max_age)

    def wait_for_target_guard_precheck(self, monitor):
        """Use fresh pre-goal evidence without delaying a clean target leg."""
        if not monitor.get("guard_enabled", True):
            return None
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
            monitor, route_name="primary", next_target_number=None):
        """Audit a guard decision before the outer route advances one leg."""
        stamp = monitor.get("last_scan_stamp")
        guard_points = monitor.get("guard_points", {})
        event = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "target_point_number": int(end_number),
            "guard_point_number": int(guard_number),
            "guard_point_numbers": sorted(guard_points),
            "route_name": str(route_name),
            "segment_index": int(leg_index),
            "segment_start_point_number": int(start_number),
            "next_target_point_number": next_target_number,
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
            "route_name": str(route_name),
            "phase": str(phase),
        })
        self.publish_state(
            "TARGET_GUARD_SKIP_%03d_%03d" % (end_number, guard_number))
        rospy.logwarn(
            "PRODUCTION_TARGET_GUARD_SKIP route=%s target=%d guard_point=%d "
            "phase=%s next_target=%s",
            route_name, end_number, guard_number, phase,
            (str(next_target_number)
             if next_target_number is not None else "none"))
        self.save_observation_summary()

    def log_small_ocr_candidate(
            self, response, context, minimum_confidence):
        """Explain why a visible but too-small OCR box was skipped."""
        if not isinstance(response, dict):
            return
        detection = response.get("detection")
        if not isinstance(detection, dict):
            return
        text = (detection.get("text") or "").strip()
        confidence = float(detection.get("confidence", -1.0))
        area = ocr_detection_bbox_area(detection)
        if (not text or confidence < float(minimum_confidence) or
                area >= self.ocr_candidate_min_bbox_area_px):
            return
        rospy.loginfo(
            "PRODUCTION_OCR_CANDIDATE_IGNORED_SMALL context=%s text=%s "
            "confidence=%.1f bbox=%s area_px=%.1f threshold_px=%.1f",
            context, json.dumps(text, ensure_ascii=True), confidence,
            self.log_safe_text(detection.get("bbox")), area,
            self.ocr_candidate_min_bbox_area_px)

    def scan_production_point(
            self, leg_index, start_number, point_number, target_label,
            target_category=None, record_categories=None):
        """Complete one stationary 360-degree scan and record new classes.

        ``record_categories`` limits which categories may be persisted.  The
        scan stops early only after every category in
        ``record_categories`` has been recorded.  Otherwise the full
        revolution is completed; ``target_category`` only identifies the
        category hunted by the enclosing cruise.
        A (category, wall_point_number) pair already in served_wall_points is
        never recorded again, so a category can be stopped at once per wall.
        """
        scan_label = "PRODUCTION_OCR_TURN_%03d" % point_number
        self.publish_state(scan_label)
        if self.use_ros_camera_for_ocr:
            self.start_ros_camera_and_wait(scan_label)
        try:
            rejected_categories = set()
            required_categories = (
                tuple(record_categories)
                if record_categories is not None else None)

            def all_required_categories_recorded():
                if required_categories is not None:
                    return (bool(required_categories) and
                            all(self.production_category_recorded(category)
                                for category in required_categories))
                return (target_category is not None and
                        self.production_category_recorded(target_category))

            def handle_candidate(response, turn_progress):
                detection = response["detection"]
                category = normalize_production_category(detection.get("text"))
                if category is None:
                    return False
                if (record_categories is not None and
                        category not in record_categories):
                    rospy.loginfo(
                        "PRODUCTION_CATEGORY_IGNORED category=%s "
                        "record_categories=%s route_point=%d",
                        category.encode("utf-8"),
                        self.log_safe_text(sorted(record_categories)),
                        point_number)
                    return False
                if category in rejected_categories:
                    rospy.loginfo(
                        "PRODUCTION_CATEGORY_SKIP_RETRY category=%s "
                        "route_point=%d reason=alignment_rejected",
                        category.encode("utf-8"), point_number)
                    return False
                # A category already recorded during this cruise (first pass
                # pre-record of the simulation category) must not trigger the
                # full stop/align/range cycle again: that repeated the same
                # wall point forever at route point 13 / wall 448 and never
                # advanced the turn.  Skip such candidates and keep turning.
                # (2026-08-11: ALREADY_SERVED infinite turn loop fix, round 2.)
                if (category != target_category and
                        self.production_category_recorded(category)):
                    rospy.loginfo(
                        "PRODUCTION_CATEGORY_SKIP_RECORDED category=%s "
                        "route_point=%d text=%s",
                        category.encode("utf-8"), point_number,
                        json.dumps(detection.get("text"),
                                   ensure_ascii=True))
                    return False
                self.stop_motion()
                self.wait_for_chassis_stop(scan_label + " candidate")
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
                if (observation["aligned"] and
                        observation.get("wall_point_number") is not None):
                    served_key = (
                        category, observation["wall_point_number"])
                    if served_key in self.served_wall_points:
                        # First pass pre-records the simulation category
                        # (seen while hunting the real category) into
                        # served_wall_points without stopping.  When the
                        # second pass returns to the same wall hunting that
                        # category, the pre-record must not block the real
                        # stop; re-record it as the current target stop.
                        # (2026-08-11: infinite ALREADY_SERVED turn loop at
                        # route point 13 / wall 448 was fixed this way.)
                        if category == target_category:
                            if not any(
                                    existing.get("processing_category") ==
                                    category and
                                    existing.get("wall_point_number") ==
                                    observation["wall_point_number"]
                                    for existing in self.observations):
                                self.observations.append(observation)
                            if all_required_categories_recorded():
                                self._ocr_turn_stop_flag = True
                            event["outcome"] = (
                                "processing_category_recorded")
                            rospy.loginfo(
                                "PRODUCTION_CATEGORY_RECORDED category=%s "
                                "route_point=%d wall_point=%d "
                                "coordinate=(%.3f,%.3f) text=%s "
                                "(second-pass stop overriding pre-record)",
                                category.encode("utf-8"), point_number,
                                observation["wall_point_number"],
                                observation["wall_point_coordinate"][0],
                                observation["wall_point_coordinate"][1],
                                json.dumps(
                                    observation["text"], ensure_ascii=True))
                        else:
                            event["outcome"] = (
                                "processing_category_already_served")
                            rospy.loginfo(
                                "PRODUCTION_CATEGORY_ALREADY_SERVED "
                                "category=%s wall_point=%d route_point=%d",
                                category.encode("utf-8"),
                                observation["wall_point_number"],
                                point_number)
                    else:
                        self.served_wall_points.add(served_key)
                        self.observations.append(observation)
                        # A single wall can expose multiple recordable
                        # categories.  Keep turning until all categories
                        # required by this scan have been recorded.
                        if all_required_categories_recorded():
                            self._ocr_turn_stop_flag = True
                        event["outcome"] = "processing_category_recorded"
                        rospy.loginfo(
                            "PRODUCTION_CATEGORY_RECORDED category=%s "
                            "route_point=%d wall_point=%d "
                            "coordinate=(%.3f,%.3f) text=%s",
                            category.encode("utf-8"), point_number,
                            observation["wall_point_number"],
                            observation["wall_point_coordinate"][0],
                            observation["wall_point_coordinate"][1],
                            json.dumps(
                                observation["text"], ensure_ascii=True))
                else:
                    rejected_categories.add(category)
                    event["outcome"] = "processing_category_rejected"
                    rospy.logwarn(
                        "PRODUCTION_CATEGORY_REJECTED category=%s "
                        "route_point=%d aligned=%s range_residual=%s",
                        category.encode("utf-8"), point_number,
                        observation["aligned"],
                        str(observation.get("range_residual_m")))
                self.target_scan_events.append(event)
                self.save_observation_summary()
                # Alignment changes yaw.  Resume the scan from the current
                # heading instead of returning to the capture yaw so the
                # remaining revolution covers fresh wall angles.
                return True

            self._ocr_turn_stop_flag = False
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
                "served_wall_points=%d",
                point_number, turn_progress, len(self.served_wall_points))
            return None
        finally:
            if self.use_ros_camera_for_ocr:
                self.stop_ros_camera_streaming(required=not rospy.is_shutdown())

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
                    self.log_small_ocr_candidate(
                        response, label, self.ocr_scan_candidate_confidence)
                    if is_navigation_ocr_candidate(
                            response, self.ocr_scan_candidate_confidence,
                            self.ocr_candidate_min_bbox_area_px):
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
                        if self._ocr_turn_stop_flag:
                            self.stop_motion()
                            self.wait_for_chassis_stop(
                                label + " required categories found")
                            return response, progress
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
                        self.log_small_ocr_candidate(
                            response, label,
                            self.ocr_scan_candidate_confidence)
                        if is_navigation_ocr_candidate(
                                response,
                                self.ocr_scan_candidate_confidence,
                                self.ocr_candidate_min_bbox_area_px):
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
                            if self._ocr_turn_stop_flag:
                                return response, progress
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
                                self.navigation_ocr_candidate_confidence,
                                self.ocr_candidate_min_bbox_area_px):
                            self.log_small_ocr_candidate(
                                response, label,
                                self.navigation_ocr_candidate_confidence)
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
        """Continuously OCR-align, then range and match after a full stop."""
        if observation_label is None:
            observation_label = "point_%03d" % route_point_number
        self.stop_motion()
        self.wait_for_chassis_stop(observation_label + " alignment start")
        previous_error = None
        previous_capture_time = None
        alignment_speed = None
        divergence_count = 0
        best_detection = None
        best_path = ""
        aligned = False
        image_width = self.camera_width
        attempt_image_paths = []

        for attempt in range(1, self.ocr_alignment_attempts + 1):
            self.require_safe()
            if alignment_speed is None:
                response = self.capture_ocr(observation_label, attempt)
            else:
                response = self.capture_ocr_while_turning(
                    alignment_speed, observation_label, attempt)
            # Python 2.7 on the vehicle has no time.monotonic().  The PD
            # derivative only needs elapsed time between adjacent parked
            # captures; ROS wall time is sufficient and Python 2-compatible.
            capture_time = time.time()
            image_path = response["image_path"]
            attempt_image_paths.append(image_path)
            image_width = int(response["width"])
            detection = response.get("detection")
            if detection is None:
                # We cannot steer safely without a current text box.  Return
                # to a stationary capture before the next attempt.
                self.stop_motion()
                self.wait_for_chassis_stop(
                    observation_label + " OCR empty")
                alignment_speed = None
                rospy.logwarn(
                    "PRODUCTION_OCR_EMPTY point=%d attempt=%d/%d",
                    route_point_number, attempt,
                    self.ocr_alignment_attempts)
                continue
            best_detection = detection
            best_path = image_path
            error = horizontal_pixel_error(detection, image_width)
            alignment_tolerance_px = self.ocr_alignment_tolerance_px
            if attempt > 5:
                alignment_tolerance_px += (
                    self.ocr_alignment_retry_tolerance_increment_px)
            rospy.loginfo(
                "PRODUCTION_OCR_BOX point=%d attempt=%d text=%s "
                "confidence=%.1f horizontal_error_px=%.1f "
                "tolerance_px=%.1f",
                route_point_number, attempt,
                json.dumps(detection["text"], ensure_ascii=True),
                detection["confidence"], error, alignment_tolerance_px)
            if abs(error) <= alignment_tolerance_px:
                aligned = True
                self.stop_motion()
                self.wait_for_chassis_stop(
                    observation_label + " OCR aligned")
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
            alignment_speed = alignment_angular_speed(
                error, self.ocr_alignment_kp, self.ocr_alignment_kd,
                self.ocr_alignment_max_speed, self.camera_mirror,
                previous_error, elapsed)
            if abs(alignment_speed) < self.ocr_alignment_min_speed:
                alignment_speed = self.ocr_alignment_min_speed * (
                    1.0 if alignment_speed >= 0.0 else -1.0)
            rospy.loginfo(
                "PRODUCTION_OCR_ALIGNMENT_CONTINUOUS point=%d "
                "attempt=%d speed=%.3f error_px=%.1f",
                route_point_number, attempt, alignment_speed, error)
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
            self.stop_motion()
            self.wait_for_chassis_stop(
                observation_label + " OCR alignment incomplete")
            rospy.logwarn(
                "PRODUCTION_OCR_NOT_ALIGNED point=%d text=%s",
                route_point_number,
                json.dumps(observation["text"], ensure_ascii=True))
            return observation

        self.wait_for_chassis_stop(
            observation_label + " before lidar")
        ocr_aligned_pose = self.current_map_pose(
            observation_label + " OCR aligned pose")
        observation["ocr_aligned_pose_map"] = list(ocr_aligned_pose)
        rospy.loginfo(
            "PRODUCTION_OCR_ALIGNED_POSE point=%d "
            "pose=(%.3f,%.3f,%.3f)",
            route_point_number, ocr_aligned_pose[0], ocr_aligned_pose[1],
            ocr_aligned_pose[2])
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
        # A fresh scan stamp may briefly outrun the TF clock (observed ~9 ms
        # jitter on the real vehicle).  Retry that transient extrapolation
        # instead of aborting; any other TF failure stays fatal.
        retry_deadline = time.time() + self.tf_lookup_retry_seconds
        while True:
            try:
                translation, rotation = self.tf_listener.lookupTransform(
                    "map", frame, scan.header.stamp)
                yaw = tf.transformations.euler_from_quaternion(rotation)[2]
                return translation[0], translation[1], yaw
            except tf.ExtrapolationException:
                if time.time() >= retry_deadline:
                    raise MissionAbort(
                        "map pose for lidar frame %s at scan stamp "
                        "unavailable after %.1f s: %s" %
                        (frame, self.tf_lookup_retry_seconds,
                         "TF still extrapolating into the future"))
                time.sleep(0.01)
            except tf.Exception as exc:
                raise MissionAbort(
                    "map pose for lidar frame %s at scan stamp unavailable: %s" %
                    (frame, exc))

    def save_observation_summary(self):
        if self.run_directory is None:
            return
        payload = {
            "route": self.production_route_numbers,
            "route_groups": getattr(self, "production_route_groups", []),
            "grouped_point_states": dict(
                getattr(self, "grouped_point_states", {})),
            "target_legs": self.production_navigation_legs,
            "fallback_route": getattr(
                self, "fallback_production_route_numbers", []),
            "fallback_target_legs": getattr(
                self, "fallback_navigation_legs", []),
            "target_guard_points": dict(
                (str(number), sorted(points))
                for number, points in self.target_guard_points.items()),
            "target_guard_events": self.target_guard_events,
            "target_scan_events": self.target_scan_events,
            "observations": self.observations,
            "items": [self.expected_real_item_text,
                      self.expected_sim_item_text],
            "categories": {
                self.expected_real_item_text: self.expected_real_category,
                self.expected_sim_item_text: self.expected_sim_category,
            },
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
        self.switch_navigation_mode("point")

    def switch_to_destination_mode(self):
        """Use the tighter CymPlanner profile only for the 441 approach."""
        self.switch_navigation_mode("destination")

    def switch_navigation_mode(self, mode):
        """Switch the CymPlanner parameter set at runtime.

        Supports "point" and "sprint" (the national 70->288 acceleration
        leg).  The latched command is repeated so delivery is observable and
        robust to a connection that completed at the edge of the wait loop.
        """
        if mode not in ("point", "destination", "sprint", "transverse"):
            raise TaskDefinitionError(
                "unsupported navigation mode %r" % mode)
        self.publish_state("SET_%s_NAVIGATION_MODE" % mode.upper())
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
        for _index in range(3):
            self.navigation_mode_pub.publish(String(data=mode))
            rospy.sleep(0.1)
        rospy.loginfo(
            "PRODUCTION_TASK navigation mode switched to %s", mode)

    def set_local_costmap_inflation_radius(self, radius, stage):
        """Apply and verify one local inflation radius via dynamic reconfigure."""
        if not getattr(self, "local_costmap_layer_control_enabled", False):
            raise MissionAbort(
                "local costmap layer control is required for %s" % stage)
        radius = float(radius)
        try:
            client = DynamicReconfigureClient(
                self.local_costmap_inflation_layer,
                timeout=self.local_costmap_reconfigure_timeout)
            configuration = client.update_configuration({
                "inflation_radius": radius,
            })
        except Exception as exc:
            raise MissionAbort(
                "cannot set local inflation radius %.3f at %s: %s" %
                (radius, stage, exc))
        applied = float(configuration["inflation_radius"])
        if abs(applied - radius) > 1e-6:
            raise MissionAbort(
                "local inflation radius %.3f did not apply at %s; got %.3f" %
                (radius, stage, applied))
        rospy.loginfo(
            "PRODUCTION_COSTMAP stage=%s local_inflation_radius=%.3f",
            stage, applied)
        return applied

    def set_global_costmap_inflation_radius(self, radius, stage):
        """Apply and verify the phase-specific global inflation radius."""
        if not getattr(self, "local_costmap_layer_control_enabled", False):
            raise MissionAbort(
                "global costmap inflation control is required for %s" % stage)
        radius = float(radius)
        try:
            client = DynamicReconfigureClient(
                self.global_costmap_inflation_layer,
                timeout=self.local_costmap_reconfigure_timeout)
            configuration = client.update_configuration({
                "inflation_radius": radius,
            })
        except Exception as exc:
            raise MissionAbort(
                "cannot set global inflation radius %.3f at %s: %s" %
                (radius, stage, exc))
        applied = float(configuration["inflation_radius"])
        if abs(applied - radius) > 1e-6:
            raise MissionAbort(
                "global inflation radius %.3f did not apply at %s; got %.3f" %
                (radius, stage, applied))
        rospy.loginfo(
            "PRODUCTION_COSTMAP stage=%s global_inflation_radius=%.3f",
            stage, applied)
        return applied

    def enter_processing_parking_profile(self):
        """Reduce local/global inflation for wall parking while keeping point mode."""
        if (self._processing_parking_original_inflation_radius_m is not None or
                self._processing_parking_original_global_inflation_radius_m is not None):
            raise MissionAbort("processing parking profile is already active")
        if not getattr(self, "local_costmap_layer_control_enabled", False):
            raise MissionAbort(
                "processing parking profile requires local costmap control")
        try:
            local_client = DynamicReconfigureClient(
                self.local_costmap_inflation_layer,
                timeout=self.local_costmap_reconfigure_timeout)
            global_client = DynamicReconfigureClient(
                self.global_costmap_inflation_layer,
                timeout=self.local_costmap_reconfigure_timeout)
            local_configuration = local_client.get_configuration()
            global_configuration = global_client.get_configuration()
            original_radius = float(local_configuration["inflation_radius"])
            original_global_radius = float(
                global_configuration["inflation_radius"])
        except Exception as exc:
            raise MissionAbort(
                "cannot read local/global inflation radii before processing "
                "parking: %s" % exc)
        self._processing_parking_original_inflation_radius_m = original_radius
        self._processing_parking_original_global_inflation_radius_m = (
            original_global_radius)
        try:
            self.set_local_costmap_inflation_radius(
                self.processing_parking_inflation_radius_m,
                "processing_parking_enter")
            self.set_global_costmap_inflation_radius(
                self.processing_parking_inflation_radius_m,
                "processing_parking_enter")
            self.switch_to_point_mode()
            rospy.loginfo(
                "PRODUCTION_PROCESSING_PROFILE entered "
                "local_inflation=%.3f global_inflation=%.3f "
                "navigation_mode=point",
                self.processing_parking_inflation_radius_m,
                self.processing_parking_inflation_radius_m)
        except Exception:
            self.exit_processing_parking_profile()
            raise

    def exit_processing_parking_profile(self):
        """Restore local/global inflation and point-mode navigation."""
        original_radius = (
            self._processing_parking_original_inflation_radius_m)
        original_global_radius = (
            self._processing_parking_original_global_inflation_radius_m)
        if original_radius is None and original_global_radius is None:
            return
        if original_radius is None or original_global_radius is None:
            raise MissionAbort(
                "processing parking profile restore state is incomplete")
        try:
            self.set_local_costmap_inflation_radius(
                original_radius, "processing_parking_exit")
            self.set_global_costmap_inflation_radius(
                original_global_radius, "processing_parking_exit")
            self.switch_to_point_mode()
            rospy.loginfo(
                "PRODUCTION_PROCESSING_PROFILE exited "
                "local_inflation=%.3f global_inflation=%.3f "
                "navigation_mode=point", original_radius,
                original_global_radius)
        finally:
            self._processing_parking_original_inflation_radius_m = None
            self._processing_parking_original_global_inflation_radius_m = None

    def set_local_costmap_dynamic_layers_enabled(self, enabled, stage):
        """Toggle local lidar obstacles and their inflation as one stage."""
        # Existing object.__new__-based unit tests do not run __init__.  Their
        # absent flag intentionally keeps this runtime-only control inactive.
        if not getattr(self, "local_costmap_layer_control_enabled", False):
            return
        layers = (
            ("obstacle_layer", self.local_costmap_obstacle_layer),
            ("inflation_layer", self.local_costmap_inflation_layer),
        )
        for layer_name, namespace in layers:
            try:
                client = DynamicReconfigureClient(
                    namespace,
                    timeout=self.local_costmap_reconfigure_timeout)
                configuration = client.update_configuration({
                    "enabled": bool(enabled),
                })
            except Exception as exc:
                raise MissionAbort(
                    "cannot set local %s enabled=%s at %s: %s" %
                    (layer_name, enabled, stage, exc))
            if bool(configuration["enabled"]) != bool(enabled):
                raise MissionAbort(
                    "local %s did not apply enabled=%s at %s" %
                    (layer_name, enabled, stage))
        rospy.loginfo(
            "PRODUCTION_COSTMAP stage=%s local_obstacle_and_inflation=%s",
            stage, "enabled" if enabled else "disabled")

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
            require_action_success=False, guard_callback=None,
            arrival_tolerance_override=None, goal_timeout=None):
        self.require_safe()
        arrival_tolerance = self.arrival_tolerance
        if arrival_tolerance_override is not None:
            arrival_tolerance = float(arrival_tolerance_override)
            if not is_finite(arrival_tolerance) or arrival_tolerance <= 0.0:
                raise TaskDefinitionError(
                    "arrival_tolerance_override must be finite and positive")
        if not require_plan:
            current = self.current_map_pose(label + " same-position check")
            if position_error(current, (x_value, y_value)) <= self.arrival_tolerance:
                self.rotate_in_place_to_yaw(
                    yaw, label + " shortest same-position rotation")
                return True
        if require_plan:
            plan_available = self.wait_for_plan(
                x_value, y_value, yaw, label,
                abort_on_failure=abort_on_navigation_failure)
            if not plan_available:
                return False

        navigation_timeout = (
            self.goal_timeout if goal_timeout is None else float(goal_timeout))
        retry_limit = self.navigation_arrival_retry_attempts
        for attempt in range(retry_limit + 1):
            goal = MoveBaseGoal()
            goal.target_pose = self.map_pose(x_value, y_value, yaw)
            rospy.loginfo(
                "PRODUCTION_TASK_GOAL label=%s target=(%.3f, %.3f) "
                "yaw=%.3f attempt=%d/%d",
                label, x_value, y_value, yaw, attempt + 1, retry_limit + 1)
            self.move_base.send_goal(goal)
            deadline = rospy.Time.now() + rospy.Duration(navigation_timeout)
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                self.require_safe()
                if guard_callback is not None and guard_callback():
                    # Do not act from a subscriber callback: cancel, zero
                    # speed, action acknowledgement and stopped-odom
                    # confirmation remain serialised in the supervisor.
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
                        label, navigation_timeout)
                    return False
                raise MissionAbort(
                    "%s timed out after %.1f s" %
                    (label, navigation_timeout))

            status = self.move_base.get_state()
            if status != GoalStatus.SUCCEEDED:
                self.stop_motion()
                if require_action_success:
                    raise MissionAbort(
                        "%s requires move_base success but ended with "
                        "status %d" % (label, status))
                pose = self.current_map_pose(label + " aborted arrival")
                arrival_error = position_error(pose, (x_value, y_value))
                if arrival_error <= arrival_tolerance:
                    rospy.logwarn(
                        "PRODUCTION_TASK_GOAL_ACCEPTED label=%s "
                        "move_base_status=%d arrival_error=%.3f m "
                        "limit=%.3f m",
                        label, status, arrival_error,
                        arrival_tolerance)
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
            if arrival_error <= arrival_tolerance:
                # move_base has completed the action; do not add a task-layer
                # zero-speed burst between navigation goals.
                rospy.loginfo(
                    "PRODUCTION_TASK_GOAL_REACHED label=%s error=%.3f m "
                    "without explicit task stop",
                    label, arrival_error)
                return True
            if attempt < retry_limit:
                rospy.logwarn(
                    "PRODUCTION_TASK_ARRIVAL_RETRY label=%s attempt=%d/%d "
                    "error=%.3f m limit=%.3f m; resending goal without "
                    "task abort",
                    label, attempt + 1, retry_limit + 1, arrival_error,
                    arrival_tolerance)
                continue
            if self.continue_on_arrival_error:
                rospy.logwarn(
                    "PRODUCTION_TASK_ARRIVAL_CONTINUE label=%s "
                    "error=%.3f m limit=%.3f m after %d attempts; "
                    "continuing without explicit task stop",
                    label, arrival_error, arrival_tolerance,
                    retry_limit + 1)
                return True
            self.stop_motion()
            if not abort_on_navigation_failure:
                rospy.logwarn(
                    "PRODUCTION_TASK_NAVIGATION_WARNING label=%s "
                    "arrival_error=%.3f m limit=%.3f m; continuing mission",
                    label, arrival_error, arrival_tolerance)
                return False
            raise MissionAbort(
                "%s stopped %.3f m from target (limit %.3f m)" %
                (label, arrival_error, arrival_tolerance))
        raise MissionAbort("%s navigation retry loop ended unexpectedly" % label)

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
            self, label, speed, stop_for_qr, qr_baseline,
            qr_accept=None, qr_observation_number=None):
        """In-place full revolution with an optional QR stop condition.

        Used as the QR scan fallback: after a face waits qr_search_timeout
        without an accepted code, the vehicle turns one full revolution;
        fresh codes are accepted while turning (stop_for_qr + qr_accept
        filter, used_qr_codes dedup preserved).
        """
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
                    if qr_accept is None or qr_accept(detected):
                        self.stop_motion()
                        rospy.loginfo(
                            "PRODUCTION_TASK_TURN_QR label=%s "
                            "progress=%.3f value=%s",
                            label, progress, self.log_safe_text(detected))
                        return detected
                    self._reject_qr_code(detected, qr_observation_number)

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
            self.stop_voice_listener()
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
