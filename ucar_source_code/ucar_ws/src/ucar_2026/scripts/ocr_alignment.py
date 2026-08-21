#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""独立 OCR 8 方位定点扫描 + 连续对齐验证节点。

该节点不导航、不搜索路线，只在当前车辆位置按固定方位数（默认 8 个）逐一
停靠扫描：转到每个方位停约 1 秒抓帧，发现候选后按省赛同源的 PD 角速度连续
对齐，直到 OCR 框进入固定像素容差或 15 秒超时。对齐完成后发布状态并通过
车端 TTS 播报“对齐完成”，用于单独验证扫描与对齐逻辑。
"""

from __future__ import print_function

import json
import math
import os
import select
import subprocess
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Image
from std_msgs.msg import String

from production_task_geometry import is_finite, positive_turn_increment
from production_task_perception import (
    alignment_angular_speed,
    horizontal_pixel_error,
    is_navigation_ocr_candidate,
    ocr_detection_bbox_area,
)


class OcrAlignmentError(RuntimeError):
    """A visible, task-level OCR alignment failure."""


class OcrAlignmentTimeout(OcrAlignmentError):
    """The continuous alignment budget expired while capturing a frame."""


class OcrAlignment(object):
    """Run only the continuous OCR alignment verification."""

    def __init__(self):
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
        self.rotation_control_rate = float(rospy.get_param(
            "~rotation_control_rate", 20.0))

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
            "~live_ppocr_path", "/home/ucar/ocr3/ppocr_trt/python/live_ppocr.py"))
        self.ppocr_root = str(rospy.get_param(
            "~ppocr_root", "/home/ucar/ocr3/ppocr_trt"))
        self.ocr_side = int(rospy.get_param("~ocr_side", 640))
        self.ocr_helper_ready_timeout = float(rospy.get_param(
            "~ocr_helper_ready_timeout", 90.0))
        self.ocr_capture_timeout = float(rospy.get_param(
            "~ocr_capture_timeout", 24.0))
        self.ocr_min_confidence = float(rospy.get_param(
            "~ocr_min_confidence", 0.30))

        self.alignment_timeout = float(rospy.get_param(
            "~alignment_timeout", 15.0))
        self.alignment_tolerance_px = float(rospy.get_param(
            "~alignment_tolerance_px", 30.0))
        self.alignment_kp = float(rospy.get_param(
            "~alignment_kp", 0.0025))
        self.alignment_kd = float(rospy.get_param(
            "~alignment_kd", 0.00035))
        self.alignment_max_speed = abs(float(rospy.get_param(
            "~alignment_max_speed", 0.22)))
        self.alignment_min_speed = abs(float(rospy.get_param(
            "~alignment_min_speed", 0.12)))

        self.ocr_scan_rotation_speed = abs(float(rospy.get_param(
            "~ocr_scan_rotation_speed", 1.5)))
        self.ocr_scan_positions = int(rospy.get_param(
            "~ocr_scan_positions", 8))
        self.ocr_scan_dwell_seconds = float(rospy.get_param(
            "~ocr_scan_dwell_seconds", 1.0))
        self.rotation_timeout_scale = float(rospy.get_param(
            "~rotation_timeout_scale", 3.5))
        self.ocr_scan_candidate_confidence = float(rospy.get_param(
            "~ocr_scan_candidate_confidence", 60.0))
        self.ocr_candidate_min_bbox_area_px = float(rospy.get_param(
            "~ocr_candidate_min_bbox_area_px", 1000.0))
        self.rotation_completion_tolerance = float(rospy.get_param(
            "~rotation_completion_tolerance_rad", 0.03))

        self.tts_enabled = bool(rospy.get_param("~tts_enabled", True))
        self.tts_python = str(rospy.get_param(
            "~tts_python", "/usr/bin/python3"))
        self.tts_helper_path = str(rospy.get_param(
            "~tts_helper_path", "/home/ucar/wake/tts_say.py"))
        self.tts_timeout = float(rospy.get_param("~tts_timeout", 15.0))

        self.result_directory = os.path.expanduser(str(rospy.get_param(
            "~result_directory", "~/.ros/ucar_2026_ocr_alignment")))

        for name, value in (
                ("odom_timeout", self.odom_timeout),
                ("stop_confirmation_timeout", self.stop_confirmation_timeout),
                ("camera_frame_timeout", self.camera_frame_timeout),
                ("ocr_helper_ready_timeout", self.ocr_helper_ready_timeout),
                ("ocr_capture_timeout", self.ocr_capture_timeout),
                ("alignment_timeout", self.alignment_timeout),
                ("alignment_tolerance_px", self.alignment_tolerance_px),
                ("alignment_kp", self.alignment_kp),
                ("alignment_max_speed", self.alignment_max_speed),
                ("alignment_min_speed", self.alignment_min_speed),
                ("ocr_scan_rotation_speed", self.ocr_scan_rotation_speed),
                ("ocr_scan_dwell_seconds", self.ocr_scan_dwell_seconds),
                ("rotation_timeout_scale", self.rotation_timeout_scale),
                ("ocr_scan_candidate_confidence",
                 self.ocr_scan_candidate_confidence),
                ("ocr_candidate_min_bbox_area_px",
                 self.ocr_candidate_min_bbox_area_px),
                ("rotation_completion_tolerance",
                 self.rotation_completion_tolerance),
                ("tts_timeout", self.tts_timeout)):
            if not is_finite(value) or value <= 0.0:
                raise OcrAlignmentError(
                    "%s must be finite and positive" % name)
        if (not is_finite(self.alignment_kd) or
                self.alignment_kd < 0.0):
            raise OcrAlignmentError(
                "alignment_kd must be finite and non-negative")
        if self.stopped_odom_samples <= 0:
            raise OcrAlignmentError("stopped_odom_samples must be positive")
        if self.ocr_scan_positions <= 0:
            raise OcrAlignmentError("ocr_scan_positions must be positive")
        if self.stopped_odom_speed_epsilon < 0.0:
            raise OcrAlignmentError(
                "stopped_odom_speed_epsilon must be non-negative")
        if not self.tts_enabled:
            raise OcrAlignmentError(
                "tts_enabled must be true for the alignment verification")

        self.cv_bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=10)
        self.state_pub = rospy.Publisher(
            "/ucar_2026/ocr_alignment/state", String,
            queue_size=1, latch=True)
        self.result_pub = rospy.Publisher(
            "/ucar_2026/ocr_alignment/result", String,
            queue_size=1, latch=True)

        self.lock = threading.RLock()
        self.latest_odom_receipt = None
        self.latest_odom_velocity = None
        self.latest_odom_finite = False
        self.latest_odom_yaw = None
        self.latest_camera_image = None
        self.latest_camera_receipt = None
        self.camera_sequence = 0
        self.critical_error = ""
        self.ocr_process = None
        self.ocr_log_handle = None
        self.capture_sequence = 0
        self.run_directory = None

        rospy.Subscriber(
            "/odom_raw", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber(
            self.camera_image_topic, Image, self.camera_image_cb,
            queue_size=1)
        rospy.Subscriber(
            "/rosout_agg", Log, self.rosout_cb, queue_size=100)
        rospy.on_shutdown(self.shutdown)
        self.publish_state("WAITING_START")

    def publish_state(self, state):
        self.state_pub.publish(String(data=str(state)))
        rospy.loginfo("OCR_ALIGNMENT_STATE %s", state)

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
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y +
                          orientation.z * orientation.z))
        with self.lock:
            self.latest_odom_receipt = rospy.Time.now()
            self.latest_odom_finite = finite
            self.latest_odom_yaw = yaw if finite else None
            self.latest_odom_velocity = (
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.angular.z,
            ) if finite else None
            if not finite:
                self.critical_error = "non-finite /odom_raw"

    def current_odom_yaw(self, context):
        self.require_safe()
        with self.lock:
            yaw = self.latest_odom_yaw
        if yaw is None:
            raise OcrAlignmentError(
                "%s: odom yaw unavailable" % context)
        return yaw

    def camera_image_cb(self, message):
        with self.lock:
            self.latest_camera_image = message
            self.latest_camera_receipt = rospy.Time.now()
            self.camera_sequence += 1

    def rosout_cb(self, message):
        text = message.msg.lower()
        if "crc16" in text and ("imu" in text or "ahrs" in text):
            rospy.logwarn_throttle(
                5.0, "OCR_ALIGNMENT_IMU_CRC_WARNING %s", message.msg)
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
            raise OcrAlignmentError(error)
        if receipt is None or not finite:
            raise OcrAlignmentError("/odom_raw is not ready with finite values")
        if (rospy.Time.now() - receipt).to_sec() > self.odom_timeout:
            raise OcrAlignmentError(
                "/odom_raw is stale by more than %.1f s" % self.odom_timeout)

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
                            "OCR_ALIGNMENT_STOP_CONFIRMED context=%s samples=%d",
                            context, stopped_samples)
                        return
                else:
                    stopped_samples = 0
            rospy.sleep(0.02)
        raise OcrAlignmentError(
            "%s did not confirm stopped odom within %.1f s" %
            (context, self.stop_confirmation_timeout))

    def stop_motion(self):
        zero = Twist()
        for _index in range(6):
            self.cmd_vel_pub.publish(zero)
            rospy.sleep(0.03)

    def prepare_result_directory(self):
        run_name = time.strftime("run_%Y%m%d_%H%M%S")
        self.run_directory = os.path.join(self.result_directory, run_name)
        os.makedirs(self.run_directory)
        rospy.loginfo("OCR_ALIGNMENT_RESULT_DIRECTORY %s", self.run_directory)

    def read_ocr_message(self, timeout, context):
        deadline = time.time() + float(timeout)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.require_safe()
            if self.ocr_process is None:
                raise OcrAlignmentError("%s: OCR helper is not running" % context)
            if self.ocr_process.poll() is not None:
                raise OcrAlignmentError(
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
                    "OCR_ALIGNMENT_NON_JSON %s", self.log_safe_text(raw_line))
        raise OcrAlignmentError(
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
            raise OcrAlignmentError(
                "live_ppocr helper did not report ready: %s" % message)
        rospy.loginfo(
            "OCR_ALIGNMENT_OCR_READY mode=%s cv2=%s candidates=%s",
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
        raise OcrAlignmentError(
            "%s did not receive a fresh camera frame" % context)

    def capture_ocr(self, capture_label, attempt):
        if self.ocr_process is None or self.ocr_process.poll() is not None:
            raise OcrAlignmentError("live_ppocr helper is not running")
        with self.lock:
            baseline_sequence = self.camera_sequence
        self.wait_for_fresh_camera_frame(baseline_sequence, capture_label)
        with self.lock:
            message = self.latest_camera_image
        if message is None:
            raise OcrAlignmentError("camera frame disappeared at %s" % capture_label)
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
            raise OcrAlignmentError(
                "cannot convert ROS camera frame: %s" % exc)
        if not cv2.imwrite(image_path, frame):
            raise OcrAlignmentError("cannot save camera frame %s" % image_path)
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
            raise OcrAlignmentError(
                "cannot command live_ppocr helper: %s" % exc)
        response = self.read_ocr_message(
            self.ocr_capture_timeout, "OCR capture %s" % capture_label)
        if not response.get("ok"):
            raise OcrAlignmentError(
                "live_ppocr capture failed for %s: %s" %
                (capture_label, response.get("error", response)))
        return response

    def start_async_capture(self, capture_label, attempt):
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

    def finish_async_capture(self, task):
        task["thread"].join()
        if task["error"] is not None:
            raise task["error"]
        return task["response"]

    def cleanup_async_capture(self, task):
        if task is None:
            return
        task["done"].wait(self.ocr_capture_timeout + 2.0)
        task["thread"].join()

    def capture_ocr_while_turning(
            self, signed_speed, capture_label, attempt, deadline):
        speed = float(signed_speed)
        if speed == 0.0:
            raise OcrAlignmentError("continuous OCR alignment has zero speed")
        direction = 1.0 if speed > 0.0 else -1.0
        if abs(speed) < self.alignment_min_speed:
            speed = self.alignment_min_speed * direction
        command = Twist()
        command.angular.z = speed
        task = None
        completed = False
        try:
            self.require_safe()
            self.cmd_vel_pub.publish(command)
            task = self.start_async_capture(
                "%s_moving" % capture_label, attempt)
            capture_deadline = min(
                time.time() + self.ocr_capture_timeout + 2.0, deadline)
            rate = rospy.Rate(self.rotation_control_rate)
            while not task["done"].is_set():
                self.require_safe()
                if time.time() >= capture_deadline:
                    raise OcrAlignmentTimeout(
                        "OCR alignment time budget expired")
                self.cmd_vel_pub.publish(command)
                rate.sleep()
            response = self.finish_async_capture(task)
            completed = True
            return response
        finally:
            if not completed:
                self.stop_motion()
                self.cleanup_async_capture(task)

    def align(self, initial_response):
        self.stop_motion()
        self.wait_for_chassis_stop("OCR alignment start")
        start_time = time.time()
        deadline = start_time + self.alignment_timeout
        previous_error = None
        previous_capture_time = None
        alignment_speed = None
        divergence_count = 0
        response = initial_response
        attempt = 0

        while not rospy.is_shutdown() and time.time() < deadline:
            self.require_safe()
            attempt += 1
            capture_time = time.time()
            detection = response.get("detection")
            if detection is None:
                rospy.logwarn(
                    "OCR_ALIGNMENT_EMPTY attempt=%d elapsed=%.1f/%.1f",
                    attempt, capture_time - start_time, self.alignment_timeout)
                if alignment_speed is None:
                    response = self.capture_ocr(
                        "OCR_ALIGNMENT_INITIAL", attempt + 1)
                else:
                    try:
                        response = self.capture_ocr_while_turning(
                            alignment_speed, "OCR_ALIGNMENT", attempt + 1,
                            deadline)
                    except OcrAlignmentTimeout:
                        break
                continue

            error = horizontal_pixel_error(detection, int(response["width"]))
            rospy.loginfo(
                "OCR_ALIGNMENT_BOX attempt=%d error_px=%.1f tolerance_px=%.1f "
                "text=%s confidence=%.1f elapsed=%.1f/%.1f",
                attempt, error, self.alignment_tolerance_px,
                self.log_safe_text(detection["text"]),
                detection["confidence"], capture_time - start_time,
                self.alignment_timeout)
            if abs(error) <= self.alignment_tolerance_px:
                self.stop_motion()
                self.wait_for_chassis_stop("OCR alignment complete")
                return response

            if (previous_error is not None and
                    abs(error) > abs(previous_error) * 1.35):
                divergence_count += 1
            else:
                divergence_count = 0
            if divergence_count >= 2:
                rospy.logwarn(
                    "OCR_ALIGNMENT_DIVERGED attempt=%d; reset PD derivative",
                    attempt)
                previous_error = None
                previous_capture_time = None
                divergence_count = 0

            elapsed = (
                capture_time - previous_capture_time
                if previous_capture_time is not None else None)
            alignment_speed = alignment_angular_speed(
                error, self.alignment_kp, self.alignment_kd,
                self.alignment_max_speed, self.camera_mirror,
                previous_error, elapsed)
            if abs(alignment_speed) < self.alignment_min_speed:
                alignment_speed = self.alignment_min_speed * (
                    1.0 if alignment_speed >= 0.0 else -1.0)
            previous_error = error
            previous_capture_time = capture_time
            try:
                response = self.capture_ocr_while_turning(
                    alignment_speed, "OCR_ALIGNMENT", attempt + 1,
                    deadline)
            except OcrAlignmentTimeout:
                break

        self.stop_motion()
        self.wait_for_chassis_stop("OCR alignment timeout")
        return None

    def scan_positions(self):
        """Scan a fixed number of headings, aligning any strong candidate.

        The scan keeps turning in one direction and uses accumulated actual
        turn progress to decide when a heading is reached.  At each heading
        the chassis stops and dwells for ``ocr_scan_dwell_seconds`` while one
        OCR frame is captured.  A strong candidate triggers the continuous
        alignment; an alignment failure continues with the remaining headings.
        """
        self.stop_motion()
        self.wait_for_chassis_stop("OCR position scan start")
        progress = 0.0
        angle_step = 2.0 * math.pi / self.ocr_scan_positions
        timeout = (2.0 * math.pi / self.ocr_scan_rotation_speed *
                   self.rotation_timeout_scale +
                   self.ocr_scan_positions * self.ocr_scan_dwell_seconds + 2.0)
        deadline = time.time() + timeout
        rate = rospy.Rate(self.rotation_control_rate)
        rospy.loginfo(
            "OCR_ALIGNMENT_POSITIONS_START positions=%d speed=%.3f "
            "dwell=%.1f timeout=%.1f",
            self.ocr_scan_positions, self.ocr_scan_rotation_speed,
            self.ocr_scan_dwell_seconds, timeout)
        try:
            for position_index in range(1, self.ocr_scan_positions + 1):
                turn_target = position_index * angle_step
                previous_yaw = self.current_odom_yaw(
                    "position %d start" % position_index)
                while not rospy.is_shutdown() and time.time() < deadline:
                    self.require_safe()
                    current_yaw = self.current_odom_yaw("position scan")
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
                if time.time() >= deadline:
                    raise OcrAlignmentTimeout(
                        "position scan timed out at %d/%d after %.1f s" %
                        (position_index, self.ocr_scan_positions, timeout))
                self.stop_motion()
                self.wait_for_chassis_stop(
                    "position %d" % position_index)
                dwell_start = time.time()
                response = self.capture_ocr(
                    "OCR_ALIGNMENT_POS_%02d" % position_index,
                    position_index)
                remaining = self.ocr_scan_dwell_seconds - (
                    time.time() - dwell_start)
                if remaining > 0.0:
                    rospy.sleep(remaining)
                detection = response.get("detection")
                if detection is not None:
                    rospy.loginfo(
                        "OCR_ALIGNMENT_POSITION_CANDIDATE position=%d/%d "
                        "text=%s confidence=%.1f",
                        position_index, self.ocr_scan_positions,
                        self.log_safe_text(detection.get("text", "")),
                        detection.get("confidence", -1.0))
                if is_navigation_ocr_candidate(
                        response, self.ocr_scan_candidate_confidence,
                        self.ocr_candidate_min_bbox_area_px):
                    aligned_response = self.align(response)
                    if aligned_response is not None:
                        return aligned_response
                    rospy.logwarn(
                        "OCR_ALIGNMENT_POSITION_ALIGN_FAILED position=%d/%d; "
                        "continuing remaining headings",
                        position_index, self.ocr_scan_positions)
            rospy.loginfo(
                "OCR_ALIGNMENT_POSITIONS_COMPLETE positions=%d progress=%.3f",
                self.ocr_scan_positions, progress)
            return None
        finally:
            self.stop_motion()

    def speak_wait(self, text):
        payload = text
        if not isinstance(payload, str):
            payload = payload.encode("utf-8")
        rospy.loginfo("OCR_ALIGNMENT_TTS text=%s", self.log_safe_text(text))
        process = subprocess.Popen([
            self.tts_python,
            self.tts_helper_path,
            payload,
        ])
        deadline = time.time() + self.tts_timeout
        while process.poll() is None and time.time() < deadline:
            if rospy.is_shutdown():
                process.terminate()
                raise OcrAlignmentError("ROS shutdown interrupted TTS")
            time.sleep(0.1)
        if process.poll() is None:
            process.terminate()
            raise OcrAlignmentError(
                "TTS did not finish within %.1f s" % self.tts_timeout)
        if process.returncode != 0:
            raise OcrAlignmentError(
                "TTS exited with code %d" % process.returncode)

    def publish_result(self, success, reason, response=None):
        payload = {
            "success": bool(success),
            "reason": str(reason),
            "alignment_timeout": self.alignment_timeout,
            "alignment_tolerance_px": self.alignment_tolerance_px,
            "image_path": response.get("image_path") if response else "",
        }
        self.result_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=True, sort_keys=True)))
        rospy.loginfo("OCR_ALIGNMENT_RESULT %s", self.log_safe_text(payload))

    def run(self):
        self.prepare_result_directory()
        self.publish_state("OCR_HELPER_START")
        try:
            self.start_ocr()
            self.publish_state("OCR_ALIGNMENT_RUNNING")
            response = self.scan_positions()
            if response is None:
                self.publish_state("ALIGNMENT_TIMEOUT")
                self.publish_result(
                    False, "alignment timeout after all scan positions")
                return
            self.publish_state("ALIGNMENT_COMPLETED")
            self.publish_result(True, "alignment completed", response)
            self.speak_wait(u"对齐完成")
        finally:
            self.stop_motion()
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
            rospy.loginfo("OCR_ALIGNMENT_OCR_CLOSED")

    def shutdown(self):
        self.stop_motion()
        self.stop_ocr()


def main():
    rospy.init_node("ocr_alignment")
    task = OcrAlignment()
    try:
        task.run()
    except OcrAlignmentError as exc:
        task.stop_motion()
        task.publish_state("ABORTED")
        task.publish_result(False, str(exc))
        rospy.logfatal("OCR_ALIGNMENT_ABORTED %s", exc)
        raise


if __name__ == "__main__":
    main()
