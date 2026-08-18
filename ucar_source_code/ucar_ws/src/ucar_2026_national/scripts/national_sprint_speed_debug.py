#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""独立调试国赛 70 -> 坡顶冲刺速度。

该节点假定小车已经物理放在国赛点 70，启动时只做以下动作：

1. 等待 /odom_raw、odom -> base_link、map -> base_link 均有效；
2. 校验当前定位仍在点 70；
3. 将 CymPlanner 切到 sprint 模式；
4. 通过 move_base 导航到 67 与 290 的中点（坡顶）；
5. 停车、恢复 point 模式并打印本次请求速度统计。

节点不启动完整国赛任务，不启动相机、二维码、OCR 或巡线节点。
"""

from __future__ import print_function

import json
import math
import os
import sys
import threading

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from std_msgs.msg import String


class SprintSpeedDebugAbort(RuntimeError):
    """调试流程必须停止时使用的显式错误。"""


def finite(value):
    return not math.isnan(value) and not math.isinf(value)


def load_numbered_points(path):
    with open(path, "r") as handle:
        document = json.load(handle)
    points = {}
    for item in document["points"]:
        number = int(item["number"])
        x_value = float(item["x_m"])
        y_value = float(item["y_m"])
        if not finite(x_value) or not finite(y_value):
            raise SprintSpeedDebugAbort(
                "grid point %d has a non-finite coordinate" % number)
        points[number] = (x_value, y_value)
    return points


def require_point(points, number):
    if number not in points:
        raise SprintSpeedDebugAbort(
            "grid does not contain required point %d" % number)
    return points[number]


def midpoint(first, second):
    return ((first[0] + second[0]) / 2.0,
            (first[1] + second[1]) / 2.0)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose_from_xy_yaw(x_value, y_value, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = float(x_value)
    pose.pose.position.y = float(y_value)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class NationalSprintSpeedDebug(object):
    def __init__(self):
        self.grid_path = os.path.expanduser(
            rospy.get_param("~grid_path"))
        self.start_point_number = int(
            rospy.get_param("~start_point_number", 70))
        self.slope_left_point_number = int(
            rospy.get_param("~slope_left_point_number", 67))
        self.slope_right_point_number = int(
            rospy.get_param("~slope_right_point_number", 290))
        self.start_yaw = math.radians(float(
            rospy.get_param("~start_yaw_deg", 180.0)))
        self.goal_yaw = math.radians(float(
            rospy.get_param("~goal_yaw_deg", 180.0)))
        self.start_position_tolerance = float(rospy.get_param(
            "~start_position_tolerance", 0.15))
        self.start_yaw_tolerance = math.radians(float(rospy.get_param(
            "~start_yaw_tolerance_deg", 12.0)))
        self.goal_position_tolerance = float(rospy.get_param(
            "~goal_position_tolerance", 0.15))
        self.goal_yaw_tolerance = math.radians(float(rospy.get_param(
            "~goal_yaw_tolerance_deg", 8.0)))
        self.safe_start_timeout = float(rospy.get_param(
            "~safe_start_timeout", 90.0))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 4.0))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 4.0))
        self.minimum_finite_odom_samples = int(rospy.get_param(
            "~minimum_finite_odom_samples", 3))
        self.move_base_ready_timeout = float(rospy.get_param(
            "~move_base_ready_timeout", 60.0))
        self.plan_timeout = float(rospy.get_param("~plan_timeout", 30.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 120.0))
        self.navigation_mode_connect_timeout = float(rospy.get_param(
            "~navigation_mode_connect_timeout", 10.0))
        self.cmd_vel_topic = str(rospy.get_param(
            "~cmd_vel_topic", "/cmd_vel"))

        points = load_numbered_points(self.grid_path)
        self.start_xy = require_point(points, self.start_point_number)
        slope_left = require_point(points, self.slope_left_point_number)
        slope_right = require_point(points, self.slope_right_point_number)
        self.goal_xy = midpoint(slope_left, slope_right)

        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.make_plan = rospy.ServiceProxy("move_base/make_plan", GetPlan)
        self.mode_pub = rospy.Publisher(
            "/ucar/navigation_mode", String, queue_size=1, latch=True)
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=10)

        self.lock = threading.RLock()
        self.latest_odom_receipt = None
        self.finite_odom_samples = 0
        self.invalid_odom_reason = None
        self.max_command_linear_x = 0.0
        self.max_command_speed = 0.0
        self.command_sample_count = 0

        rospy.Subscriber(
            "/odom_raw", Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(
            self.cmd_vel_topic, Twist, self.command_callback, queue_size=10)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "NATIONAL_SPRINT_DEBUG points: start=%d (%.3f, %.3f), "
            "slope_top=(%.3f, %.3f), yaw=%.1f deg",
            self.start_point_number, self.start_xy[0], self.start_xy[1],
            self.goal_xy[0], self.goal_xy[1], math.degrees(self.goal_yaw))

    def odom_callback(self, message):
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        )
        if not all(finite(float(value)) for value in values):
            with self.lock:
                if self.invalid_odom_reason is None:
                    self.invalid_odom_reason = "/odom_raw contains NaN or Inf"
                    rospy.logerr(
                        "NATIONAL_SPRINT_DEBUG unsafe odom: %s",
                        self.invalid_odom_reason)
            self.publish_zero()
            return
        with self.lock:
            self.latest_odom_receipt = rospy.Time.now()
            self.finite_odom_samples += 1

    def command_callback(self, message):
        linear_x = abs(float(message.linear.x))
        speed = math.sqrt(
            float(message.linear.x) ** 2 + float(message.linear.y) ** 2)
        with self.lock:
            self.max_command_linear_x = max(
                self.max_command_linear_x, linear_x)
            self.max_command_speed = max(self.max_command_speed, speed)
            self.command_sample_count += 1

    def publish_zero(self):
        self.cmd_vel_pub.publish(Twist())

    def stop_motion(self):
        zero = Twist()
        for _index in range(8):
            self.cmd_vel_pub.publish(zero)
            rospy.sleep(0.05)

    def shutdown(self):
        self.stop_motion()
        self.publish_mode("point", require_connection=False)

    def safety_failure_reason(self):
        with self.lock:
            invalid_reason = self.invalid_odom_reason
            odom_receipt = self.latest_odom_receipt
            finite_samples = self.finite_odom_samples
        if invalid_reason is not None:
            return invalid_reason
        if odom_receipt is None:
            return "/odom_raw has not produced a finite sample"
        odom_age = (rospy.Time.now() - odom_receipt).to_sec()
        if odom_age > self.odom_timeout:
            return "/odom_raw is stale by %.3f s" % odom_age
        if finite_samples < self.minimum_finite_odom_samples:
            return "only %d/%d finite odom samples" % (
                finite_samples, self.minimum_finite_odom_samples)
        for target, source in (("odom", "base_link"),
                               ("map", "base_link")):
            try:
                latest = self.tf_listener.getLatestCommonTime(target, source)
                if latest.is_zero():
                    return "%s -> %s TF has zero timestamp" % (target, source)
                age = (rospy.Time.now() - latest).to_sec()
                if age > self.tf_timeout:
                    return "%s -> %s TF is stale by %.3f s" % (
                        target, source, age)
                translation, rotation = self.tf_listener.lookupTransform(
                    target, source, rospy.Time(0))
                if not all(finite(float(value))
                           for value in tuple(translation) + tuple(rotation)):
                    return "%s -> %s TF contains NaN or Inf" % (
                        target, source)
            except tf.Exception as exc:
                return "%s -> %s TF unavailable: %s" % (
                    target, source, exc)
        return None

    def wait_for_safe_state(self, context, timeout):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown():
            reason = self.safety_failure_reason()
            if reason is None:
                rospy.loginfo("NATIONAL_SPRINT_DEBUG safety passed: %s", context)
                return
            if self.invalid_odom_reason is not None:
                self.stop_motion()
                raise SprintSpeedDebugAbort(
                    "%s safety failed: %s" % (context, reason))
            if rospy.Time.now() >= deadline:
                self.stop_motion()
                raise SprintSpeedDebugAbort(
                    "%s safety timeout: %s" % (context, reason))
            rospy.sleep(0.1)
        raise SprintSpeedDebugAbort("ROS shutdown while checking %s" % context)

    def current_map_pose(self):
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
        except tf.Exception as exc:
            raise SprintSpeedDebugAbort(
                "cannot read map -> base_link pose: %s" % exc)
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        return translation[0], translation[1], yaw

    def check_start_pose(self):
        x_value, y_value, yaw = self.current_map_pose()
        distance = math.sqrt(
            (x_value - self.start_xy[0]) ** 2 +
            (y_value - self.start_xy[1]) ** 2)
        yaw_error = abs(normalize_angle(yaw - self.start_yaw))
        rospy.loginfo(
            "NATIONAL_SPRINT_DEBUG start pose: actual=(%.3f, %.3f, %.1f deg) "
            "expected=(%.3f, %.3f, %.1f deg) error=%.3f m/%.1f deg",
            x_value, y_value, math.degrees(yaw),
            self.start_xy[0], self.start_xy[1], math.degrees(self.start_yaw),
            distance, math.degrees(yaw_error))
        if distance > self.start_position_tolerance:
            raise SprintSpeedDebugAbort(
                "vehicle is not at sprint start point %d: %.3f m away" %
                (self.start_point_number, distance))
        if yaw_error > self.start_yaw_tolerance:
            raise SprintSpeedDebugAbort(
                "vehicle heading is %.1f deg away from sprint heading" %
                math.degrees(yaw_error))

    def publish_mode(self, mode, require_connection=True):
        deadline = (rospy.Time.now() +
                    rospy.Duration(self.navigation_mode_connect_timeout))
        while (require_connection and
               self.mode_pub.get_num_connections() <= 0 and
               not rospy.is_shutdown() and rospy.Time.now() < deadline):
            rospy.sleep(0.1)
        if require_connection and self.mode_pub.get_num_connections() <= 0:
            raise SprintSpeedDebugAbort(
                "CymPlanner is not connected to /ucar/navigation_mode")
        for _index in range(3):
            self.mode_pub.publish(String(data=mode))
            rospy.sleep(0.1)
        rospy.loginfo("NATIONAL_SPRINT_DEBUG navigation mode=%s", mode)

    def wait_for_plan(self):
        target = pose_from_xy_yaw(
            self.goal_xy[0], self.goal_xy[1], self.goal_yaw)
        deadline = rospy.Time.now() + rospy.Duration(self.plan_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.wait_for_safe_state("plan", 1.0)
            try:
                rospy.wait_for_service("move_base/make_plan", timeout=1.0)
                current = self.current_map_pose()
                start = pose_from_xy_yaw(current[0], current[1], current[2])
                response = self.make_plan(start, target, 0.0)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("NATIONAL_SPRINT_DEBUG plan service: %s", exc)
                continue
            if len(response.plan.poses) > 1:
                rospy.loginfo(
                    "NATIONAL_SPRINT_DEBUG global plan ready: %d poses",
                    len(response.plan.poses))
                return
            rospy.sleep(0.5)
        raise SprintSpeedDebugAbort(
            "no global plan to slope top within %.1f s" % self.plan_timeout)

    def send_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose = pose_from_xy_yaw(
            self.goal_xy[0], self.goal_xy[1], self.goal_yaw)
        self.move_base.send_goal(goal)
        deadline = rospy.Time.now() + rospy.Duration(self.goal_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.wait_for_safe_state("sprint motion", 1.0)
            if self.move_base.wait_for_result(rospy.Duration(0.1)):
                break
        if rospy.is_shutdown():
            raise SprintSpeedDebugAbort("ROS shutdown during sprint motion")
        if rospy.Time.now() >= deadline:
            self.move_base.cancel_goal()
            self.stop_motion()
            raise SprintSpeedDebugAbort(
                "sprint goal timed out after %.1f s" % self.goal_timeout)
        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            self.stop_motion()
            raise SprintSpeedDebugAbort(
                "sprint goal finished with action state %d" % state)

    def check_goal_pose(self):
        x_value, y_value, yaw = self.current_map_pose()
        distance = math.sqrt(
            (x_value - self.goal_xy[0]) ** 2 +
            (y_value - self.goal_xy[1]) ** 2)
        yaw_error = abs(normalize_angle(yaw - self.goal_yaw))
        rospy.loginfo(
            "NATIONAL_SPRINT_DEBUG goal pose: actual=(%.3f, %.3f, %.1f deg) "
            "error=%.3f m/%.1f deg",
            x_value, y_value, math.degrees(yaw),
            distance, math.degrees(yaw_error))
        if distance > self.goal_position_tolerance:
            raise SprintSpeedDebugAbort(
                "slope top arrival error %.3f m exceeds %.3f m" %
                (distance, self.goal_position_tolerance))
        if yaw_error > self.goal_yaw_tolerance:
            raise SprintSpeedDebugAbort(
                "slope top heading error %.1f deg exceeds %.1f deg" %
                (math.degrees(yaw_error),
                 math.degrees(self.goal_yaw_tolerance)))

    def run(self):
        self.wait_for_safe_state("startup", self.safe_start_timeout)
        self.check_start_pose()
        if not self.move_base.wait_for_server(
                rospy.Duration(self.move_base_ready_timeout)):
            raise SprintSpeedDebugAbort(
                "move_base action server not ready within %.1f s" %
                self.move_base_ready_timeout)
        self.publish_mode("sprint")
        self.wait_for_plan()
        rospy.loginfo(
            "NATIONAL_SPRINT_DEBUG starting 70 -> slope top; "
            "speed is controlled by mode3_sprint parameters")
        self.send_goal()
        self.stop_motion()
        self.check_goal_pose()
        self.publish_mode("point")
        with self.lock:
            max_linear_x = self.max_command_linear_x
            max_speed = self.max_command_speed
            command_samples = self.command_sample_count
        rospy.loginfo(
            "NATIONAL_SPRINT_DEBUG complete: requested_max_linear_x=%.3f "
            "requested_max_speed=%.3f command_samples=%d",
            max_linear_x, max_speed, command_samples)


def main():
    rospy.init_node("national_sprint_speed_debug")
    debug = NationalSprintSpeedDebug()
    try:
        debug.run()
    except SprintSpeedDebugAbort as exc:
        rospy.logerr("NATIONAL_SPRINT_DEBUG ABORTED: %s", exc)
        try:
            debug.move_base.cancel_all_goals()
        finally:
            debug.stop_motion()
        return 1
    except rospy.ROSInterruptException:
        debug.stop_motion()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
