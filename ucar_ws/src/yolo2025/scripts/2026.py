#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""2026 navigation startup task, scan forwarding, and post-goal QR sweep."""

import math
import sys

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Navigation2026(object):
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.startup_goal_enabled = rospy.get_param("~startup_goal_enabled", False)
        self.startup_goal_x = rospy.get_param("~startup_goal_x", -1.534)
        self.startup_goal_y = rospy.get_param("~startup_goal_y", 2.105)
        self.startup_goal_yaw = rospy.get_param("~startup_goal_yaw", -2.950)
        self.startup_goal_delay = rospy.get_param("~startup_goal_delay", 2.0)
        self.startup_goal_ready_timeout = rospy.get_param("~startup_goal_ready_timeout", 90.0)
        self.startup_pose_settle_seconds = float(
            rospy.get_param("~startup_pose_settle_seconds", 3.0))
        self.startup_pose_settle_distance = float(
            rospy.get_param("~startup_pose_settle_distance", 0.05))
        self.startup_plan_settle_samples = max(1, int(
            rospy.get_param("~startup_plan_settle_samples", 5)))

        self.qr_scan_enabled = rospy.get_param("~qr_scan_enabled", True)
        self.qr_scan_min_count = int(rospy.get_param("~qr_scan_min_count", 3))
        self.qr_scan_hold_sec = float(rospy.get_param("~qr_scan_hold_sec", 4.0))
        self.qr_scan_timeout = float(rospy.get_param("~qr_scan_timeout", 40.0))
        self.qr_heading_goal_timeout = float(
            rospy.get_param("~qr_heading_goal_timeout", 6.0))
        self.post_qr_goal_enabled = rospy.get_param("~post_qr_goal_enabled", True)
        self.post_qr_goal_x = float(rospy.get_param("~post_qr_goal_x", -1.737))
        self.post_qr_goal_y = float(rospy.get_param("~post_qr_goal_y", 1.003))
        self.post_qr_goal_yaw = float(rospy.get_param("~post_qr_goal_yaw", 3.140))
        self.second_goal_enabled = rospy.get_param("~second_goal_enabled", True)
        self.second_goal_x = float(rospy.get_param("~second_goal_x", -1.722))
        self.second_goal_y = float(rospy.get_param("~second_goal_y", -0.269))
        self.second_goal_yaw = float(rospy.get_param("~second_goal_yaw", -3.140))
        self.next_goal_enabled = rospy.get_param("~next_goal_enabled", True)
        self.next_goal_x = float(rospy.get_param("~next_goal_x", -2.265))
        self.next_goal_y = float(rospy.get_param("~next_goal_y", -0.001))
        self.next_goal_yaw = float(rospy.get_param("~next_goal_yaw", -1.557))
        self.cym_holonomic_mode_param = rospy.get_param(
            "~cym_holonomic_mode_param",
            "/move_base/cym_planner/CymPlanner/holonomic_mode")
        self.task_linear_speed = float(rospy.get_param("~task_linear_speed", 0.1))
        self.cym_task_max_vel_param = rospy.get_param(
            "~cym_task_max_vel_param",
            "/move_base/cym_planner/CymPlanner/task_max_vel")

        self.tf_listener = tf.TransformListener()
        self.make_plan = rospy.ServiceProxy("move_base/make_plan", GetPlan)

        self.scan_scale = 1.0
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.global_scan_pub = rospy.Publisher(
            "/scan_global_obstacles", LaserScan, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan_raw", LaserScan, self.scan_cb, queue_size=1)
        self.global_obstacle_filter_enabled = rospy.get_param(
            "~global_obstacle_filter_enabled", True)
        self.global_static_filter_radius = float(rospy.get_param(
            "~global_static_filter_radius", 0.22))
        self.global_static_filter_threshold = int(rospy.get_param(
            "~global_static_filter_threshold", 65))
        self.global_static_filter_mask = None
        self.global_static_filter_info = None
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)

        self.qr_result_sub = rospy.Subscriber("/qr_result", String, self.qr_result_cb, queue_size=10)
        self.qr_codes = set()
        self.qr_scan_active = False
        self.qr_scan_steps = []
        self.qr_scan_step_index = 0
        self.qr_scan_phase = None
        self.qr_scan_hold_timer = None
        self.qr_heading_goal_timer = None
        self.qr_scan_timeout_timer = None
        self.post_qr_start_timer = None

        if self.startup_goal_enabled:
            rospy.Timer(rospy.Duration(self.startup_goal_delay), self.startup_goal_cb, oneshot=True)

    def scan_cb(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min * self.scan_scale
        out.range_max = msg.range_max * self.scan_scale
        out.ranges = [distance * self.scan_scale for distance in msg.ranges]
        out.intensities = msg.intensities
        self.scan_pub.publish(out)
        self.global_scan_pub.publish(self.filtered_global_scan(out))

    @staticmethod
    def clone_scan(scan):
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.intensities = scan.intensities
        out.ranges = list(scan.ranges)
        return out

    def map_cb(self, msg):
        """Dilate static occupied cells once for O(1) per-laser filtering."""
        if not self.global_obstacle_filter_enabled:
            return

        info = msg.info
        if info.width <= 0 or info.height <= 0 or info.resolution <= 0.0:
            return

        radius_cells = int(math.ceil(self.global_static_filter_radius /
                                     info.resolution))
        offsets = []
        radius_squared = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_squared:
                    offsets.append((dx, dy))

        width = info.width
        height = info.height
        mask = bytearray(width * height)
        for index, cost in enumerate(msg.data):
            if cost < self.global_static_filter_threshold:
                continue
            cell_y = index // width
            cell_x = index - cell_y * width
            for dx, dy in offsets:
                x = cell_x + dx
                y = cell_y + dy
                if 0 <= x < width and 0 <= y < height:
                    mask[y * width + x] = 1

        self.global_static_filter_mask = mask
        self.global_static_filter_info = info
        rospy.loginfo("Global obstacle scan filter ready: %.2f m static-wall mask.",
                      self.global_static_filter_radius)

    def filtered_global_scan(self, scan):
        """Remove laser returns already represented by the static global map."""
        if not self.global_obstacle_filter_enabled:
            return scan

        if (self.global_static_filter_mask is None or
                self.global_static_filter_info is None):
            # Never mark the raw scan before the static mask is available:
            # those mapped-wall returns would survive in the global obstacle
            # layer and temporarily close the narrow doorway.
            filtered = self.clone_scan(scan)
            filtered.ranges = [float("inf")] * len(scan.ranges)
            return filtered

        try:
            translation, rotation = self.tf_listener.lookupTransform(
                "map", scan.header.frame_id, scan.header.stamp)
        except tf.Exception:
            return scan

        info = self.global_static_filter_info
        mask = self.global_static_filter_mask
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        filtered = self.clone_scan(scan)

        for index, distance in enumerate(filtered.ranges):
            if (distance != distance or math.isinf(distance) or
                    distance < scan.range_min or distance > scan.range_max):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            map_x = translation[0] + cos_yaw * local_x - sin_yaw * local_y
            map_y = translation[1] + sin_yaw * local_x + cos_yaw * local_y
            cell_x = int(math.floor(
                (map_x - info.origin.position.x) / info.resolution))
            cell_y = int(math.floor(
                (map_y - info.origin.position.y) / info.resolution))
            if (cell_x < 0 or cell_x >= info.width or
                    cell_y < 0 or cell_y >= info.height or
                    mask[cell_y * info.width + cell_x]):
                filtered.ranges[index] = float("inf")

        return filtered

    def startup_goal_cb(self, _event):
        if not self.move_base.wait_for_server(rospy.Duration(15.0)):
            rospy.logerr("move_base action server was not available within 15 seconds.")
            return

        self.restore_task_motion("startup goal")

        rospy.loginfo("Startup goal: frame=map x=%.3f y=%.3f yaw=%.3f rad (%.1f deg)",
                      self.startup_goal_x, self.startup_goal_y, self.startup_goal_yaw,
                      math.degrees(self.startup_goal_yaw))
        if not self.wait_for_ready_plan(self.startup_goal_x, self.startup_goal_y,
                                        self.startup_goal_yaw):
            rospy.logwarn("Startup goal skipped: no ready localization and global plan within %.1f s.",
                          self.startup_goal_ready_timeout)
            return

        rospy.loginfo("Startup goal is ready; sending it to move_base.")
        self.send_goal(self.startup_goal_x, self.startup_goal_y, self.startup_goal_yaw,
                       self.startup_goal_done_cb)

    @staticmethod
    def map_pose(x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def wait_for_ready_plan(self, goal_x, goal_y, goal_yaw):
        deadline = rospy.Time.now() + rospy.Duration(self.startup_goal_ready_timeout)
        goal_pose = self.map_pose(goal_x, goal_y, goal_yaw)
        stable_since = None
        previous_translation = None
        ready_plan_samples = 0
        rospy.loginfo("Waiting for localization and global plan for up to %.1f s.",
                      self.startup_goal_ready_timeout)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                rospy.wait_for_service("move_base/make_plan", timeout=1.0)
                translation, rotation = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                if previous_translation is None:
                    stable_since = rospy.Time.now()
                else:
                    pose_shift = math.hypot(translation[0] - previous_translation[0],
                                            translation[1] - previous_translation[1])
                    if pose_shift > self.startup_pose_settle_distance:
                        rospy.loginfo("Startup localization changed by %.3f m; waiting for it to settle.",
                                      pose_shift)
                        stable_since = rospy.Time.now()
                        ready_plan_samples = 0
                previous_translation = translation

                if (stable_since is None or
                        (rospy.Time.now() - stable_since).to_sec() <
                        self.startup_pose_settle_seconds):
                    ready_plan_samples = 0
                    rospy.sleep(1.0)
                    continue

                start_pose = PoseStamped()
                start_pose.header.frame_id = "map"
                start_pose.header.stamp = rospy.Time.now()
                start_pose.pose.position.x = translation[0]
                start_pose.pose.position.y = translation[1]
                start_pose.pose.position.z = translation[2]
                start_pose.pose.orientation.x = rotation[0]
                start_pose.pose.orientation.y = rotation[1]
                start_pose.pose.orientation.z = rotation[2]
                start_pose.pose.orientation.w = rotation[3]

                plan = self.make_plan(start_pose, goal_pose, 0.0)
                if len(plan.plan.poses) > 1:
                    ready_plan_samples += 1
                    rospy.loginfo("Stable localization and global plan ready with %d poses (%d/%d).",
                                  len(plan.plan.poses), ready_plan_samples,
                                  self.startup_plan_settle_samples)
                    if ready_plan_samples >= self.startup_plan_settle_samples:
                        return True
                else:
                    ready_plan_samples = 0
            except (rospy.ROSException, rospy.ServiceException, tf.Exception):
                pass
            rospy.sleep(1.0)
        return False

    def send_goal(self, x, y, yaw, done_cb=None):
        goal = MoveBaseGoal()
        goal.target_pose = self.map_pose(x, y, yaw)
        self.move_base.send_goal(goal, done_cb=done_cb)

    def set_holonomic_mode(self, enabled, stage):
        rospy.set_param(self.cym_holonomic_mode_param, bool(enabled))
        rospy.loginfo("CYM_PLANNER_MODE stage=%s mode=%s", stage,
                      "holonomic" if enabled else "normal")

    def set_task_linear_speed(self, speed, stage):
        rospy.set_param(self.cym_task_max_vel_param, max(0.0, float(speed)))
        rospy.loginfo("CYM_PLANNER_TASK_SPEED stage=%s linear_max=%.3f", stage, speed)

    @staticmethod
    def post_qr_status(message):
        """Report task progress without placing control after a ROS log call."""
        sys.stdout.write("[POST_QR] %s\n" % message)
        sys.stdout.flush()

    def restore_task_motion(self, stage):
        self.set_holonomic_mode(False, stage)
        self.set_task_linear_speed(0.0, stage)

    def startup_goal_done_cb(self, status, _result):
        if status != GoalStatus.SUCCEEDED:
            rospy.logwarn("Startup goal did not succeed (action status=%d); QR sweep is skipped.", status)
            return
        rospy.loginfo("Startup goal reached.")
        if self.qr_scan_enabled:
            self.begin_qr_scan()

    def begin_qr_scan(self):
        if self.qr_scan_active:
            return
        self.qr_codes = set()
        self.qr_scan_active = True
        # User-facing angles are clockwise from the start yaw=0.  ROS yaw is
        # counter-clockwise positive, so 90/180/270 degrees become -pi/2, pi,
        # and +pi/2.  Every heading is sent to move_base/CymPlanner.  The
        # sequence ends at clockwise 90 degrees, so the vehicle stays aligned
        # with i after scanning.
        self.qr_scan_steps = [
            ("d", math.pi / 2.0),
            ("a", math.pi),
            ("i", -math.pi / 2.0),
        ]
        self.qr_scan_step_index = 0
        self.qr_scan_phase = "hold"
        self.qr_scan_timeout_timer = rospy.Timer(
            rospy.Duration(self.qr_scan_timeout), self.qr_scan_timeout_cb, oneshot=True)
        rospy.loginfo("QR heading scan started: d at clockwise 270 deg, a at 180 deg, i at 90 deg; hold %.1f s per code.",
                      self.qr_scan_hold_sec)
        self.enter_qr_hold()

    def qr_result_cb(self, msg):
        # The camera can decode while the planner is settling at a heading.
        # Count results throughout the QR phase; a 0.5 s hold is too short to
        # make the scanner callback timing a requirement for task completion.
        if not self.qr_scan_active:
            return
        code = msg.data.strip()
        if not code or code in self.qr_codes:
            return
        self.qr_codes.add(code)
        label = self.qr_scan_steps[self.qr_scan_step_index][0]
        rospy.loginfo("QR_SCAN_RESULT stop=%s %d/%d: %s", label, len(self.qr_codes),
                      self.qr_scan_min_count, code)

    def current_map_position(self, context):
        try:
            translation, _rotation = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
            return translation[0], translation[1]
        except tf.Exception as exc:
            rospy.logerr("%s skipped: cannot get current map pose: %s", context, exc)
            return None

    def enter_qr_hold(self):
        if not self.qr_scan_active:
            return
        label, yaw = self.qr_scan_steps[self.qr_scan_step_index]
        self.qr_scan_phase = "hold"
        rospy.loginfo("QR_SCAN_HOLD stop=%s yaw=%.3f for %.1f s.", label, yaw,
                      self.qr_scan_hold_sec)
        self.qr_scan_hold_timer = rospy.Timer(
            rospy.Duration(self.qr_scan_hold_sec), self.qr_hold_done_cb, oneshot=True)

    def qr_hold_done_cb(self, _event):
        if not self.qr_scan_active:
            return
        if self.qr_scan_step_index >= len(self.qr_scan_steps) - 1:
            self.finish_qr_scan("heading sequence completed")
            return
        self.qr_scan_step_index += 1
        label, yaw = self.qr_scan_steps[self.qr_scan_step_index]
        current_position = self.current_map_position("QR heading goal")
        if current_position is None:
            self.finish_qr_scan("cannot get current pose for heading goal")
            return
        self.qr_scan_phase = "heading_goal"
        rospy.loginfo("QR_SCAN_GOAL stop=%s target=(%.3f, %.3f) yaw=%.3f via move_base/CymPlanner.",
                      label, current_position[0], current_position[1], yaw)
        self.send_goal(current_position[0], current_position[1], yaw,
                       self.qr_heading_goal_done_cb)
        self.qr_heading_goal_timer = rospy.Timer(
            rospy.Duration(self.qr_heading_goal_timeout),
            self.qr_heading_goal_timeout_cb, oneshot=True)

    def qr_heading_goal_done_cb(self, status, _result):
        if not self.qr_scan_active or self.qr_scan_phase != "heading_goal":
            return
        if self.qr_heading_goal_timer is not None:
            self.qr_heading_goal_timer.shutdown()
            self.qr_heading_goal_timer = None
        label, yaw = self.qr_scan_steps[self.qr_scan_step_index]
        if status != GoalStatus.SUCCEEDED:
            self.finish_qr_scan("heading goal %s failed with status %d" % (label, status))
            return
        rospy.loginfo("QR_SCAN_GOAL_REACHED stop=%s yaw=%.3f.", label, yaw)
        self.enter_qr_hold()

    def qr_heading_goal_timeout_cb(self, _event):
        if not self.qr_scan_active or self.qr_scan_phase != "heading_goal":
            return
        label, yaw = self.qr_scan_steps[self.qr_scan_step_index]
        self.qr_heading_goal_timer = None
        # Change phase before cancelling.  The action callback from this
        # cancellation must not turn the timeout into a task failure.
        self.qr_scan_phase = "hold"
        self.move_base.cancel_goal()
        rospy.logwarn("QR_SCAN_GOAL_TIMEOUT stop=%s yaw=%.3f after %.1f s; cancelling rotation and scanning current heading.",
                      label, yaw, self.qr_heading_goal_timeout)
        self.enter_qr_hold()

    def qr_scan_timeout_cb(self, _event):
        if not self.qr_scan_active:
            return
        self.move_base.cancel_goal()
        self.finish_qr_scan("timeout")

    def finish_qr_scan(self, reason):
        if not self.qr_scan_active:
            return
        self.qr_scan_active = False
        if self.qr_scan_hold_timer is not None:
            self.qr_scan_hold_timer.shutdown()
            self.qr_scan_hold_timer = None
        if self.qr_heading_goal_timer is not None:
            self.qr_heading_goal_timer.shutdown()
            self.qr_heading_goal_timer = None
        if self.qr_scan_timeout_timer is not None:
            self.qr_scan_timeout_timer.shutdown()
            self.qr_scan_timeout_timer = None
        ordered_codes = sorted(self.qr_codes)
        rospy.loginfo("QR_SCAN_FINISHED reason=%s codes=%d/%d values=%s",
                      reason, len(ordered_codes), self.qr_scan_min_count,
                      ordered_codes)
        if len(ordered_codes) < self.qr_scan_min_count:
            rospy.logwarn("QR sweep stopped with fewer than %d distinct QR codes.", self.qr_scan_min_count)
            return
        if reason != "heading sequence completed":
            rospy.logwarn("QR sweep completed its code count, but post-QR goal is skipped: %s.", reason)
            return
        if self.post_qr_goal_enabled:
            # finish_qr_scan normally runs in the QR hold timer callback.
            # Start the route in a new callback after that timer has fully
            # returned, otherwise the transition can stop after scanning.
            self.post_qr_start_timer = rospy.Timer(
                rospy.Duration(0.1), self.post_qr_start_cb, oneshot=True)

    def post_qr_start_cb(self, _event):
        self.post_qr_start_timer = None
        if not rospy.is_shutdown():
            self.send_post_qr_goal()

    def send_post_qr_goal(self):
        """Start the three-point, speed-limited post-QR task route."""
        self.set_holonomic_mode(False, "post_qr_first_goal")
        self.set_task_linear_speed(self.task_linear_speed, "post_qr_first_goal")
        self.send_goal(self.post_qr_goal_x, self.post_qr_goal_y, self.post_qr_goal_yaw,
                       self.post_qr_goal_done_cb)
        self.post_qr_status(
            "FIRST_GOAL target=(%.3f, %.3f) yaw=%.3f speed=%.3f" %
            (self.post_qr_goal_x, self.post_qr_goal_y,
             self.post_qr_goal_yaw, self.task_linear_speed))

    def post_qr_goal_done_cb(self, status, _result):
        if status == GoalStatus.SUCCEEDED:
            if self.second_goal_enabled:
                self.send_second_goal()
            else:
                self.restore_task_motion("post_qr_first_goal_finished")
            self.post_qr_status("FIRST_GOAL_REACHED")
        else:
            self.restore_task_motion("post_qr_first_goal_failed")
            self.post_qr_status("FIRST_GOAL_FAILED action_status=%d" % status)

    def send_second_goal(self):
        """Send the fixed second post-QR goal at the task speed."""
        # The first post-QR point is approached normally.  All remaining
        # points use lateral motion after that point has been reached.
        self.set_holonomic_mode(True, "post_qr_second_goal")
        self.send_goal(self.second_goal_x, self.second_goal_y, self.second_goal_yaw,
                       self.second_goal_done_cb)
        self.post_qr_status(
            "SECOND_GOAL target=(%.3f, %.3f) yaw=%.3f speed=%.3f" %
            (self.second_goal_x, self.second_goal_y,
             self.second_goal_yaw, self.task_linear_speed))

    def second_goal_done_cb(self, status, _result):
        if status != GoalStatus.SUCCEEDED:
            self.restore_task_motion("post_qr_second_goal_failed")
            self.post_qr_status("SECOND_GOAL_FAILED action_status=%d" % status)
            return
        if not self.next_goal_enabled:
            self.restore_task_motion("post_qr_second_goal_finished")
            self.post_qr_status("SECOND_GOAL_REACHED")
            return
        self.set_holonomic_mode(True, "fixed_third_goal")
        self.send_goal(self.next_goal_x, self.next_goal_y, self.next_goal_yaw,
                       self.next_goal_done_cb)
        self.post_qr_status(
            "SECOND_GOAL_REACHED; THIRD_GOAL target=(%.3f, %.3f) yaw=%.3f speed=%.3f holonomic=true" %
            (self.next_goal_x, self.next_goal_y, self.next_goal_yaw,
             self.task_linear_speed))

    def next_goal_done_cb(self, status, _result):
        self.restore_task_motion("fixed_third_goal_finished")
        if status == GoalStatus.SUCCEEDED:
            self.post_qr_status("THIRD_GOAL_REACHED")
        else:
            self.post_qr_status("THIRD_GOAL_FAILED action_status=%d" % status)


if __name__ == "__main__":
    rospy.init_node("navigation_2026")
    Navigation2026()
    rospy.loginfo("2026 navigation node started.")
    rospy.spin()
