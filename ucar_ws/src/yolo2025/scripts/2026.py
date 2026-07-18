#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""2026 navigation startup task, scan forwarding, and post-goal QR sweep."""

import json
import math
import sys

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from geometry_msgs.msg import Point, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker


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
        self.production_route_enabled = rospy.get_param("~production_route_enabled", True)
        self.production_square_centers_path = rospy.get_param(
            "~production_square_centers_path", "")
        self.production_route_numbers = rospy.get_param(
            "~production_route_numbers", [2, 12, 22, 32, 31, 21, 11, 1])
        self.production_linear_speed = float(rospy.get_param(
            "~production_linear_speed", 0.25))
        self.production_arrival_tolerance = float(rospy.get_param(
            "~production_arrival_tolerance", 0.05))
        self.production_arrival_verification_tolerance = max(
            self.production_arrival_tolerance,
            float(rospy.get_param("~production_arrival_verification_tolerance", 0.08)))
        self.production_heading_tolerance = float(
            rospy.get_param("~production_heading_tolerance", 0.07))
        self.production_alignment_timeout = float(
            rospy.get_param("~production_alignment_timeout", 8.0))
        self.production_alignment_control_rate = float(
            rospy.get_param("~production_alignment_control_rate", 20.0))
        self.production_alignment_kp = float(
            rospy.get_param("~production_alignment_kp", 1.2))
        self.production_alignment_max_angular_speed = float(
            rospy.get_param("~production_alignment_max_angular_speed", 0.5))
        self.local_obstacle_layer = rospy.get_param(
            "~local_obstacle_layer", "/move_base/local_costmap/obstacle_layer")
        self.cym_holonomic_mode_param = rospy.get_param(
            "~cym_holonomic_mode_param",
            "/move_base/cym_planner/CymPlanner/holonomic_mode")
        self.cym_task_max_vel_param = rospy.get_param(
            "~cym_task_max_vel_param",
            "/move_base/cym_planner/CymPlanner/task_max_vel")

        self.tf_listener = tf.TransformListener()
        self.make_plan = rospy.ServiceProxy("move_base/make_plan", GetPlan)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.footprint_marker_pub = rospy.Publisher(
            "/navigation_2026/footprint", Marker, queue_size=2, latch=True)
        self.robot_footprint = rospy.get_param(
            "~robot_footprint",
            [[0.171, -0.128], [0.171, 0.128], [-0.171, 0.128], [-0.171, -0.128]])
        self.footprint_safety_margin = float(
            rospy.get_param("~footprint_safety_margin", 0.05))

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
        self.production_start_timer = None
        self.production_alignment_timer = None
        self.production_active = False
        self.production_phase = None
        self.production_alignment_yaw = None
        self.production_alignment_deadline = None
        self.production_waypoint_index = 0
        self.production_local_obstacle_layer_disabled = False
        self.production_global_obstacles_frozen = False
        self.production_centres = {}
        self.production_requested_route = self.load_production_route()
        self.production_route = self.expand_production_route(
            self.production_requested_route)

        rospy.on_shutdown(self.shutdown)
        self.publish_footprint_markers()

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
        if not self.production_global_obstacles_frozen:
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

    @classmethod
    def empty_scan(cls, scan):
        """Return a scan that cannot mark or clear a costmap obstacle layer."""
        out = cls.clone_scan(scan)
        out.ranges = [float("inf")] * len(scan.ranges)
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
            return self.empty_scan(scan)

        try:
            translation, rotation = self.tf_listener.lookupTransform(
                "map", scan.header.frame_id, scan.header.stamp)
        except tf.Exception as exc:
            # A scan without a map-frame pose is unsafe for global marking:
            # forwarding it would place vehicle-frame returns in map space.
            rospy.logwarn_throttle(
                5.0,
                "Global obstacle scan dropped: no map<-%s TF at scan stamp: %s" %
                (scan.header.frame_id, exc))
            return self.empty_scan(scan)

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

    def load_production_route(self):
        """Resolve the user-selected production route from numbered grid centres."""
        if not self.production_route_enabled:
            return []
        try:
            with open(self.production_square_centers_path, "r") as handle:
                data = json.load(handle)
            centres = {}
            for point in data["points"]:
                centres[int(point["number"])] = (
                    float(point["x_m"]), float(point["y_m"]))
            self.production_centres = centres
            route_numbers = [int(number) for number in self.production_route_numbers]
            if not route_numbers:
                raise ValueError("production_route_numbers is empty")
            route = []
            for number in route_numbers:
                if number not in centres:
                    raise ValueError("production point %d is not in the centres file" % number)
                x, y = centres[number]
                route.append((number, x, y))
            rospy.loginfo("Production route loaded: %s", route_numbers)
            return route
        except (IOError, ValueError, KeyError, TypeError) as exc:
            rospy.logerr("Production route is unavailable: %s", exc)
            return []

    def production_centre_at(self, x, y):
        """Return the numbered centre at an exact grid intersection, if any."""
        for number, point in self.production_centres.items():
            if abs(point[0] - x) < 1e-6 and abs(point[1] - y) < 1e-6:
                return number, point[0], point[1]
        return None

    def expand_production_route(self, requested_route):
        """Insert a grid centre for every diagonal requested transition.

        The enforced order is horizontal then vertical.  For example, 1 -> 26
        becomes 1 -> 6 -> 26: point 6 shares point 1's row and point 26's
        column.  This makes every requested move_base segment axis-aligned and
        makes every intentional turn occur at a numbered centre.
        """
        if not requested_route:
            return []
        route = [requested_route[0]]
        for number, target_x, target_y in requested_route[1:]:
            _, previous_x, previous_y = route[-1]
            if abs(target_x - previous_x) >= 1e-6 and abs(target_y - previous_y) >= 1e-6:
                turning_point = self.production_centre_at(target_x, previous_y)
                if turning_point is None:
                    rospy.logerr(
                        "Production route cannot connect point %d to %d: "
                        "no horizontal-then-vertical turning centre.",
                        route[-1][0], number)
                    return []
                route.append(turning_point)
            route.append((number, target_x, target_y))
        rospy.loginfo("Production route expanded: %s", [point[0] for point in route])
        return route

    def current_map_pose(self, context):
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rotation)[2]
            return translation[0], translation[1], yaw
        except tf.Exception as exc:
            rospy.logerr("%s skipped: cannot get current map pose: %s", context, exc)
            return None

    def publish_motion(self, linear_x=0.0, angular_z=0.0):
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.cmd_vel_pub.publish(command)

    def publish_stop(self):
        self.publish_motion()

    def publish_footprint_markers(self):
        """Show the physical footprint and its 5 cm safety envelope in RViz."""
        try:
            points = [(float(point[0]), float(point[1]))
                      for point in self.robot_footprint]
            if len(points) < 3:
                raise ValueError("fewer than three footprint vertices")
        except (TypeError, ValueError, IndexError) as exc:
            rospy.logerr("Cannot publish robot footprint marker: %s", exc)
            return
        self.publish_footprint_marker(0, points, 1.0, 0.1, 0.1, "physical")
        min_x = min(point[0] for point in points) - self.footprint_safety_margin
        max_x = max(point[0] for point in points) + self.footprint_safety_margin
        min_y = min(point[1] for point in points) - self.footprint_safety_margin
        max_y = max(point[1] for point in points) + self.footprint_safety_margin
        self.publish_footprint_marker(
            1, [(max_x, min_y), (max_x, max_y), (min_x, max_y), (min_x, min_y)],
            1.0, 0.85, 0.0, "safety_margin")

    def publish_footprint_marker(self, marker_id, points, red, green, blue, namespace):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "navigation_2026_footprint"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        # RViz rejects an all-zero quaternion as uninitialised, even though a
        # LINE_STRIP has no meaningful pose rotation.  Use the identity pose
        # so the footprint remains fixed in base_link and is renderable.
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.018
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.text = namespace
        outline = list(points) + [points[0]]
        for x, y in outline:
            point = Point()
            point.x = x
            point.y = y
            marker.points.append(point)
        self.footprint_marker_pub.publish(marker)

    def set_local_obstacle_layer_enabled(self, enabled, stage):
        """Disable local lidar overlay without erasing the global costmap."""
        try:
            client = DynamicReconfigureClient(self.local_obstacle_layer, timeout=2.0)
            client.update_configuration({"enabled": bool(enabled)})
        except Exception as exc:
            rospy.logerr("%s: cannot set local obstacle layer to %s: %s",
                         stage, enabled, exc)
            return False
        self.production_local_obstacle_layer_disabled = not enabled
        rospy.loginfo("PRODUCTION_COSTMAP stage=%s local_dynamic_obstacles=%s",
                      stage, "enabled" if enabled else "disabled")
        return True

    def finish_production_route(self, success, reason):
        if self.production_alignment_timer is not None:
            self.production_alignment_timer.shutdown()
            self.production_alignment_timer = None
        self.production_active = False
        self.production_phase = None
        self.production_alignment_yaw = None
        self.production_alignment_deadline = None
        self.production_global_obstacles_frozen = False
        self.publish_stop()
        self.set_holonomic_mode(False, "production_route_finish")
        self.set_task_linear_speed(0.0, "production_route_finish")
        if self.production_local_obstacle_layer_disabled:
            self.set_local_obstacle_layer_enabled(True, "production_route_restore")
        outcome = "REACHED" if success else "STOPPED"
        message = "[PRODUCTION_ROUTE] %s reason=%s" % (outcome, reason)
        sys.stdout.write(message + "\n")
        sys.stdout.flush()
        rospy.loginfo(message)

    def production_goal_yaw(self, index):
        """Finish a move_base goal facing its approach, not the next turn."""
        if index > 0:
            _, previous_x, previous_y = self.production_route[index - 1]
            _, x, y = self.production_route[index]
            return math.atan2(y - previous_y, x - previous_x)
        # QR scanning ends at -pi/2, which already matches the configured
        # first production segment (2 -> 12).  Retain that compatible heading.
        if index + 1 < len(self.production_route):
            _, x, y = self.production_route[index]
            _, next_x, next_y = self.production_route[index + 1]
            return math.atan2(next_y - y, next_x - x)
        pose = self.current_map_pose("production first-goal heading")
        return pose[2] if pose is not None else 0.0

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def production_goal_plan_ready(self, target_x, target_y, target_yaw):
        """Fail safely when move_base has no map-valid route to the next centre."""
        pose = self.current_map_pose("production route plan")
        if pose is None:
            return False
        try:
            rospy.wait_for_service("move_base/make_plan", timeout=3.0)
            plan = self.make_plan(self.map_pose(pose[0], pose[1], pose[2]),
                                  self.map_pose(target_x, target_y, target_yaw), 0.0)
            if len(plan.plan.poses) > 1:
                rospy.loginfo("PRODUCTION_ROUTE global plan to next centre has %d poses.",
                              len(plan.plan.poses))
                return True
            rospy.logerr("PRODUCTION_ROUTE no global plan to the next centre.")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("PRODUCTION_ROUTE cannot verify global plan: %s", exc)
        return False

    def start_next_production_goal(self):
        if not self.production_active:
            return
        if self.production_waypoint_index >= len(self.production_route):
            self.finish_production_route(True, "all configured grid centres reached")
            return
        number, target_x, target_y = self.production_route[self.production_waypoint_index]
        target_yaw = self.production_goal_yaw(self.production_waypoint_index)
        if not self.production_goal_plan_ready(target_x, target_y, target_yaw):
            self.finish_production_route(False, "no safe global plan to point %d" % number)
            return
        rospy.loginfo(
            "PRODUCTION_ROUTE goal=%d target=(%.3f, %.3f) yaw=%.3f via move_base.",
            number, target_x, target_y, target_yaw)
        self.send_goal(target_x, target_y, target_yaw, self.production_goal_done_cb)

    def start_production_alignment(self):
        """At a reached grid centre, orient before planning the next segment."""
        if self.production_waypoint_index + 1 >= len(self.production_route):
            self.finish_production_route(True, "all configured grid centres reached")
            return
        number, x, y = self.production_route[self.production_waypoint_index]
        next_number, next_x, next_y = self.production_route[
            self.production_waypoint_index + 1]
        pose = self.current_map_pose("production centre alignment")
        if pose is None:
            self.finish_production_route(False, "cannot align at point %d" % number)
            return
        self.production_phase = "align"
        self.production_alignment_yaw = math.atan2(next_y - y, next_x - x)
        self.production_alignment_deadline = (
            rospy.Time.now() + rospy.Duration(self.production_alignment_timeout))
        if self.production_alignment_timer is not None:
            self.production_alignment_timer.shutdown()
        period = 1.0 / max(1.0, self.production_alignment_control_rate)
        self.production_alignment_timer = rospy.Timer(
            rospy.Duration(period), self.production_alignment_control_cb)
        rospy.loginfo(
            "PRODUCTION_ALIGN point=%d next=%d heading=%.3f timeout=%.1f s.",
            number, next_number, self.production_alignment_yaw,
            self.production_alignment_timeout)

    def production_alignment_control_cb(self, _event):
        if not self.production_active or self.production_phase != "align":
            return
        number, _, _ = self.production_route[self.production_waypoint_index]
        pose = self.current_map_pose("production alignment control")
        if pose is None:
            self.finish_production_route(False, "map pose unavailable while aligning point %d" % number)
            return
        heading_error = self.normalize_angle(self.production_alignment_yaw - pose[2])
        if abs(heading_error) <= self.production_heading_tolerance:
            self.publish_stop()
            if self.production_alignment_timer is not None:
                self.production_alignment_timer.shutdown()
                self.production_alignment_timer = None
            self.production_phase = None
            rospy.loginfo("PRODUCTION_ALIGN point=%d complete; planning next segment.", number)
            self.production_waypoint_index += 1
            self.start_next_production_goal()
            return
        if rospy.Time.now() >= self.production_alignment_deadline:
            self.finish_production_route(
                False, "point %d heading did not settle within %.1f s" % (
                    number, self.production_alignment_timeout))
            return
        angular_z = self.clamp(
            self.production_alignment_kp * heading_error,
            -self.production_alignment_max_angular_speed,
            self.production_alignment_max_angular_speed)
        # Translation remains entirely under move_base; this bounded command is
        # only the required in-place turn at a numbered grid centre.
        self.publish_motion(0.0, angular_z)

    def production_goal_done_cb(self, status, _result):
        if not self.production_active:
            return
        number, target_x, target_y = self.production_route[self.production_waypoint_index]
        self.publish_stop()
        if status != GoalStatus.SUCCEEDED:
            self.finish_production_route(
                False, "move_base goal point %d failed with status %d" % (number, status))
            return
        pose = self.current_map_pose("production goal completion")
        if pose is None:
            self.finish_production_route(False, "cannot verify arrival at point %d" % number)
            return
        arrival_error = math.hypot(target_x - pose[0], target_y - pose[1])
        # CymPlanner reaches the centre with its 0.05 m control threshold.
        # lidar_loc can update map->odom between action completion and this
        # callback, so this is an audit guard rather than a second controller.
        if arrival_error > self.production_arrival_verification_tolerance:
            self.finish_production_route(
                False, "move_base stopped %.3f m from point %d (limit %.3f m)" % (
                    arrival_error, number,
                    self.production_arrival_verification_tolerance))
            return
        rospy.loginfo("PRODUCTION_ROUTE point=%d reached.", number)
        self.start_production_alignment()

    def begin_production_route(self):
        """Run the expanded grid-centre sequence through normal move_base planning."""
        if not self.production_route_enabled:
            rospy.loginfo("Production route is disabled; task ends after QR scan.")
            self.restore_task_motion("production_route_disabled")
            return
        if not self.production_route:
            rospy.logerr("Production route has no valid grid centres; task stops.")
            self.restore_task_motion("production_route_configuration_invalid")
            return
        self.move_base.cancel_all_goals()
        self.publish_stop()
        self.set_holonomic_mode(False, "production_route_enter")
        self.set_task_linear_speed(self.production_linear_speed, "production_route_enter")
        if not self.move_base.wait_for_server(rospy.Duration(5.0)):
            self.finish_production_route(False, "move_base unavailable for production route")
            return
        # Keep the global obstacle layer and all of its existing cost values.
        # Freezing its scan source blocks new marks without clearing old cells.
        self.production_global_obstacles_frozen = True
        if not self.set_local_obstacle_layer_enabled(False, "production_route_enter"):
            self.production_global_obstacles_frozen = False
            self.restore_task_motion("production_route_costmap_disable_failed")
            return
        self.production_active = True
        self.production_waypoint_index = 0
        self.start_next_production_goal()

    def shutdown(self):
        self.publish_stop()
        self.production_global_obstacles_frozen = False
        if self.production_alignment_timer is not None:
            self.production_alignment_timer.shutdown()
            self.production_alignment_timer = None
        if self.production_local_obstacle_layer_disabled:
            self.set_local_obstacle_layer_enabled(True, "shutdown_restore")

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
            rospy.logwarn("QR sweep completed its code count, but the production route is skipped: %s.", reason)
            return
        # The QR hold timer has to return before direct base control starts.
        self.production_start_timer = rospy.Timer(
            rospy.Duration(0.1), self.production_start_cb, oneshot=True)

    def production_start_cb(self, _event):
        self.production_start_timer = None
        if not rospy.is_shutdown():
            self.begin_production_route()


if __name__ == "__main__":
    rospy.init_node("navigation_2026")
    Navigation2026()
    rospy.loginfo("2026 navigation node started.")
    rospy.spin()
