#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Navigation-only scan bridge for the real UCar.

The driver is deliberately remapped to ``/scan_raw``.  This node restores the
normal ``/scan`` navigation input without creating goals or task state.  It
also keeps the global dynamic-obstacle input separate from the static map:
laser returns that already fall on a static wall are removed before they reach
the global obstacle layer, while new obstacles remain available for replanning.
"""

from __future__ import print_function

import math

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class NavigationScanRelay(object):
    def __init__(self):
        rospy.init_node('navigation_scan_relay')
        self.tf_listener = tf.TransformListener()
        self.static_filter_enabled = rospy.get_param(
            '~global_obstacle_filter_enabled', True)
        self.static_filter_radius = float(rospy.get_param(
            '~global_static_filter_radius', 0.22))
        self.static_filter_threshold = int(rospy.get_param(
            '~global_static_filter_threshold', 65))
        self.transform_max_age = float(rospy.get_param(
            '~global_transform_max_age', 0.20))
        self.static_mask = None
        self.static_map_info = None

        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=5)
        self.global_scan_pub = rospy.Publisher(
            '/scan_global_obstacles', LaserScan, queue_size=5)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb,
                                        queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan_raw', LaserScan, self.scan_cb,
                                         queue_size=5)
        rospy.loginfo('navigation_scan_relay: /scan_raw -> /scan is ready')

    @staticmethod
    def copy_scan(scan):
        copied = LaserScan()
        copied.header = scan.header
        copied.angle_min = scan.angle_min
        copied.angle_max = scan.angle_max
        copied.angle_increment = scan.angle_increment
        copied.time_increment = scan.time_increment
        copied.scan_time = scan.scan_time
        copied.range_min = scan.range_min
        copied.range_max = scan.range_max
        copied.ranges = list(scan.ranges)
        copied.intensities = scan.intensities
        return copied

    @classmethod
    def empty_scan(cls, scan):
        empty = cls.copy_scan(scan)
        empty.ranges = [float('inf')] * len(scan.ranges)
        return empty

    def map_cb(self, grid):
        """Build a dilated static-wall mask once per map publication."""
        if not self.static_filter_enabled:
            return

        info = grid.info
        if info.width <= 0 or info.height <= 0 or info.resolution <= 0.0:
            return

        radius_cells = int(math.ceil(self.static_filter_radius / info.resolution))
        radius_squared = radius_cells * radius_cells
        offsets = []
        for delta_y in range(-radius_cells, radius_cells + 1):
            for delta_x in range(-radius_cells, radius_cells + 1):
                if delta_x * delta_x + delta_y * delta_y <= radius_squared:
                    offsets.append((delta_x, delta_y))

        width = info.width
        height = info.height
        mask = bytearray(width * height)
        for index, cost in enumerate(grid.data):
            if cost < self.static_filter_threshold:
                continue
            cell_y = index // width
            cell_x = index - cell_y * width
            for delta_x, delta_y in offsets:
                x = cell_x + delta_x
                y = cell_y + delta_y
                if 0 <= x < width and 0 <= y < height:
                    mask[y * width + x] = 1

        self.static_mask = mask
        self.static_map_info = info
        rospy.loginfo('navigation_scan_relay: %.2f m static-wall mask ready',
                      self.static_filter_radius)

    def filter_global_scan(self, scan):
        """Drop returns that are already represented by the static map."""
        if not self.static_filter_enabled:
            return scan
        if self.static_mask is None or self.static_map_info is None:
            # Fail closed until map and localization are ready.  Marking raw
            # static-wall returns can otherwise close narrow mapped passages.
            return self.empty_scan(scan)

        try:
            translation, rotation = self.tf_listener.lookupTransform(
                'map', scan.header.frame_id, scan.header.stamp)
        except tf.Exception as exact_error:
            # The YDLidar message can lead the latest map<-laser TF by a few
            # milliseconds.  Retain the scan only when a very recent common
            # transform exists; otherwise preserve the fail-closed behaviour.
            try:
                latest_stamp = self.tf_listener.getLatestCommonTime(
                    'map', scan.header.frame_id)
                transform_age = (rospy.Time.now() - latest_stamp).to_sec()
                if latest_stamp.is_zero() or transform_age > self.transform_max_age:
                    raise tf.Exception(
                        'latest common TF age %.3f s exceeds %.3f s' % (
                            transform_age, self.transform_max_age))
                translation, rotation = self.tf_listener.lookupTransform(
                    'map', scan.header.frame_id, rospy.Time(0))
            except tf.Exception as fallback_error:
                rospy.logwarn_throttle(
                    5.0, 'navigation_scan_relay: dropping global scan without '
                    'usable map<-%s TF (exact: %s; fallback: %s)' %
                    (scan.header.frame_id, exact_error, fallback_error))
                return self.empty_scan(scan)
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, 'navigation_scan_relay: dropping global scan without '
                'map<-%s TF: %s' % (scan.header.frame_id, exc))
            return self.empty_scan(scan)

        info = self.static_map_info
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        filtered = self.copy_scan(scan)
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
                    self.static_mask[cell_y * info.width + cell_x]):
                filtered.ranges[index] = float('inf')
        return filtered

    def scan_cb(self, scan):
        if rospy.is_shutdown():
            return
        try:
            self.scan_pub.publish(scan)
            self.global_scan_pub.publish(self.filter_global_scan(scan))
        except rospy.ROSException:
            # roslaunch can close publishers while the final queued scan
            # callback is still running.  Suppress only that shutdown race.
            if not rospy.is_shutdown():
                raise


if __name__ == '__main__':
    try:
        NavigationScanRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
