#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Pure alignment and lidar helpers for the 2026 production mission."""

from __future__ import print_function

import math

from production_task_geometry import is_finite, position_error


def select_alignment_detection(detections, image_width):
    """Prefer confidence, then closeness to the horizontal image centre."""
    if not detections:
        return None
    centre = float(image_width) / 2.0

    def score(item):
        left, _top, width, _height = item["bbox"]
        offset = abs((left + width / 2.0) - centre)
        return float(item["confidence"]) - 0.02 * offset

    return max(detections, key=score)


def horizontal_pixel_error(detection, image_width):
    left, _top, width, _height = detection["bbox"]
    return (left + width / 2.0) - float(image_width) / 2.0


def is_navigation_ocr_candidate(response, minimum_confidence):
    """Return whether a moving OCR response is strong enough to stop for."""
    if not isinstance(response, dict) or not response.get("ok"):
        return False
    detection = response.get("detection")
    if not isinstance(detection, dict):
        return False
    text = (detection.get("text") or "").strip()
    try:
        confidence = float(detection.get("confidence", -1.0))
    except (TypeError, ValueError):
        return False
    return bool(text) and confidence >= float(minimum_confidence)


def odom_velocity_is_stopped(velocity, epsilon):
    """Return whether planar odometry velocity is inside the stop gate."""
    if velocity is None or len(velocity) != 3:
        return False
    return max(abs(float(value)) for value in velocity) <= float(epsilon)


def front_scan_distance(scan, half_angle_radians):
    """Median finite lidar range in a symmetric forward angular window."""
    if scan is None or not scan.ranges or scan.angle_increment == 0.0:
        return None
    distances = []
    for index, raw_distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if abs(angle) > float(half_angle_radians):
            continue
        distance = float(raw_distance)
        if (
                is_finite(distance) and
                distance >= float(scan.range_min) and
                distance <= float(scan.range_max)):
            distances.append(distance)
    if not distances:
        return None
    distances.sort()
    middle = len(distances) // 2
    if len(distances) % 2:
        return distances[middle]
    return (distances[middle - 1] + distances[middle]) / 2.0


def target_guard_scan_matches(scan, laser_pose, guard_points, max_error):
    """Match finite filtered lidar hits to one target's guard vertices.

    ``scan`` must be the global obstacle scan, where returns already explained
    by the static map are removed.  The function is ROS-free so the geometric
    contract can be checked on the development computer.
    """
    if (
            scan is None or not scan.ranges or
            scan.angle_increment == 0.0 or not guard_points):
        return {}
    try:
        laser_x, laser_y, laser_yaw = [float(value) for value in laser_pose]
        threshold = float(max_error)
    except (TypeError, ValueError):
        return {}
    if (
            not all(is_finite(value)
                    for value in (laser_x, laser_y, laser_yaw, threshold)) or
            threshold <= 0.0):
        return {}

    matches = {}
    for index, raw_distance in enumerate(scan.ranges):
        try:
            distance = float(raw_distance)
        except (TypeError, ValueError):
            continue
        if (
                not is_finite(distance) or
                distance < float(scan.range_min) or
                distance > float(scan.range_max)):
            continue
        angle = float(scan.angle_min) + index * float(scan.angle_increment)
        hit = (
            laser_x + distance * math.cos(laser_yaw + angle),
            laser_y + distance * math.sin(laser_yaw + angle),
        )
        nearest = nearest_numbered_point(hit, guard_points)
        if nearest is None:
            continue
        number, _coordinate, error = nearest
        if error <= threshold:
            previous = matches.get(number)
            if previous is None or error < previous:
                matches[number] = error
    return matches


def projected_wall_hit(pose, front_distance, lidar_forward_offset=0.0):
    """Project the front lidar return into map coordinates."""
    x_value, y_value, yaw = pose
    distance = float(front_distance) + float(lidar_forward_offset)
    return (
        float(x_value) + distance * math.cos(float(yaw)),
        float(y_value) + distance * math.sin(float(yaw)),
    )


def nearest_numbered_point(hit, numbered_points):
    """Return ``(number, coordinate, error)`` for the nearest candidate."""
    if not numbered_points:
        return None
    number, coordinate = min(
        numbered_points.items(),
        key=lambda item: position_error(hit, item[1]))
    return number, coordinate, position_error(hit, coordinate)


def select_three_observations(observations):
    """Choose the strongest observation for each wall point, then top three."""
    strongest = {}
    for observation in observations:
        point_number = observation.get("wall_point_number")
        text = (observation.get("text") or "").strip()
        if point_number is None or not text:
            continue
        previous = strongest.get(point_number)
        if (
                previous is None or
                float(observation.get("confidence", -1.0)) >
                float(previous.get("confidence", -1.0))):
            strongest[point_number] = observation
    return sorted(
        strongest.values(),
        key=lambda item: (
            float(item.get("confidence", -1.0)),
            -float(item.get("wall_match_error_m", float("inf")))),
        reverse=True)[:3]
