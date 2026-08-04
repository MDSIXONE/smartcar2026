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


def alignment_angular_speed(
        error_pixels, kp, kd, maximum_speed, image_is_mirrored,
        previous_error_pixels=None, sample_seconds=None):
    """Return a bounded PD yaw rate that moves an OCR box to centre.

    OCR reports its bbox in the same post-processing image coordinates that
    the controller observes.  The vehicle trial proves this convention stays
    valid when the helper mirrors a ROS image: applying a second sign reversal
    moves the box farther from centre.
    """
    error = float(error_pixels)
    maximum = abs(float(maximum_speed))
    derivative = 0.0
    if previous_error_pixels is not None and sample_seconds is not None:
        previous = float(previous_error_pixels)
        elapsed = max(0.05, float(sample_seconds))
        derivative = (error - previous) / elapsed
    command = -float(kp) * error - float(kd) * derivative
    return max(-maximum, min(maximum, command))


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


def normalize_production_category(text):
    """Return the one allowed production category named by OCR, if any.

    The OCR classifier may return either the short name or the full workshop
    label.  Keep this deliberately narrow: unrelated text must not stop a
    rotating vehicle or become a mission result.
    """
    if text is None:
        return None
    try:
        value = text.strip()
    except AttributeError:
        return None
    for keyword, category in ((u"日用品", u"日用品"),
                              (u"食品", u"食品"),
                              (u"电子产品", u"电子产品"),
                              (u"电子", u"电子产品")):
        if keyword in value:
            return category
    return None


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


def forward_ray_wall_intersection(pose, wall_points):
    """Intersect a forward lidar ray with the rectangular middle boundary.

    The immutable grid supplies boundary reference points on all four sides.
    This avoids treating the noisy front range endpoint as an arbitrary map
    point.  The caller still checks that the measured range agrees with the
    predicted wall distance before accepting a result.
    """
    if not wall_points:
        return None
    try:
        x_value, y_value, yaw = [float(value) for value in pose]
        xs = [float(point[0]) for point in wall_points.values()]
        ys = [float(point[1]) for point in wall_points.values()]
    except (TypeError, ValueError, IndexError):
        return None
    if not all(is_finite(value) for value in (x_value, y_value, yaw)):
        return None
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    dx, dy = math.cos(yaw), math.sin(yaw)
    candidates = []
    epsilon = 1e-9
    if abs(dx) > epsilon:
        for boundary_x in (min_x, max_x):
            distance = (boundary_x - x_value) / dx
            hit_y = y_value + distance * dy
            if distance > epsilon and min_y - epsilon <= hit_y <= max_y + epsilon:
                candidates.append((distance, (boundary_x, hit_y)))
    if abs(dy) > epsilon:
        for boundary_y in (min_y, max_y):
            distance = (boundary_y - y_value) / dy
            hit_x = x_value + distance * dx
            if distance > epsilon and min_x - epsilon <= hit_x <= max_x + epsilon:
                candidates.append((distance, (hit_x, boundary_y)))
    if not candidates:
        return None
    return min(candidates, key=lambda item: item[0])


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


def select_three_processing_observations(observations):
    """Choose one accepted result per requested workshop category."""
    strongest = {}
    for observation in observations:
        category = normalize_production_category(
            observation.get("processing_category"))
        if category is None or observation.get("wall_point_number") is None:
            continue
        previous = strongest.get(category)
        if (previous is None or
                float(observation.get("confidence", -1.0)) >
                float(previous.get("confidence", -1.0))):
            strongest[category] = observation
    return sorted(strongest.values(),
                  key=lambda item: (item.get("processing_category", u""),
                                    -float(item.get("confidence", -1.0))))
