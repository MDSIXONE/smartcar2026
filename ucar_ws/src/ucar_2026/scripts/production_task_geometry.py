#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Pure geometry and grid-loading helpers for the 2026 production mission.

This module deliberately has no ROS imports so its coordinate and heading
contract can be tested on the development computer as well as the vehicle.
"""

from __future__ import print_function

import json
import math


DEFAULT_PRODUCTION_ROUTE = [
    12, 22, 13, 23, 14, 24, 15, 25, 16, 26, 17, 27, 18, 28, 19, 29,
]
DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG = [
    -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45, -45, 45,
]


class TaskDefinitionError(ValueError):
    """Raised when the numbered-grid task configuration is incomplete."""


def is_finite(value):
    """Python 2-compatible finite-number check."""
    return not math.isnan(value) and not math.isinf(value)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def bearing(source, target):
    """Return the map yaw from a source ``(x, y)`` to a target ``(x, y)``."""
    delta_x = float(target[0]) - float(source[0])
    delta_y = float(target[1]) - float(source[1])
    if abs(delta_x) < 1e-12 and abs(delta_y) < 1e-12:
        raise TaskDefinitionError("bearing requires two different points")
    return math.atan2(delta_y, delta_x)


def position_error(source, target):
    """Return planar Euclidean distance between two ``(x, y)`` positions."""
    return math.hypot(
        float(target[0]) - float(source[0]),
        float(target[1]) - float(source[1]))


def needs_recenter(error, trigger_tolerance):
    """Return whether post-turn drift is large enough to request correction."""
    return float(error) > float(trigger_tolerance)


def positive_turn_increment(previous_yaw, current_yaw, direction=1.0):
    """Return forward progress for a commanded signed turn across +/-pi."""
    signed_delta = normalize_angle(float(current_yaw) - float(previous_yaw))
    directed_delta = signed_delta if direction >= 0.0 else -signed_delta
    return max(0.0, directed_delta)


def build_straight_segments(
        route_numbers, points, angular_tolerance_radians=math.radians(1.0)):
    """Collapse consecutive forward-collinear route legs into endpoints.

    Each returned pair is ``(start_number, end_number)``.  A leg may extend
    the current segment only when its direction remains forward (positive
    dot product) and its signed cross/dot angle stays inside the supplied
    tolerance.  Repeated route points are rejected rather than silently
    producing a zero-length navigation leg.
    """
    route = [int(number) for number in route_numbers]
    tolerance = float(angular_tolerance_radians)
    if len(route) < 2:
        raise TaskDefinitionError("route requires at least two points")
    if not is_finite(tolerance) or tolerance < 0.0 or tolerance >= math.pi:
        raise TaskDefinitionError(
            "angular_tolerance_radians must be finite and in [0, pi)")

    seen = set()
    seen_coordinates = set()
    coordinates = []
    for number in route:
        if number in seen:
            raise TaskDefinitionError("duplicate route point %d" % number)
        seen.add(number)
        if number not in points:
            raise TaskDefinitionError("grid point %d is missing" % number)
        coordinate = points[number]
        try:
            x_value = float(coordinate[0])
            y_value = float(coordinate[1])
        except (IndexError, TypeError, ValueError) as exc:
            raise TaskDefinitionError(
                "invalid coordinate for point %d: %s" % (number, exc))
        if not is_finite(x_value) or not is_finite(y_value):
            raise TaskDefinitionError("grid point %d is not finite" % number)
        if (x_value, y_value) in seen_coordinates:
            raise TaskDefinitionError(
                "duplicate route coordinate at point %d" % number)
        seen_coordinates.add((x_value, y_value))
        coordinates.append((x_value, y_value))

    segments = []
    segment_start = 0
    for index in range(1, len(route) - 1):
        previous = coordinates[index - 1]
        current = coordinates[index]
        following = coordinates[index + 1]
        first_x = current[0] - previous[0]
        first_y = current[1] - previous[1]
        second_x = following[0] - current[0]
        second_y = following[1] - current[1]
        first_length = math.hypot(first_x, first_y)
        second_length = math.hypot(second_x, second_y)
        if first_length < 1e-12 or second_length < 1e-12:
            raise TaskDefinitionError(
                "route contains coincident consecutive points at %d" %
                route[index])
        dot = first_x * second_x + first_y * second_y
        cross = first_x * second_y - first_y * second_x
        angle = abs(math.atan2(cross, dot))
        if dot <= 0.0 or angle > tolerance:
            segments.append((route[segment_start], route[index]))
            segment_start = index
    segments.append((route[segment_start], route[-1]))
    return segments


def load_numbered_points(path):
    """Load and validate the numbered points in the supplied full-grid JSON."""
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except (IOError, ValueError) as exc:
        raise TaskDefinitionError("cannot load grid file %s: %s" % (path, exc))

    raw_points = document.get("points")
    if not isinstance(raw_points, list):
        raise TaskDefinitionError("grid file has no points list")

    points = {}
    for raw_point in raw_points:
        try:
            number = int(raw_point["number"])
            x_value = float(raw_point["x_m"])
            y_value = float(raw_point["y_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise TaskDefinitionError("invalid grid point: %s" % exc)
        if number in points:
            raise TaskDefinitionError("duplicate grid point number %d" % number)
        if not is_finite(x_value) or not is_finite(y_value):
            raise TaskDefinitionError("grid point %d is not finite" % number)
        points[number] = (x_value, y_value)
    return points


def load_wall_reference_points(path):
    """Load only the explicitly approved middle-wall matching candidates."""
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except (IOError, ValueError) as exc:
        raise TaskDefinitionError("cannot load grid file %s: %s" % (path, exc))
    raw_numbers = document.get("wall_reference_point_numbers")
    if not isinstance(raw_numbers, list) or not raw_numbers:
        raise TaskDefinitionError(
            "grid file has no wall_reference_point_numbers list")
    points = load_numbered_points(path)
    return require_points(points, raw_numbers)


def load_middle_target_guard_points(path, target_numbers):
    """Return the four numbered middle-grid vertices around each target.

    A production target is a 0.5 m middle-zone square centre.  Its guard is
    the four line endpoints one half cell away in X and Y.  Deriving this
    from the immutable grid file keeps the task route and guard labels in one
    coordinate contract instead of maintaining a second hand-written map.
    """
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except (IOError, ValueError) as exc:
        raise TaskDefinitionError("cannot load grid file %s: %s" % (path, exc))

    try:
        side_length = float(document["square_side_m"])
        endpoint_range = document["numbering_scheme"][
            "middle_line_endpoints"]
        first_endpoint = int(endpoint_range[0])
        last_endpoint = int(endpoint_range[1])
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise TaskDefinitionError(
            "grid file has invalid middle guard metadata: %s" % exc)
    if not is_finite(side_length) or side_length <= 0.0:
        raise TaskDefinitionError("square_side_m must be finite and positive")
    if last_endpoint < first_endpoint:
        raise TaskDefinitionError("middle line endpoint range is reversed")

    points = load_numbered_points(path)
    endpoints = require_points(
        points, range(first_endpoint, last_endpoint + 1))
    half_side = side_length / 2.0
    tolerance = 1e-8
    result = {}
    for raw_target in target_numbers:
        target_number = int(raw_target)
        if target_number not in points:
            raise TaskDefinitionError("grid point %d is missing" % target_number)
        target_x, target_y = points[target_number]
        expected = (
            (target_x - half_side, target_y + half_side),
            (target_x + half_side, target_y + half_side),
            (target_x - half_side, target_y - half_side),
            (target_x + half_side, target_y - half_side),
        )
        guard_points = {}
        for expected_point in expected:
            matches = [
                number for number, coordinate in endpoints.items()
                if position_error(coordinate, expected_point) <= tolerance]
            if len(matches) != 1:
                raise TaskDefinitionError(
                    "target %d requires one middle endpoint at (%.3f, %.3f), "
                    "found %d" % (
                        target_number, expected_point[0], expected_point[1],
                        len(matches)))
            number = matches[0]
            guard_points[number] = endpoints[number]
        if len(guard_points) != 4:
            raise TaskDefinitionError(
                "target %d did not resolve four distinct guard points" %
                target_number)
        result[target_number] = guard_points
    return result


def require_points(points, numbers):
    """Return the requested points, rejecting a mission with missing labels."""
    resolved = {}
    for raw_number in numbers:
        number = int(raw_number)
        if number not in points:
            raise TaskDefinitionError("grid point %d is missing" % number)
        resolved[number] = points[number]
    return resolved
