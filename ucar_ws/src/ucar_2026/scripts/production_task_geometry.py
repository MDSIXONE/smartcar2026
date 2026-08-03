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
    12, 23, 14, 25, 16,
]
DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG = [
    -45, 45, -45, 45, 45,
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
    """Return each middle target's four adjacent line-endpoint vertices.

    A target guard is deliberately derived from the numbered-grid geometry,
    rather than hard-coded in the mission.  This makes the rule explicit and
    prevents a future grid edit from silently associating a centre with a
    neighbouring square's wall endpoints.
    """
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except (IOError, ValueError) as exc:
        raise TaskDefinitionError("cannot load grid file %s: %s" % (path, exc))

    raw_points = document.get("points")
    if not isinstance(raw_points, list):
        raise TaskDefinitionError("grid file has no points list")
    try:
        half_side = float(document["square_side_m"]) / 2.0
    except (KeyError, TypeError, ValueError) as exc:
        raise TaskDefinitionError("invalid square_side_m: %s" % exc)
    if not is_finite(half_side) or half_side <= 0.0:
        raise TaskDefinitionError("square_side_m must be finite and positive")

    points = load_numbered_points(path)
    records = {}
    for raw_point in raw_points:
        try:
            number = int(raw_point["number"])
        except (KeyError, TypeError, ValueError) as exc:
            raise TaskDefinitionError("invalid grid point: %s" % exc)
        if number in records:
            raise TaskDefinitionError("duplicate grid point number %d" % number)
        records[number] = raw_point

    guards = {}
    for raw_target in target_numbers:
        target = int(raw_target)
        centre = records.get(target)
        if centre is None:
            raise TaskDefinitionError("grid point %d is missing" % target)
        if centre.get("type") != "center" or centre.get("region") != "middle":
            raise TaskDefinitionError(
                "target guard point %d must be a middle centre" % target)
        centre_coordinate = points[target]
        guard = {}
        for candidate_number, candidate in records.items():
            if (
                    candidate.get("type") != "vertex" or
                    candidate.get("region") != "middle" or
                    candidate.get("role") != "line_endpoint"):
                continue
            candidate_coordinate = points[candidate_number]
            if (
                    abs(abs(candidate_coordinate[0] - centre_coordinate[0]) -
                        half_side) <= 1e-6 and
                    abs(abs(candidate_coordinate[1] - centre_coordinate[1]) -
                        half_side) <= 1e-6):
                guard[candidate_number] = candidate_coordinate
        if len(guard) != 4:
            raise TaskDefinitionError(
                "middle target %d has %d guard endpoints, expected 4" %
                (target, len(guard)))
        guards[target] = guard
    return guards


def require_points(points, numbers):
    """Return the requested points, rejecting a mission with missing labels."""
    resolved = {}
    for raw_number in numbers:
        number = int(raw_number)
        if number not in points:
            raise TaskDefinitionError("grid point %d is missing" % number)
        resolved[number] = points[number]
    return resolved
